#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <time.h>
#include <digitalWriteFast.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>

// SLEEP LIB
#include "LowPower.h"

// PINY
#define LED_PIN         5
#define SPEAKER_PIN     7 //A2 // 7 minipro
#define TRANSMISION_PIN 0 // 4 w proto
#define USER_SWITCH     2 // PD2 INT0

#define SWITCH_TIMEOUT  3000          // czas przycisku nacisniecia
#define SW_RST_TIMEOUT  100          // czas w ktorym należy wykonac klikniecia dla RST
#define SW_RST_COUNT    5             // ilosc nacisniec do wykonania resetu
#define TIME_TO_WAIT_MS 50            // czas do nastepnego wyzwolenia????
#define TIMEOUT_1       72000       // pierwszy timeiut // realnie wychodzi jakies (1 800 000 ms = 30 min) / 25 = 72000
#define TIMEOUT_2       144000       // drugi prog = 5 400 000 = 90 min // z uwagi na sleep-millis: 60 min

#define RF_SENDBACK     25          // ilosc transmisji nadawczych do odbioru powrotnej (musi byc identyczna z Odbiornikiem!)

//#define DEBUGMODE

// BME280 LIB
#define TINY_BME280_SPI
#include "TinyBME280.h"
tiny::BME280 bme1; //Uses I2C address 0x76 (jumper closed)

struct outdata
{
  float bme_data;   // cisnienie z nadajnika
  int   i_send;      // licznik z nadajnika
  bool  slowtime;   // zwolnij odswiezanie
  bool  sleeptime;  // uspij nadajnik
  bool  reset;
};
outdata nrfdata;

//float bme_data;

enum uc_State {
  UC_GO_SLEEP = 0,
  UC_WAKE_AND_CHECK = 1,
  UC_WAITING_FOR_SENDBACK = 2,
  UC_BTN_CHECK  = 3,
};
uc_State uc_state;

RF24 radio(9, 10); // CE, CSN
const byte address[17] = "1100110011001100";
int i_receive = 0;
int received_state;
bool transmit_done;

period_t sleeptime;
time_t current_time, last_time;
time_t btn_current, btn_pressed_time, btn_timeout, last_rst_click;
bool btn_state = LOW;
bool btn_last_state = LOW;
int btn_rst_counter = 0;

bool device_in_longsleep = false;
bool delegate_to_longsleep = false;


/*****************************************************
 * Obsluga przerwania przycisku
 * ***************************************************/
void ButtonPressed()
{
  if(device_in_longsleep == true)  // jesli urzadzenie jest wylaczone
  {
    btn_pressed_time = millis();

    // wlacz i przejdz do sprawdzenia stanu przycisku
    //uc_state = UC_BTN_CHECK;
  }
}

/*****************************************************
 * Przerwanie dla przycisku
 * ***************************************************/
void ISR_INT0_vect()
{
  ButtonPressed();
}

/*****************************************************
 * Wylacza wszystko do spania
 * ***************************************************/
void prepareToSleep()
{
  //clock_prescale_set(clock_div_16);

  ADCSRA &= ~(1 << 7); // TURN OFF ADC CONVERTER

  power_adc_disable(); // ADC converter
  power_spi_disable(); // SPI
  #ifdef DEBUGMODE
    power_usart0_enable();// Serial (USART) test
  #else
    power_usart0_disable();
  #endif
  //power_timer0_disable();// TIMER 0 SLEEP WDT ...
  power_timer1_disable();// Timer 1
  power_timer2_disable();// Timer 2

  PORTD &= ~(1 << PD0);   // LOW pin0 CMT2110

  //radio.powerDown();
}

/*****************************************************
 * SW reset - dont reset peripherials
 * ***************************************************/
void softReset(){
  //asm volatile ("  jmp 0");
  cli(); //irq's off
  wdt_enable(WDTO_60MS); //wd on,15ms
  while(1); //loop
}

/*********************************************************************
 * PO ODEBRANIU INFO Z ODBIORNIKA USTAWIA CZAS PROBKOWANIA I SPANIA
 * *******************************************************************/
void manageTimeout()
{
  if(nrfdata.slowtime == true)
  {
    delegate_to_longsleep == false;
    sleeptime = SLEEP_250MS;    // zmniejszone probkowanie
  }
  else if(nrfdata.sleeptime == true)
  {
    delegate_to_longsleep == true;
    sleeptime = SLEEP_FOREVER;    // spanie na amen
  }
  else
  {
    delegate_to_longsleep == false;
    sleeptime = SLEEP_120MS;    // domyslne
  }
}

/*************************************************************************************
 * Odczytuje odpowiedz z odbiornika o tym czy i kiedy nadajnik ma sie wylaczyc
 * ***********************************************************************************/
void read_answer_from_receiver()
{
  if(nrfdata.i_send == RF_SENDBACK)     // odczyt z odbiornika raz na 4 transmisje nadawcze
  {
    radio.openReadingPipe(0, address);  // przestawiamy na odbior
    radio.startListening();             // nasluchujemy

    if(radio.available())               // jesli cos jest
    {
      radio.read(&nrfdata, sizeof(nrfdata));  // czytamy

      //nrfdata.i_send = 0;    // zeruje to odbiornik.

      #ifdef DEBUGMODE
        Serial.println(nrfdata.i_send); //delayMicroseconds(400);
        Serial.println(nrfdata.slowtime); delayMicroseconds(400);
        Serial.println(nrfdata.sleeptime); delayMicroseconds(400);
      #endif
      
      //uc_state = UC_BTN_CHECK;
    }
  }
  else
  {
    //uc_state = UC_BTN_CHECK;
  }
}

void set_back_to_transmit()
{
  radio.openWritingPipe(address);       // przestawiamy sie na transmisje
  radio.stopListening();                // konczymy nasluch
}

void setup() {
  //clock_prescale_set(clock_div_1);
  
  // wylacz WDT
  MCUSR= 0 ;
  WDTCSR |= _BV(WDCE) | _BV(WDE);
  WDTCSR = 0;

  ADCSRA &= ~(1 << 7); // TURN OFF ADC CONVERTER
  power_adc_disable(); // ADC converter
  //power_spi_disable(); // SPI
  #ifdef DEBUGMODE
    power_usart0_enable();// Serial (USART) test
  #else
    power_usart0_disable();
  #endif
  //power_timer0_disable();// TIMER 0 SLEEP WDT ...
  power_timer1_disable();// Timer 1 - I2C...
  power_timer2_disable();// Timer 2

  PORTD &= ~(1 << PD0);   // LOW pin0 CMT2110


  for (byte i = 0; i <= A5; i++)
  {
    pinModeFast(i, OUTPUT);    // changed as per below
    digitalWriteFast(i, LOW);  //     ditto
  }

  pinModeFast(LED_PIN,OUTPUT);
  pinModeFast(SPEAKER_PIN,OUTPUT);
  pinModeFast(TRANSMISION_PIN,OUTPUT);
  pinModeFast(USER_SWITCH,INPUT);
  digitalWriteFast(LED_PIN, LOW);  // LED OFF
  digitalWriteFast(SPEAKER_PIN, LOW);    // SPK
  digitalWriteFast(TRANSMISION_PIN, LOW);    // RF433

  pinModeFast(SS,OUTPUT);
  pinModeFast(MOSI,OUTPUT);
  pinModeFast(MISO,OUTPUT);
  pinModeFast(SCK,OUTPUT);
  digitalWriteFast(SS,HIGH);
  digitalWriteFast(MOSI,HIGH);
  digitalWriteFast(MISO,HIGH);
  digitalWriteFast(SCK,HIGH);

  uc_state = UC_GO_SLEEP; // default uC state

  bme1.beginSPI(8);
  //Serial.begin(115200);
  
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();

  nrfdata.reset = true;     //
  nrfdata.bme_data = bme1.readFixedPressure();        // Odczyt z czujnika bme
  radio.write(&nrfdata, sizeof(nrfdata));   // Wyslij dane przez nRF
  nrfdata.reset = false;
  for(int i=0; i<8; i++)
  {
    digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
    delay(100);
  }
}

void loop() {
  switch(uc_state)
  {
    case UC_GO_SLEEP:
    {
      if (delegate_to_longsleep == true)  // dluga kima
      {
        //Serial.println("deepsleep"); delayMicroseconds(950);
        prepareToSleep(); // wylacza zbedne peryferia na czas snu
        attachInterrupt(digitalPinToInterrupt(2), ISR_INT0_vect, RISING); // przerwanie sw
        LowPower.powerDown(SLEEP_FOREVER,ADC_OFF,BOD_OFF);
      }
      else    // krotka kima
      {
        //radio.powerDown();
        //Serial.println("sleep"); delayMicroseconds(550);
        prepareToSleep(); // wylacza zbedne peryferia na czas snu
        LowPower.powerDown(sleeptime,ADC_OFF,BOD_OFF);
        interrupts();
        power_spi_enable(); // SPI
        //radio.powerUp();
        
        uc_state = UC_WAKE_AND_CHECK; // pokimal to sprawdzic co sie dzieje->
      }
      break;
    }
    case UC_WAKE_AND_CHECK:
    {
      set_back_to_transmit();

      nrfdata.bme_data = bme1.readFixedPressure();        // Odczyt z czujnika bme
      radio.write(&nrfdata, sizeof(nrfdata));   // Wyslij dane przez nRF

      //Serial.print("BME nadano:");                // debug
      //Serial.println(nrfdata.bme_data);           // debug

      if(nrfdata.i_send == RF_SENDBACK)
      {
        uc_state = UC_WAITING_FOR_SENDBACK;
      }
      else
      {
        nrfdata.i_send++;                           // licznik nadan
      }
      //break;
    }
    case UC_WAITING_FOR_SENDBACK:
    {
      read_answer_from_receiver();
      break;
    }
    case UC_BTN_CHECK:
    {
      btn_last_state = btn_state;               // do rst
      btn_state = digitalReadFast(USER_SWITCH); // odczyt stanu guzika
      
      current_time = millis();

      if(btn_state != btn_last_state) // jezeli stan przycisku sie zmienil
      {
        if(btn_state == HIGH)         // jezeli jest wysoki
        {
          btn_rst_counter++;          // licznik klikniec ++
          last_rst_click = current_time;  // zeruj timeout
        }
        if(current_time - last_rst_click >= SW_RST_TIMEOUT)
        {
          btn_rst_counter = 0;
          last_rst_click = current_time;
        }
        if(btn_rst_counter >= SW_RST_COUNT)
        {
          for(int i=0; i<12; i++)
          {
            digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
            delay(100);
          }
          //softReset();
        }
      }

      // jesli przycisk nie jest wcisniety lub zostal zwolniony
      if(btn_state == LOW)
      {
        btn_pressed_time = current_time; // ustaw obecny czas
        // jesli przycisk zostanie nacisniety ostatnia wartość stad nie bedzie nadpisywana
      }

      // jesli przycisk nie jest wcisniety gdy urzadzenie pracuje -> loop
      if(btn_state == LOW && device_in_longsleep == false)
      {
        btn_pressed_time = current_time; // to wlasciwie mozna usunac na rzecz tego na gorze?
        // i od razu w krotka kime
        uc_state = UC_GO_SLEEP;
        break;
      }

      // jesli sie obudzi po przerwaniu a przycisk juz nie jest wcisniety -> deepsleep
      if(btn_state == LOW && device_in_longsleep == true)
      {
        device_in_longsleep = true;           // flaga off dla pewnosci
        delegate_to_longsleep = true;   // deleguj do glebokiego snu
        uc_state = UC_GO_SLEEP;
      }

      // jesli przycisk wcisniety gdy urzadzenie bylo wylaczone:
      if(btn_state == HIGH && device_in_longsleep == true) // jesli guzik + nadajnik off
      {
        if(current_time - btn_pressed_time >= SWITCH_TIMEOUT)
        {
          // pobudka
          btn_pressed_time = current_time;
          digitalWriteFast(LED_PIN,HIGH);
          delay(1000);
          digitalWriteFast(LED_PIN,LOW);

          device_in_longsleep = false;

          // po dlugim snie moze przy checktimeout wpasc znow w deepsleep
          // dlatego last positive = teraz
          // last_positive = current_time; 

          detachInterrupt(digitalPinToInterrupt(2));
          uc_state = UC_WAKE_AND_CHECK;
        }
        //uc_state = UC_GO_SLEEP;// nowe - przemyslec!
      }

      // jesli przycisk wcisniety a urzadzenie pracuje normalnie:
      else if(btn_state == true && device_in_longsleep == false) // guzik + nadajnik ON
      {
        if(current_time - btn_pressed_time >= SWITCH_TIMEOUT)
        {
          // spij
          device_in_longsleep = true;
          delegate_to_longsleep = true;          
          digitalWriteFast(LED_PIN,HIGH);
          delay(400);
          digitalWriteFast(LED_PIN,LOW);
          delay(400);
          digitalWriteFast(LED_PIN,HIGH);
          delay(400);
          digitalWriteFast(LED_PIN,LOW);
          uc_state = UC_GO_SLEEP;
        }
      }
      else
      {
        // yyyyy...
      }

      break;
    }
  }
  manageTimeout();
  digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
  delay(25);
}