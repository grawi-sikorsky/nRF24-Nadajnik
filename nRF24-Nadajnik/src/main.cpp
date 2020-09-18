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

// PRESSURE DEFINICJE I ZMIENNE
#define BME_AVG_COUNT 20      // wiecej -> dluzszy powrot avg do normy
#define BME_AVG_DIFF  800     // im mniej tym dluzej wylacza sie po dmuchaniu. Zbyt malo powoduje ze mimo wylaczenia sie gwizdka, wlacza sie ponownie gdy wartosci wracaja do normy i avg.
#define BME_AVG_SENS  200     // czulosc dmuchniecia
float bme_raw;                // dane raw z BME280
float bme_tbl[BME_AVG_COUNT]; // tablica z probkami cisnienia 
float bme_avg = 0;            // srednie cisnienie -> bme_avg / BME_AVG_COUNT
int   bme_avg_i = 0;          // licznik AVG
bool  bme_rozbieg = true;     // info o pierwszym wypelnianiu tabeli AVG
bool  gwizd_on    = false;    // info o aktywnym gwizdku
bool  no_gwizd    = false;    // przestawia transmisje na 'nieaktywna' = 2 w kolejnej petli
time_t gwizd_start_at;        // timeout gwizd


RF24 radio(8, 9); // CE, CSN
const byte address[5] = "Odb1";  // domyslny adres odbiornika
bool  whistle_connected = false;

period_t sleeptime = SLEEP_120MS;
time_t current_time;
time_t btn_current, btn_pressed_time, btn_timeout, last_rst_click;
bool btn_state = LOW;
bool btn_last_state = LOW;
int btn_rst_counter = 0;

bool device_in_longsleep = false;
bool delegate_to_longsleep = false;

struct outdata
{
  //int     ID_nadajnika;
  int     sendgwizd;
  //time_t  odbiornik_gwizd_time_at;
};
outdata nrfdata;

bool ackOK;

enum uc_State {
  UC_GO_SLEEP = 0,
  UC_WAKE_AND_CHECK = 1,
  UC_WAITING_FOR_SENDBACK = 2,
  UC_BTN_CHECK  = 3,
};
uc_State uc_state;

#define DEBUGSERIAL
//#define DEBUG
#define UNO

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
  #ifdef UNO
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

// ODCZYTUJE Z BME
void pressure_read()
{
  bme_raw = bme1.readFixedPressure();        // Odczyt z czujnika bme
  #ifdef DEBUGSERIAL
    //Serial.println(bme_raw);
  #endif
}

/*********************************************************************
 * FUNKCJA ROZBIEGOWA TABLICY Z PROBKAMI CISNIENIA
 * URUCHAMIANA RAZ PO DLUZSZEJ (RF_OFF_TIME) NIEAKTYWNOSCI NADAJNIKA
 * *******************************************************************/
void pressure_prepare()
{
  // START ODBIORU I USREDNIANIE DANYCH
  for(int i=0; i < BME_AVG_COUNT; i++)  // w rozbiegu usredniaj wraz z rosnacym licznikiem i.
  {
    bme_tbl[i] = bme_raw;      // przypisz dane z nadajnika x AVG COUNT
    bme_avg += bme_tbl[i];              // dodaj do sredniej wartosc z tablicy[i]
  }
  bme_avg = bme_avg / BME_AVG_COUNT;    // dzielimy przez ilosc zapisanych wartosci w tablicy
  bme_rozbieg = false;
  bme_avg_i = 0;

  #ifdef DEBUGSERIAL
    Serial.println(bme_avg);
  #endif
}

/*********************************************************************
 * ODBIOR DANYCH Z NADAJNIKA I USREDNIENIE WYNIKU DO DALSZYCH DZIALAN
 * URUCHAMIANE TYLKO KIEDY NADAJNIK ODBIERA BME
 * *******************************************************************/
void manage_pressure()
{
  // ROZBIEG TABLICY SREDNIEGO CISNIENIA
  if(bme_rozbieg == true)
  {
    pressure_prepare();
    #ifdef DEBUGSERIAL
      Serial.println("pressure prepare");
    #endif
  }

  // START USREDNIANIA DANYCH
  if(bme_avg_i < BME_AVG_COUNT)
  {
    if((bme_raw < (bme_avg + BME_AVG_DIFF)) && (bme_raw > (bme_avg - BME_AVG_DIFF)))  // jesli obecne probka miesci sie w widelkach +-[BME_AVG_DIFF] 
    {
      bme_tbl[bme_avg_i] = bme_raw;  // dodaj nowa wartosc do tabeli
      bme_avg_i++;                    // zwieksz licznik
    }
  }
  else
  {
    bme_avg_i = 0;
  }
    unsigned long start = micros();
    // Call to your function

  bme_avg = 0;                          // zeruj srednia przed petla
  for(int i=0; i <= BME_AVG_COUNT; i++) // Usredniaj zgodnie z iloscia probek [BME_AVG_COUNT]
  {
    bme_avg += bme_tbl[i];              // dodaj do sredniej wartosc z tablicy[i]
  }
  bme_avg = bme_avg / BME_AVG_COUNT;    // dzielimy przez ilosc zapisanych wartosci w tablicy

    // Compute the time it took
    unsigned long end = micros();
    unsigned long delta = end - start;
  Serial.print("czas: ");Serial.println(delta);
}

/*********************************************************************
 * SPRAWDZA CZY NASTAPIL WZROST CISNIENIA W STOSUNKU DO SREDNIEJ AVG
 * CZULOSC BME_AVG_SENS
 * *******************************************************************/
void check_pressure()
{
  if(no_gwizd == true)
  {
    nrfdata.sendgwizd = 2;
  }
  if(bme_raw > (bme_avg + BME_AVG_SENS))             // JESLI NOWA PROBKA JEST WIEKSZA OD SREDNIEJ [AVG + AVG_DIFF]
  {
    #ifdef DEBUGSERIAL
      Serial.println("GWIZD ON");
    #endif

    gwizd_on = true;                                  // ustaw gwizdek aktywny
    no_gwizd = false;
    nrfdata.sendgwizd = 1;
    gwizd_start_at = millis();                        // ustaw czas ostatniego gwizdniecia
  }
  else if((bme_raw < (bme_avg + BME_AVG_DIFF)) && (bme_raw > (bme_avg - BME_AVG_DIFF)) && gwizd_on == true)   // JESLI CISNIENIE WRACA DO WIDELEK [AVG +- AVG_DIFF] a gwizdek jest aktywny
  {
    #ifdef DEBUGSERIAL
      Serial.println("Gwizd OFF");
    #endif
    nrfdata.sendgwizd = 0;
    gwizd_on = false;                                 // flaga gwizdka rowniez OFF
    no_gwizd = true;
  }
}

// PRZESTAWIA NADAJNIK NA TRANSMISJE
void set_to_transmit()
{
  radio.openWritingPipe(address);       // przestawiamy sie na transmisje
  radio.stopListening();                // konczymy nasluch
}

// PRZESTAWIA NADAJNIK NA ODBIOR
void set_to_reading()
{
  radio.openReadingPipe(0, address);
  radio.startListening();
}

// PIERWSZA TRANSMISJA Z ODBIOREM ID OD ODBIORNIKA
// Przypisuje ID z odbiornika do nrfdata
// ustawia flage whistle_connected = true
bool SendRFData()
{
  bool result;
  result = radio.write(&nrfdata, sizeof(nrfdata));   // PIERWSZA TRANSMISJA DO ODBIORNIKA!

  if(result)
  {
    if ( radio.isAckPayloadAvailable() ) 
    {
      radio.read(&ackOK, sizeof(ackOK));
      whistle_connected = true;
      Serial.print("ACK:"); Serial.println(ackOK);
    }
    else
    {
      whistle_connected = false;
      Serial.print("ACK OK, no data");
    }
  }
  else
  {
    whistle_connected = false;
    Serial.print("NO ACK");
  }

  return whistle_connected;
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
  #ifdef UNO
    power_usart0_enable();// Serial (USART) test
  #else
    power_usart0_disable();
  #endif
  //power_timer0_disable();// TIMER 0 SLEEP WDT ...
  power_timer1_disable();// Timer 1 - I2C...
  power_timer2_disable();// Timer 2

  PORTD &= ~(1 << PD0);   // LOW pin0 CMT2110

  #ifndef UNO
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
  #endif

  uc_state = UC_GO_SLEEP; // default uC state

  bme1.beginSPI(10); // 8 w uno

  #ifdef DEBUGSERIAL
    Serial.begin(115200);
  #endif
  
  radio.begin();
  radio.openWritingPipe(address);
  radio.enableAckPayload();
  radio.setRetries(1,8); // delay, count
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(95);
  radio.stopListening();

  pressure_read();      // odczyt z bme
  pressure_prepare();   // rozbieg tablicy avg

  //setup_whistle_ID();   // pierwsza transmisja z odbiorem danych z odbiornika z przekazaniem wlasciwego ID.

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
        #ifdef DEBUGSERIAL
          Serial.println("longsleep");
        #endif
        prepareToSleep(); // wylacza zbedne peryferia na czas snu
        attachInterrupt(digitalPinToInterrupt(2), ISR_INT0_vect, RISING); // przerwanie sw
        LowPower.powerDown(SLEEP_FOREVER,ADC_OFF,BOD_OFF);
      }
      else    // krotka kima
      {
        #ifdef DEBUGSERIAL
          //Serial.println("shortsleep"); delayMicroseconds(850);
        #endif
        prepareToSleep(); // wylacza zbedne peryferia na czas snu
        LowPower.powerDown(SLEEP_120MS,ADC_OFF,BOD_OFF);
        interrupts();
        power_spi_enable(); // SPI

        #ifndef UNO
          digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
          delay(5);
          //digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
          //delay(25);
        #endif

        uc_state = UC_WAKE_AND_CHECK; // pokimal to sprawdzic co sie dzieje->
      }
      //break;
    }
    case UC_WAKE_AND_CHECK:
    {
        pressure_read();
        manage_pressure();
        check_pressure();

        #ifdef DEBUGSERIAL
          Serial.print("RAW: "); Serial.println(bme_raw);
          Serial.print("AVG: "); Serial.println(bme_avg);
        #endif

        if(SendRFData() == false)
        {
          //delayMicroseconds(1500);

          if(SendRFData() == true) break;
          //else SendRFData();
        
        }
        
      uc_state = UC_GO_SLEEP;
      break;
    }
    case UC_WAITING_FOR_SENDBACK:
    {
      //read_answer_from_receiver();
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
  //manageTimeout();
}