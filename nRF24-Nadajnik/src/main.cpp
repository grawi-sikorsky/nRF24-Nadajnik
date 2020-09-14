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

// BME280 LIB
#define TINY_BME280_SPI
#include "TinyBME280.h"
tiny::BME280 bme1; //Uses I2C address 0x76 (jumper closed)
float bme_data;

enum uc_State {
  UC_GO_SLEEP = 0,
  UC_WAKE_AND_CHECK = 1,
  UC_BTN_CHECK  = 2,
};
uc_State uc_state;

RF24 radio(9, 10); // CE, CSN
const byte address[17] = "1100110011001100";
int data = 0;

time_t btn_current, btn_pressed_at, btn_timeout;
bool btn_state = LOW;
bool btn_last_state = LOW;
int btn_rst_counter = 0;

bool device_is_off = false;
bool delegate_to_longsleep = false;


/*****************************************************
 * Obsluga przerwania przycisku
 * ***************************************************/
void ButtonPressed()
{
  if(device_is_off == true)  // jesli urzadzenie jest wylaczone
  { 
    btn_pressed_at = millis();

    // wlacz i przejdz do sprawdzenia stanu przycisku
    uc_state = UC_BTN_CHECK;
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
    power_usart0_disable();// Serial (USART) test
  #else
    power_usart0_disable();
  #endif
  //power_timer0_disable();// TIMER 0 SLEEP WDT ...
  power_timer1_disable();// Timer 1
  power_timer2_disable();// Timer 2

  PORTD &= ~(1 << PD0);   // LOW pin0 CMT2110
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


void setup() {
  clock_prescale_set(clock_div_1);
  
  // wylacz WDT
  MCUSR= 0 ;
  WDTCSR |= _BV(WDCE) | _BV(WDE);
  WDTCSR = 0;

  ADCSRA &= ~(1 << 7); // TURN OFF ADC CONVERTER
  power_adc_disable(); // ADC converter
  //power_spi_disable(); // SPI
  power_usart0_disable();// Serial (USART)
  //power_timer0_disable();// TIMER 0 SLEEP WDT ...
  power_timer1_disable();// Timer 1 - I2C...
  power_timer2_disable();// Timer 2

  PORTD &= ~(1 << PD0);   // LOW pin0 CMT2110

  for (byte i = 0; i <= A5; i++)
  {
    pinModeFast(i, OUTPUT);    // changed as per below
    digitalWriteFast(i, LOW);  //     ditto
  }

  //pinModeFast(LED_PIN,OUTPUT);
  //pinModeFast(SPEAKER_PIN,OUTPUT);
  //pinModeFast(TRANSMISION_PIN,OUTPUT);
  //pinModeFast(USER_SWITCH,INPUT);
  //digitalWriteFast(LED_PIN, LOW);  // LED OFF
  //digitalWriteFast(SPEAKER_PIN, LOW);    // SPK
  //digitalWriteFast(TRANSMISION_PIN, LOW);    // RF433

  pinModeFast(SS,OUTPUT);
  pinModeFast(MOSI,OUTPUT);
  pinModeFast(MISO,OUTPUT);
  pinModeFast(SCK,OUTPUT);
  digitalWriteFast(SS,HIGH);
  digitalWriteFast(MOSI,HIGH);
  digitalWriteFast(MISO,HIGH);
  digitalWriteFast(SCK,HIGH);

  power_timer1_enable();  // Timer 1 - I2C...        
  //readValues();               // pierwsze pobranie wartosci - populacja zmiennych
  //prev_press = press_odczyt; // jednorazowe na poczatku w setup
  //setupTimer1();              // Ustawia timer1
  power_timer1_disable(); // Timer 1 - I2C...

  //makeMsg();                  // Przygotowuje ramke danych

  //last_blink = millis();

  //startup = false;
  //was_whistled = false;
  //uc_state = UC_GO_SLEEP; // default uC state


  bme1.beginSPI(8);
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
}

void loop() {
  switch(uc_state)
  {
    case UC_GO_SLEEP:
    {
      if (delegate_to_longsleep == true)  // dluga kima
      {
        prepareToSleep(); // wylacza zbedne peryferia na czas snu
        attachInterrupt(digitalPinToInterrupt(2), ISR_INT0_vect, RISING); // przerwanie sw
        LowPower.powerDown(SLEEP_FOREVER,ADC_OFF,BOD_OFF);
      }
      else    // krotka kima
      {
        prepareToSleep(); // wylacza zbedne peryferia na czas snu
        LowPower.powerDown(SLEEP_120MS,ADC_OFF,BOD_OFF);
        interrupts();
        uc_state = UC_WAKE_AND_CHECK; // pokima�? to sprawdzi� co sie dzieje->
      }
      
      break;
    }
    case UC_WAKE_AND_CHECK:
    {

      break;
    }
    case UC_BTN_CHECK:
    {

      break;
    }
  }
  bme_data = bme1.readFixedPressure();  // Odczyt z czujnika bme
  Serial.print("BME nadano:");
  Serial.println(bme_data);



  radio.write(&bme_data, sizeof(bme_data));
  delay(200);
}