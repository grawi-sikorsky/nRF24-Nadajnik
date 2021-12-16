#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <time.h>
#include <digitalWriteFast.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include "LowPower.h"
#include "EEPROM.h"

#include "configuration.h"
#include "nadajnik.h"

//**************************
//  NADAJNIK GWIZDEK
//**************************
Nadajnik nadajnik;
RF24 radio(8, 9); // CE, CSN

struct outdata
{
  int     sendgwizd = 2;
  float   raw;
  float   avg;
};
outdata nrfdata;

//#define DEBUGSERIAL
//#define DEBUG
//#define UNO

/*****************************************************
 * Przerwanie dla przycisku
 * ***************************************************/
void isr_button() {
  nadajnik.ButtonPressed();
}



/*****************************************************
 * ARDUINO SETUP
 * ***************************************************/
void setup() 
{
  nadajnik.uc_state = nadajnik.UC_GO_SLEEP; // default uC state

  // wylacz WDT
  MCUSR= 0 ;
  WDTCSR |= _BV(WDCE) | _BV(WDE);
  WDTCSR = 0;

  ADCSRA &= ~(1 << 7); // TURN OFF ADC CONVERTER
  //power_timer0_disable();// TIMER 0 SLEEP WDT ...
  power_timer1_disable();// Timer 1
  power_timer2_disable();// Timer 2
  power_twi_disable(); // TWI (I2C)
  power_adc_disable(); // ADC converter
  
  #ifdef UNO
    power_usart0_enable();// Serial (USART) test
  #else
    power_usart0_disable();
  #endif
  #ifdef DEBUGSERIAL
    Serial.begin(115200);
  #endif

  for (byte i = 0; i <= A5; i++)
  {
    pinModeFast(i, OUTPUT);
    digitalWriteFast(i, LOW);
  }

  pinModeFast(LED_PIN,OUTPUT);
  pinModeFast(BUTTON_PIN,INPUT);
  digitalWriteFast(LED_PIN, LOW);  // LED OFF

  pinModeFast(SS,OUTPUT);
  pinModeFast(MOSI,OUTPUT);
  pinModeFast(MISO,OUTPUT);
  pinModeFast(SCK,OUTPUT);
  digitalWriteFast(SS,HIGH);
  digitalWriteFast(MOSI,HIGH);
  digitalWriteFast(MISO,HIGH);
  digitalWriteFast(SCK,HIGH);

  nadajnik.setAddress( EEPROM.read(EEPROM_ADDRESS_PLACE) ); // odczytaj z eeprom wartosc przed inicjalizacja nrfki

  nadajnik.init();

  // blink LED
  for(int i=0; i<6; i++)
  {
    digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
    delay(40);
  }
}

/*****************************************************
 * LOOP
 * ***************************************************/
void loop() {
  switch(nadajnik.uc_state)
  {
    case nadajnik.UC_GO_SLEEP:
    {
      if (nadajnik.getGoToLongsleep() == true)  // dluga kima
      {
        nadajnik.prepareToSleep(); // wylacza zbedne peryferia na czas snu
        attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), isr_button, RISING); // przerwanie sw
        LowPower.powerDown(SLEEP_FOREVER,ADC_OFF,BOD_OFF);
        // here sleeping ->
        nadajnik.wakeUp();
        nadajnik.setBmeTableNeedsInitialization(true);
      }
      else    // krotka kima
      {
        nadajnik.prepareToSleep(); // wylacza zbedne peryferia na czas snu
        LowPower.powerDown(nadajnik.getSleepTime(), ADC_OFF, BOD_OFF);
        nadajnik.wakeUp();

        nadajnik.uc_state = nadajnik.UC_WAKE_AND_CHECK; // pokimal to sprawdzic co sie dzieje->
      }
      break;
    }
    case nadajnik.UC_WAKE_AND_CHECK:
    {
        nadajnik.getCurrentTime();
        nadajnik.pressureRead();
        nadajnik.managePressure();
        nadajnik.checkPressure();

        nrfdata.raw = nadajnik.getBmeRawData();
        nrfdata.avg = nadajnik.getBmeAverage();

        if(nadajnik.getSendSignalState() && !nadajnik.getPauseAfterGwizd())
        {
          nadajnik.SendRFData();
          nadajnik.setRepeatSending(0);
          nadajnik.setPauseAfterGwizd(true);
        }
        else
        {
          if ( nadajnik.getRepeatSending() < RF_REPEAT )
          {
            nadajnik.SendRFData();
            nadajnik.setRepeatSending( nadajnik.getRepeatSending()+1 );
          }
          else{
            nadajnik.setSendSignal(false);
          }
        }
        nadajnik.uc_state = nadajnik.UC_BTN_CHECK;
      break;
    }
    case nadajnik.UC_BTN_CHECK:
    {
      nadajnik.setGoToLongsleep(false);   // inaczej pojdzie spac long, device_is_off pozostaje dla ustalenia czy nadajnik był wybudzony czy pracowal normalnie

      nadajnik.manageButton();

      break;
    }
  }
  nadajnik.manageTimeout();
}