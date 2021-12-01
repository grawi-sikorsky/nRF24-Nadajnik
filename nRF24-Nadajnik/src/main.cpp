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

time_t gwizd_start_at, giwzd_timeout;        // timeout gwizd

period_t sleeptime = SLEEP_120MS;
time_t current_time;
time_t btn_current, btn_pressed_time, btn_timeout, last_rst_click;
time_t start_click_addr;
bool btn_state = LOW;
bool btn_last_state = LOW;
int btn_rst_counter = 0;

struct outdata
{
  int     sendgwizd = 2;
  float   raw;
  float   avg;
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
 * SW reset - dont reset peripherials
 * ***************************************************/
void(* resetFunc) (void) = 0; //declare reset function at address 0


/*****************************************************
 * ARDUINO SETUP
 * ***************************************************/
void setup() 
{
  uc_state = UC_GO_SLEEP; // default uC state

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
  switch(uc_state)
  {
    case UC_GO_SLEEP:
    {
      if (nadajnik.getGoToLongsleep() == true)  // dluga kima
      {
        #ifdef DEBUGSERIAL
          Serial.println("longsleep"); delay(20);
        #endif
        nadajnik.prepareToSleep(); // wylacza zbedne peryferia na czas snu
        attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), isr_button, RISING); // przerwanie sw
        LowPower.powerDown(SLEEP_FOREVER,ADC_OFF,BOD_OFF);
        nadajnik.wakeUp();
        nadajnik.setBmeTableNeedsInitialization(true);
      }
      else    // krotka kima
      {
        #ifdef DEBUGSERIAL
          //Serial.println("shortsleep"); delayMicroseconds(850);
        #endif
        nadajnik.prepareToSleep(); // wylacza zbedne peryferia na czas snu
        LowPower.powerDown(sleeptime,ADC_OFF,BOD_OFF);
        nadajnik.wakeUp();

        uc_state = UC_WAKE_AND_CHECK; // pokimal to sprawdzic co sie dzieje->
      }
      break;
    }
    case UC_WAKE_AND_CHECK:
    {
        current_time = millis();
        nadajnik.pressureRead();
        nadajnik.managePressure();
        nadajnik.checkPressure();

        #ifdef DEBUGSERIAL
          //Serial.print("nrfgwi: "); Serial.println(nrfdata.sendgwizd);
          Serial.print("AVG: "); Serial.println(bme_avg);
        #endif
        nrfdata.raw = bme_raw;
        nrfdata.avg = bme_avg;

        if(gwizd_on)
        {
          nadajnik.SendRFData();
          rf_repeat = 0;
        }
        else
        {
          if ( rf_repeat < RF_REPEAT )
          {
            nadajnik.SendRFData();
            rf_repeat++;
          }
        }
        uc_state = UC_BTN_CHECK;
      break;
    }
    case UC_WAITING_FOR_SENDBACK:
    {
      break;
    }
    case UC_BTN_CHECK:
    {
      nadajnik.SetGoToLongsleep(false);  // inaczej pojdzie spac long
                                      // device_is_off pozostaje dla ustalenia czy 
                                      // nadajnik był wybudzony czy pracowal normalnie

      btn_last_state = btn_state;               // do rst
      btn_state = digitalReadFast(BUTTON_PIN); // odczyt stanu guzika
      
      current_time = millis();
      
      if(btn_state != btn_last_state) // jezeli stan przycisku sie zmienil
      {
        if(btn_state == HIGH)         // jezeli jest wysoki
        {
          btn_rst_counter++;          // licznik klikniec ++
          last_rst_click = current_time;  // zeruj timeout
          start_click_addr = current_time;  // ustaw start klikniecia do zmiany adresu
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
          resetFunc(); //call reset
        }
      }

      // CZEK CZY KLIK I DMUCHNIECIE ADDRESS

        // do parowania z odbiornikiem!
        // sprawdzamy czy po 1s od nacisniecia gwidka nie pojawilo sie dmuchniecie - jesli tak: parowanie.
      if(current_time - start_click_addr >= 850 && current_time - start_click_addr <= 2400)
      {
        if(btn_state == LOW)
        {
          // funkcja parowania z odbiornikiem!
          if(nadajnik.getAddress() < 7) { nadajnik.setAddress( nadajnik.getAddress()+1 ); }
          else { nadajnik.setAddress(0); }

          radio.openWritingPipe(addressList[ nadajnik.getAddress() ]);
          EEPROM.update(EEPROM_ADDRESS_PLACE, nadajnik.getAddress());

          for (int i = 0; i < ((nadajnik.getAddress()+1)*2); i++)
          {
            digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
            delay(200);
          }
          start_click_addr = current_time = millis();
        }
      }

      // jesli przycisk nie jest wcisniety lub zostal zwolniony
      if(btn_state == LOW)
      {
        btn_pressed_time = current_time;  // ustaw obecny czas
        start_click_addr = current_time;  // zeruj start timer dla zmiany adresu
        // jesli przycisk zostanie nacisniety ostatnia wartość stad nie bedzie nadpisywana
      }

      // jesli przycisk nie jest wcisniety gdy urzadzenie pracuje -> loop
      if(btn_state == LOW && nadajnik.getGoToLongsleep() == false)
      {
        btn_pressed_time = current_time; // to wlasciwie mozna usunac na rzecz tego na gorze?
        // i od razu w krotka kime
        uc_state = UC_GO_SLEEP;
        break;
      }

      // jesli sie obudzi po przerwaniu a przycisk juz nie jest wcisniety -> deepsleep
      if(btn_state == LOW && nadajnik.isLongsleep() == true)
      {
        nadajnik.setToLongsleep(true);           // flaga off dla pewnosci
        nadajnik.setGoToLongsleep(true);   // deleguj do glebokiego snu
        uc_state = UC_GO_SLEEP;
      }

      // jesli przycisk wcisniety gdy urzadzenie bylo wylaczone:
      if(btn_state == HIGH && nadajnik.isLongsleep() == true) // jesli guzik + nadajnik off
      {
        if(current_time - btn_pressed_time >= SWITCH_TIMEOUT)
        {
          // pobudka
          btn_pressed_time = current_time;

          for(int i=0; i<192; i++)
          {
            analogWrite(LED_PIN, i);
            delay(10);
          }
          for(int ip=0; ip<6; ip++)
          {
            for(int i=0; i<128; i++)
            {
              analogWrite(LED_PIN, i);
              delayMicroseconds(500);
            }
            for(int i=128; i>0; i--)
            {
              analogWrite(LED_PIN, i);
              delayMicroseconds(500);
            }
          }
          digitalWriteFast(LED_PIN,LOW);
          analogWrite(LED_PIN, 0);

          deviceIsLongsleep = false;

          // po dlugim snie moze przy checktimeout wpasc znow w deepsleep
          // dlatego gwizd_start_at = teraz
          gwizd_start_at = current_time; 

          detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
          uc_state = UC_WAKE_AND_CHECK;
        }
        //uc_state = UC_GO_SLEEP;// nowe - przemyslec!
      }

      // jesli przycisk wcisniety a urzadzenie pracuje normalnie:
      else if(btn_state == true && deviceIsLongsleep == false) // guzik + nadajnik ON
      {
        if(current_time - btn_pressed_time >= SWITCH_TIMEOUT)
        {
          // spij
          deviceIsLongsleep = true;
          delegate_to_longsleep = true;    

          for(int i=192; i>0; i--)
          {
            analogWrite(LED_PIN, i);
            delay(10);
          }
          digitalWriteFast(LED_PIN,LOW);
          analogWrite(LED_PIN, 0);

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
  nadajnik.manageTimeout();
}