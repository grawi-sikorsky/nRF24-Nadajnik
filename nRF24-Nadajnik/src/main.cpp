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
#define SW_RST_TIMEOUT  250          // czas w ktorym należy wykonac klikniecia dla RST
#define SW_RST_COUNT    5             // ilosc nacisniec do wykonania resetu
#define TIME_TO_WAIT_MS 5            // czas do nastepnego wyzwolenia????
#define TIMEOUT_1       10000// 72000       // pierwszy timeiut // realnie wychodzi jakies (1 800 000 ms = 30 min) / 25 = 72000
#define TIMEOUT_2       30000//144000       // drugi prog = 5 400 000 = 90 min // z uwagi na sleep-millis: 60 min

#define RF_SENDBACK     25          // ilosc transmisji nadawczych do odbioru powrotnej (musi byc identyczna z Odbiornikiem!)

#define RF_REPEAT       6           // ilosc powtorzen transmisji [w tym zawieraja sie tez ponizsze], [domyslne 0, dodatkowe 0, 2, 2]
#define RF_OFF_REPEAT   2           // ilosc powtorzen OFF  [jako dodatkowa poza domyslna jedna]

// BME280 LIB
#define TINY_BME280_SPI
#include "TinyBME280.h"
tiny::BME280 bme1; //Uses I2C address 0x76 (jumper closed)

// PRESSURE DEFINICJE I ZMIENNE
#define BME_AVG_COUNT 20      // wiecej -> dluzszy powrot avg do normy
#define BME_AVG_DIFF  800     // im mniej tym dluzej wylacza sie po dmuchaniu. Zbyt malo powoduje ze mimo wylaczenia sie gwizdka, wlacza sie ponownie gdy wartosci wracaja do normy i avg.
#define BME_AVG_SENS  200     // czulosc dmuchniecia
float bme_raw;                // dane raw z BME280
float bme_tbl[BME_AVG_COUNT+1]; // tablica z probkami cisnienia 
float bme_avg = 0;            // srednie cisnienie -> bme_avg / BME_AVG_COUNT
int   bme_avg_i = 0;          // licznik AVG
bool  bme_rozbieg = true;     // info o pierwszym wypelnianiu tabeli AVG
bool  gwizd_on    = false;    // info o aktywnym gwizdku
bool  no_gwizd    = false;    // przestawia transmisje na 'nieaktywna' = 2 w kolejnej petli
int   rf_repeat   = 0;        // ilosc powtorzen transmisji do odbiornika
int   rf_off_repeat = 0;      // ilosc powtorzen wysylki 0 (off) do odbiornika
time_t gwizd_start_at, giwzd_timeout;        // timeout gwizd


RF24 radio(8, 9); // CE, CSN
const byte address[5] = "Odb1";  // domyslny adres odbiornika
bool  whistle_connected = false;

period_t sleeptime = SLEEP_2S;
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


void prepareToSleep()
{
  //clock_prescale_set(clock_div_16);

  ADCSRA &= ~(1 << 7); // TURN OFF ADC CONVERTER

  power_adc_disable(); // ADC converter
  //power_timer0_disable();// TIMER 0 SLEEP WDT ...
  power_timer1_disable();// Timer 1
  power_timer2_disable();// Timer 2

  #ifdef UNO
    power_usart0_enable();// Serial (USART) test
  #else
    power_usart0_disable();
  #endif

  //power_twi_disable(); // TWI (I2C)
  
  //PORTD &= ~(1 << PD0);   // LOW pin0 CMT2110
  //radio.stopListening();
  radio.powerDown();// delay(5);
  bme1.setMode(00);
  bme1.end();
  power_spi_disable(); // SPI
}

void wakeUp()
{
  power_spi_enable(); // SPI
  radio.powerUp();
}

void setup() {
  //clock_prescale_set(clock_div_1);

  uc_state = UC_GO_SLEEP; // default uC state

  bme1.beginSPI(10);
  bme1.setMode(00);
  bme1.end();
  #ifdef DEBUGSERIAL
    Serial.begin(115200);
  #endif
  
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
    pinModeFast(USER_SWITCH,INPUT);
    digitalWriteFast(LED_PIN, LOW);  // LED OFF

    pinModeFast(SS,OUTPUT);
    pinModeFast(MOSI,OUTPUT);
    pinModeFast(MISO,OUTPUT);
    pinModeFast(SCK,OUTPUT);
    digitalWriteFast(SS,HIGH);
    digitalWriteFast(MOSI,HIGH);
    digitalWriteFast(MISO,HIGH);
    digitalWriteFast(SCK,HIGH);
  #endif

  radio.begin();
  radio.openWritingPipe(address);
  radio.enableAckPayload();
  radio.setRetries(1,8); // delay, count
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(95);
  radio.stopListening();

  sleeptime = SLEEP_2S;
  rf_repeat = 0;

  for(int i=0; i<8; i++)
  {
    digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
    delay(50);
  }
}

void loop() 
{

        #ifdef DEBUGSERIAL
          //Serial.println("shortsleep"); delayMicroseconds(850);
        #endif
        prepareToSleep(); // wylacza zbedne peryferia na czas snu
        LowPower.powerDown(sleeptime,ADC_OFF,BOD_OFF);
        //interrupts();
        power_spi_enable(); // SPI
        //radio.powerUp();
        digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
        delay(2000);
}