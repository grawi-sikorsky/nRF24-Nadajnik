#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <time.h>
#include <digitalWriteFast.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>

// BME280 LIB
#define TINY_BME280_SPI
#include "TinyBME280.h"
tiny::BME280 bme1; //Uses I2C address 0x76 (jumper closed)
float bme_data;

RF24 radio(9, 10); // CE, CSN
const byte address[17] = "1100110011001100";
int data = 0;

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
  bme_data = bme1.readFixedPressure();  // Odczyt z czujnika bme
  Serial.print("BME nadano:");
  Serial.println(bme_data);
  //bme1.end();

  const char text[] = "0101010101";
  data++;
  //radio.begin();
  //radio.openWritingPipe(address);
  //radio.setPALevel(RF24_PA_LOW);
  //radio.stopListening();
  radio.write(&bme_data, sizeof(bme_data));
  delay(200);
}