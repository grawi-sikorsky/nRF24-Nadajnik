/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// BME280 LIB
//#define TINY_BME280_SPI
//#include "TinyBME280.h"
//tiny::BME280 bme1; //Uses I2C address 0x76 (jumper closed)
//float bme_data;

RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";
int data = 100;

void setup() {
  //bme1.beginSPI(8);
  //Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
}

void loop() {
  //bme_data = bme1.readFixedPressure();  // Odczyt z czujnika bme
  //Serial.println(bme_data);
  //bme1.end();

  const char text[] = "0101010101";
  data++;
  //radio.begin();
  //radio.openWritingPipe(address);
  //radio.setPALevel(RF24_PA_LOW);
  //radio.stopListening();
  radio.write(&data, sizeof(data));
  delay(1000);
}