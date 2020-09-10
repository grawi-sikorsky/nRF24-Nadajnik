#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// BME280 LIB
#define TINY_BME280_SPI
#include "TinyBME280.h"
tiny::BME280 bme1; //Uses I2C address 0x76 (jumper closed)
float bme_data;

RF24 radio(9, 10); // CE, CSN
const byte address[17] = "1100110011001100";
int data = 0;

void setup() {
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