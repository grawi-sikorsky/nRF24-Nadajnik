#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <time.h>
#include <digitalWriteFast.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include "LowPower.h"

// ************************
// NADAJNIK POMOCNICZY!
// ************************

// PINY
#define LED_PIN         5
#define LIGHTS_INT      2   // PD2 INT0
#define LIGHTS_INT2     3   // PD3 INT1

// USTAWIENIA ADRESU
#define ADDR1  5     // Zworka1 - ZW1
#define ADDR2  6     // Zworka2 - ZW2
#define ADDR3  7     // Zworka3 - ZW3
bool addr1_State, addr2_State, addr3_State,
    prev_addr1_State, prev_addr2_State, prev_addr3_State;


#define PICK_SIDE_INPUT 16  // PC2
#define LIGHTSUP_PERIOD 4           // ilosc powtorzen przez ktore wysylamy RF po wlaczeniu swiatel, pozniej moga byc wlaczone ale nie wysylamy danych

int INPUT1_RF_VAL = 11,             //  
    INPUT2_RF_VAL = 12,             //  
    BOTH_RF_VAL   = 13;             //  

bool input_1, input_2, input_1_prev, input_2_prev;    // info o wlaczonym wejsciu
bool lightsup;                        
int lightsup_iter;                    // ilosc powrotrzen sygnalu gdy wejscie jest wlaczone
bool RightSide;                       // zworka na pcb do wyboru strony boiska na ktorym pracuje ten nadajnik. TRUE/LOW = prawa strona, FALSE/HIGH = lewa strona.

byte address[][5] = {"Odb0","Odb1","Odb2","Odb3","Odb4","Odb5","Odb6","Odb7"};  // dostepne adresy odbiornikow zgodnie ze zworkami 1-3
int address_nr = 0; // wybor adresu z tablicy powyzej
RF24 radio(9, 10); // CE, CSN

period_t sleeptime = SLEEP_250MS;
time_t current_time;

bool delegate_to_longsleep = false;
bool whistle_connected = false;
bool ackOK;

struct outdata
{
  int     sendgwizd = 2;
  float   raw = 0;
  float   avg = 0;
};
outdata nrfdata;

enum uc_State {
  UC_GO_SLEEP = 0,
  UC_WAKE_AND_CHECK = 1,
};
uc_State uc_state;

void manage_zworki();

//#define DEBUGSERIAL
//#define DEBUG
//#define UNO

/*****************************************************
 * Obsluga przerwania przycisku
 * ***************************************************/
void LightsUp()
{
  #ifdef DEBUGSERIAL
    Serial.println("INTERRUPT");
  #endif
  uc_state = UC_WAKE_AND_CHECK; // wlacz i przejdz do sprawdzenia swiatel
}

/*****************************************************
 * Przerwanie dla przycisku
 * ***************************************************/
void ISR_INT0_vect()
{
  detachInterrupt(LIGHTS_INT);
  detachInterrupt(LIGHTS_INT2);
  LightsUp();
}
void ISR_INT1_vect()
{
  detachInterrupt(LIGHTS_INT);
  detachInterrupt(LIGHTS_INT2);
  LightsUp();
}

/*****************************************************
 * Uruchamia niezbedne peryferia po spaniu
 * Dodatkowo sprawdza adres na zworkach
 * ***************************************************/
void wakeUp()
{
  power_spi_enable(); // SPI
  radio.powerUp();
  manage_zworki();
}

/*****************************************************
 * Wylacza wszystko do spania
 * ***************************************************/
void prepareToSleep()
{
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

  radio.stopListening();
  radio.powerDown();
  power_spi_disable(); // SPI
}

/*****************************************************
 * SW reset - dont reset peripherials
 * ***************************************************/
void(* resetFunc) (void) = 0; //declare reset function at address 0


/*********************************************************************
 * Odlicza Czas do wylaczenia
 * *******************************************************************/
void manageTimeout()
{
  current_time = millis();
}

// PIERWSZA TRANSMISJA Z ODBIOREM ID OD ODBIORNIKA
// Przypisuje ID z odbiornika do nrfdata
// ustawia flage whistle_connected = true
bool SendRFData()
{
  bool result;
  result = radio.write(&nrfdata, sizeof(nrfdata));   // TRANSMISJA

  #ifdef ACK_ON
    if(result)
    {
      if ( radio.isAckPayloadAvailable() ) 
      {
        radio.read(&ackOK, sizeof(ackOK));
        whistle_connected = true;
        #ifdef DEBUGSERIAL
        Serial.print("ACK:"); Serial.println(ackOK);
        #endif
      }
      else
      {
        whistle_connected = false;
        #ifdef DEBUGSERIAL
        Serial.println("ACK OK, no data");
        #endif
      }
    }
    else
    {
      whistle_connected = false;
        #ifdef DEBUGSERIAL
        Serial.println("NO ACK");
        #endif
    }
    return whistle_connected;
  #endif

  return result;
}

// POBIERA ADRES ZE ZWOREK I USTAWIA GO DLA RFki
void setRFaddress()
{
  if(     addr1_State == false && addr2_State == false && addr3_State == false){address_nr = 0;}
  else if(addr1_State == false && addr2_State == false && addr3_State == true){address_nr = 1;}
  else if(addr1_State == false && addr2_State == true && addr3_State == false){address_nr = 2;}
  else if(addr1_State == false && addr2_State == true && addr3_State == true){address_nr = 3;}
  else if(addr1_State == true && addr2_State == false && addr3_State == false){address_nr = 4;}
  else if(addr1_State == true && addr2_State == false && addr3_State == true){address_nr = 5;}
  else if(addr1_State == true && addr2_State == true && addr3_State == false){address_nr = 6;}
  else if(addr1_State == true && addr2_State == true && addr3_State == true){address_nr = 7;}

  radio.openWritingPipe(address[address_nr]);
  radio.stopListening();
}

// SPRAWDZA CZY NASTAPILA ZMIANA W ZWORKACH
// JESLI TAK TO USTAWIA NOWY ADRES DLA ODBIORNIKA
void manage_zworki()
{
  // NAJPIERW USTAWIAMY PULLUPY NA PINACH BO PO DEEPSLEEP SA WYLACZONE I DALSZY ODCZYT JEST BLEDNY!
  pinModeFast(ADDR1, INPUT_PULLUP);
  pinModeFast(ADDR2, INPUT_PULLUP);
  pinModeFast(ADDR3, INPUT_PULLUP);

  // ODCZYTUJEMY WARTOSCI ZE ZWOREK (ZW1 - ZW3) W URZADZENIU
  addr1_State = !digitalReadFast(ADDR1);
  addr2_State = !digitalReadFast(ADDR2);
  addr3_State = !digitalReadFast(ADDR3);

  if( addr1_State != prev_addr1_State || addr2_State != prev_addr2_State || addr3_State != prev_addr3_State )
  {
    prev_addr1_State = addr1_State;
    prev_addr2_State = addr2_State;
    prev_addr3_State = addr3_State;
    setRFaddress();
  } 
}

void setup()
{
  uc_state = UC_GO_SLEEP;   // domyslny stan uC po uruchomieniu

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

  #ifndef UNO
    for (byte i = 0; i <= A5; i++)
    {
      pinModeFast(i, OUTPUT);    // changed as per below
      digitalWriteFast(i, LOW);  //     ditto
    }

    pinModeFast(LED_PIN,OUTPUT);

    pinModeFast(SS,OUTPUT);
    pinModeFast(MOSI,OUTPUT);
    pinModeFast(MISO,OUTPUT);
    pinModeFast(SCK,OUTPUT);
    digitalWriteFast(SS,HIGH);
    digitalWriteFast(MOSI,HIGH);
    digitalWriteFast(MISO,HIGH);
    digitalWriteFast(SCK,HIGH);
  #endif
  
  pinModeFast(ADDR1, INPUT_PULLUP);
  pinModeFast(ADDR2, INPUT_PULLUP);
  pinModeFast(ADDR3, INPUT_PULLUP);
  pinModeFast(LIGHTS_INT,INPUT_PULLUP);
  pinModeFast(LIGHTS_INT2,INPUT_PULLUP);
  pinModeFast(PICK_SIDE_INPUT, INPUT_PULLUP); // zwora do masy

  radio.begin();
  radio.openWritingPipe(address[address_nr]);
  //radio.enableAckPayload();
  radio.setRetries(1,8); // delay, count
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(95);
  radio.stopListening();

  manage_zworki();

  radio.powerDown();
  sleeptime = SLEEP_120MS;
  RightSide = !digitalReadFast(PICK_SIDE_INPUT);  // negacja bo do masy zwarte.

  if(RightSide == true)
  {
    INPUT1_RF_VAL = 11;
    INPUT2_RF_VAL = 12;
    BOTH_RF_VAL   = 13;
  }
  else  // left side
  {
    INPUT1_RF_VAL = 21;
    INPUT2_RF_VAL = 22;
    BOTH_RF_VAL   = 23;
  }
  
  for(int i=0; i<8; i++)
  {
    digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
    delay(50);
  }
}

void loop()
{
  switch(uc_state)
  {
    case UC_GO_SLEEP:
    {
      if(delegate_to_longsleep == true)  // dluga kima
      {
        #ifdef DEBUGSERIAL
          Serial.println("longsleep"); delay(5);
        #endif
        prepareToSleep(); // wylacza zbedne peryferia na czas snu
        attachInterrupt(digitalPinToInterrupt(LIGHTS_INT), ISR_INT0_vect, FALLING); // przerwanie 1
        attachInterrupt(digitalPinToInterrupt(LIGHTS_INT2), ISR_INT1_vect, FALLING); // przerwanie 2
        LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
        wakeUp();
        uc_state = UC_WAKE_AND_CHECK;
      }
      else    // krotka kima
      {
        prepareToSleep(); // wylacza zbedne peryferia na czas snu
        LowPower.powerDown(sleeptime,ADC_OFF,BOD_OFF);
        wakeUp();

        uc_state = UC_WAKE_AND_CHECK; // pokimal to sprawdzic co sie dzieje->
      }
      break;
    }

    case UC_WAKE_AND_CHECK:
    {
      //setRFaddress();
      input_1 = !digitalReadFast(LIGHTS_INT);   // przypisz negacje bo stan LOW to aktywne swiatlo
      input_2 = !digitalReadFast(LIGHTS_INT2);   // przypisz negacje bo stan LOW to aktywne swiatlo

      #ifdef DEBUGSERIAL
        Serial.print("INPUT1 state:"); Serial.println(input_1);
        Serial.print("INPUT2 state:"); Serial.println(input_2);
      #endif

      if(input_1 != input_1_prev)
      {
        if(input_1 == true)
        {
          #ifdef DEBUGSERIAL
            Serial.println("input1 true");
          #endif
          lightsup = true;                        // flaga lightsup
          delegate_to_longsleep = false;          // wylacz dlugi sen (od teraz odswieza sie co 500ms)
          lightsup_iter = 0;                      // przypisz czas wlaczenia
        }
        else
        {
          delegate_to_longsleep = true;
          lightsup = false;
        }
        input_1_prev = input_1;
      }

      if(input_2 != input_2_prev)
      {
        if(input_2 == true)
        {
          lightsup = true;                        // flaga lightsup
          delegate_to_longsleep = false;          // wylacz dlugi sen (od teraz odswieza sie co 500ms)
          lightsup_iter = 0;                      // przypisz czas wlaczenia
        }
        else
        {
          delegate_to_longsleep = true;
          lightsup = false;
        }
        input_2_prev = input_2;
      }

      if(lightsup == true)
      {
        if(lightsup_iter < LIGHTSUP_PERIOD) 
        {
          lightsup_iter++;
          #ifdef DEBUGSERIAL
            Serial.println("wysylamy..");
          #endif
          if(input_1 == true)
          {
            nrfdata.sendgwizd = INPUT1_RF_VAL;
          }
          if(input_2 == true)
          {
            nrfdata.sendgwizd = INPUT2_RF_VAL;
          }
          if(input_1 == true && input_2 == true)
          {
            nrfdata.sendgwizd = BOTH_RF_VAL;
          }
          SendRFData();                             // wyslij rf
          delay(2);                                 // delay zeby serial sie odswiezyl
        }
        else                                        // jesli input wciaz swieci a transmisja zostala juz nadana ->
        {
          sleeptime = SLEEP_1S;                     // zmniejsz odswiezanie w oczekiwaniu na wylaczenie input LEDow
        }
      }
      else
      {
        sleeptime = SLEEP_250MS;                    // domyslna wartosc -> uklad i tak idzie spac w deepsleep
        delegate_to_longsleep = true;
      }
      
      uc_state = UC_GO_SLEEP;
      break;
    }
  }
  manageTimeout();
}