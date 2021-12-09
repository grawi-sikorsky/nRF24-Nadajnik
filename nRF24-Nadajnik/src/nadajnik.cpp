#include "nadajnik.h"
#include "configuration.h"
#include <digitalWriteFast.h>
#include "EEPROM.h"

// BME280 LIB
#define TINY_BME280_SPI
#include "TinyBME280.h"
tiny::BME280 bme;

extern RF24 radio; // CE, CSN

/*****************************************************
 * SW reset - dont reset peripherials
 * ***************************************************/
void(* resetFunc) (void) = 0; //declare reset function at address 0


void Nadajnik::init(){

    bme.beginSPI(10);

    radio.begin();
    radio.openWritingPipe(addressList[pickedAddress]);
    //radio.enableAckPayload();
    radio.setRetries(1,8); // delay, count
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setChannel(95);
    radio.stopListening();
    radio.powerDown();
    
    sleeptime = SLEEP_120MS;
    repeatSending = 0;

    pressureRead();      // odczyt z bme
    pressureInitialize();   // inicjalizacja tablicy AVG

    bme.setMode(00);
    bme.end();
}

void Nadajnik::ButtonPressed(){
  if(isInLongsleep == true)  // jesli urzadzenie jest wylaczone
  {
    btn_pressed_time = millis();
    uc_state = UC_BTN_CHECK;  // wlacz i przejdz do sprawdzenia stanu przycisku
  }
}

/*****************************************************
 * Uruchamia niezbedne peryferia po deepsleep
 * ***************************************************/
void Nadajnik::wakeUp()
{
  power_spi_enable();
  radio.powerUp();
  bme.begin();
  bme.setMode(11);
}

/*****************************************************
 * Wylacza wszystko do spania
 * ***************************************************/
void Nadajnik::prepareToSleep()
{
  ADCSRA &= ~(1 << 7);    // TURN OFF ADC CONVERTER
  //power_timer0_disable(); // TIMER 0 SLEEP WDT ...
  power_timer1_disable(); // Timer 1
  power_timer2_disable(); // Timer 2
  power_twi_disable();    // TWI (I2C)
  power_adc_disable();    // ADC converter

  #ifdef UNO
    power_usart0_enable();// Serial (USART) test
  #else
    power_usart0_disable();
  #endif

  radio.stopListening();
  radio.powerDown();    // delay(5);
  bme.setMode(00);
  bme.end();
  power_spi_disable();  // SPI
}

/*****************************************************
 * Odczyt z BME
 * ***************************************************/
void Nadajnik::pressureRead()
{
  if(bmeTableNeedsInit == true) delay(5);           // delay bo???? bo byc moze po deepsleep po resecie bme280 pierwsza wartosc odczytu moze byc nieprawdziwa

  bmeRaw = bme.readFixedPressure();         // Odczyt z czujnika bme
}

/*********************************************************************
 * FUNKCJA INICJALIZACYJNA TABLICY Z PROBKAMI CISNIENIA
 * URUCHAMIANA RAZ PO DLUZSZEJ (RF_OFF_TIME) NIEAKTYWNOSCI NADAJNIKA
 * *******************************************************************/
void Nadajnik::pressureInitialize()
{
  // START I USREDNIANIE DANYCH
  for(int i=0; i < BME_AVG_COUNT; i++)  // w rozbiegu usredniaj wraz z rosnacym licznikiem i.
  {
    bmeTable[i] = bmeRaw;               // przypisz dane z nadajnika x AVG COUNT
    bmeAverage += bmeTable[i];              // dodaj do sredniej wartosc z tablicy[i]
  }
  bmeAverage = bmeAverage / BME_AVG_COUNT;    // dzielimy przez ilosc zapisanych wartosci w tablicy
  bmeTableNeedsInit = false;
  bmeAverageIterator = 0;

  #ifdef DEBUGSERIAL
    Serial.println(bmeAverage);
  #endif
}

/*********************************************************************
 * FUNKCJA CZYSZCZACA TABLICE Z PROBKAMI CISNIENIA
 * USTAWIA bmeTableNeedsInit NA TRUE!
 * *******************************************************************/
void Nadajnik::clearPressureAvg()
{
  for (int i = 0; i < BME_AVG_COUNT; i++)
  {
    bmeTable[i] = 0;
  }
  bmeAverage = bmeAverageIterator = 0;
  bmeTableNeedsInit = true;
}

/*********************************************************************
 * ODBIOR DANYCH Z NADAJNIKA I USREDNIENIE WYNIKU DO DALSZYCH DZIALAN
 * URUCHAMIANE TYLKO KIEDY NADAJNIK ODBIERA BME
 * *******************************************************************/
void Nadajnik::managePressure()
{
  // Inicjalizacja TABLICY USREDNIONEGO CISNIENIA
  if(bmeTableNeedsInit == true)
  {
    clearPressureAvg();
    pressureInitialize();
  }

  // START USREDNIANIA DANYCH
  // dopoki bmeAverageIterator jest mniejsze od ilosci probek BME_AVG_COUNT
  // czy tu czasem nie jest o jedna wartosc mniej poprzez  < zamiast <= ?
  if(bmeAverageIterator < BME_AVG_COUNT)
  {
    if((bmeRaw < (bmeAverage + BME_AVG_DIFF)) && (bmeRaw > (bmeAverage - BME_AVG_DIFF)))  // jesli obecna probka miesci sie w widelkach +-[BME_AVG_DIFF] 
    {
      bmeTable[bmeAverageIterator] = bmeRaw;   // dodaj nowa wartosc do tabeli
      bmeAverageIterator++;                    // zwieksz licznik
    }
  }
  else
  {
    bmeAverageIterator = 0;
  }

  bmeAverage = 0;                          // zeruj srednia przed petla
  for(int i=0; i <= BME_AVG_COUNT; i++) // Usredniaj zgodnie z iloscia probek [BME_AVG_COUNT]
  {
    bmeAverage += bmeTable[i];              // dodaj do sredniej wartosc z tablicy[i]
  }
  bmeAverage = bmeAverage / BME_AVG_COUNT;    // dzielimy przez ilosc zapisanych wartosci w tablicy
}

/*********************************************************************
 * SPRAWDZA CZY NASTAPIL WZROST CISNIENIA W STOSUNKU DO SREDNIEJ AVG
 * CZULOSC BME_AVG_SENS
 * 0  - WYLACZ WYJSCIA
 * 1  - WLACZ WYJSCIA
 * 2  - BRAK TRANSMISJI [aby nie kolidowac z innym nadajnikiem]
 * *******************************************************************/
#ifdef GWIZD_2S
void Nadajnik::checkPressure()
{
  if(no_gwizd == true)                    //  jesli cisnienie wrocilo do normy-> najpierw kilka razy powtorz 0 a nastepnie 2 jako brak transmisji
  {
    // if(repeatSendingOffMsg < RF_OFF_REPEAT)     //  jesli 
    // {
    //   nrfdata.sendgwizd = 1;              // najpierw 0 jako informacja o wylaczeniu
    //   repeatSendingOffMsg++;
    // }
    // else                                  // jak juz wystarczajaco duzo 0 poleci - ustaw transmisje na nieaktywna
    // {
    //   nrfdata.sendgwizd = 2;
    //   no_gwizd = false;
    // }
  }
  if(bmeRaw > (bmeAverage + BME_AVG_SENS))             // JESLI NOWA PROBKA JEST WIEKSZA OD SREDNIEJ [AVG + AVG_DIFF]
  {
    #ifdef DEBUGSERIAL
      Serial.println("GWIZD ON");
    #endif

    sendSignal = true;                                  // ustaw gwizdek aktywny
    no_gwizd = false;
    nrfdata.sendgwizd = 1;                            // dane do wysylki
    gwizdStartTime = millis();                        // ustaw czas ostatniego gwizdniecia
  }
  else if((bmeRaw < (bmeAverage + BME_AVG_DIFF)) && (bmeRaw > (bmeAverage - BME_AVG_DIFF)) && sendSignal == true)   // JESLI CISNIENIE WRACA DO WIDELEK [AVG +- AVG_DIFF] a gwizdek jest aktywny
  {
    #ifdef DEBUGSERIAL
      Serial.println("Gwizd OFF");
    #endif
    //nrfdata.sendgwizd = 2;
    sendSignal = false;                                 // flaga gwizdka rowniez OFF
    no_gwizd = true;
    repeatSendingOffMsg = 0;
  }
}
#else
void check_pressure()
{
  if(no_gwizd == true)                    //  jesli cisnienie wrocilo do normy-> najpierw kilka razy powtorz 0 a nastepnie 2 jako brak transmisji
  {
    if(repeatSendingOffMsg < RF_OFF_REPEAT)     //  jesli 
    {
      nrfdata.sendgwizd = 0;              // najpierw 0 jako informacja o wylaczeniu
      repeatSendingOffMsg++;
    }
    else                                  // jak juz wystarczajaco duzo 0 poleci - ustaw transmisje na nieaktywna
    {
      nrfdata.sendgwizd = 2;
      no_gwizd = false;
    }
  }
  if(bmeRaw > (bmeAverage + BME_AVG_SENS))             // JESLI NOWA PROBKA JEST WIEKSZA OD SREDNIEJ [AVG + AVG_DIFF]
  {
    #ifdef DEBUGSERIAL
      Serial.println("GWIZD ON");
    #endif

    sendSignal = true;                                  // ustaw gwizdek aktywny
    no_gwizd = false;                                 // ustaw brak transmisji jako nieprawda
    nrfdata.sendgwizd = 1;                            // dane do wysylki
    gwizdStartTime = millis();                        // ustaw czas ostatniego gwizdniecia
  }
  else if((bmeRaw < (bmeAverage + BME_AVG_DIFF)) && (bmeRaw > (bmeAverage - BME_AVG_DIFF)) && sendSignal == true)   // JESLI CISNIENIE WRACA DO WIDELEK [AVG +- AVG_DIFF] a gwizdek jest aktywny
  {
    #ifdef DEBUGSERIAL
      Serial.println("Gwizd OFF");
    #endif
    nrfdata.sendgwizd = 0;
    sendSignal = false;                                 // flaga gwizdka rowniez OFF
    repeatSendingOffMsg = 0;                                // zeruj licznik powtorzen wysylki 0
    no_gwizd = true;
  }
}
#endif

/*********************************************************************
 * Odlicza Czas do wylaczenia
 * *******************************************************************/
void Nadajnik::manageTimeout()
{
  currentTime = millis();
  time_t inactiveTime = currentTime - gwizdStartTime;

  if(inactiveTime < TIMEOUT_1) // pierwszy prog
  {
    sleeptime = SLEEP_120MS;
  }
  else if(inactiveTime > TIMEOUT_1 && inactiveTime < TIMEOUT_2) // drugi prog
  {
    // zmniejsz probkowanie 2x/s
    sleeptime = SLEEP_250MS; // 1S
  }
  else if(inactiveTime > TIMEOUT_2 )
  { 
    goToLongsleep = true;
    isInLongsleep = true;
  }
}

/*********************************************************************
 * Transmisja do odbiornika przez NRF24
 * *******************************************************************/
bool Nadajnik::SendRFData()
{
  bool result;
  result = radio.write(&nrfdata, sizeof(nrfdata));   // PIERWSZA TRANSMISJA DO ODBIORNIKA!

  return result;
}

/*********************************************************************
 * Obsluga przycisku i jego akcji
 * *******************************************************************/
void Nadajnik::manageButton(){

      buttonLastState = buttonState;                // do rst
      buttonState = digitalReadFast(BUTTON_PIN);    // odczyt stanu guzika
      
      currentTime = millis();
      
      if(buttonState != buttonLastState) // jezeli stan przycisku sie zmienil
      {
        if(buttonState == HIGH)         // jezeli jest wysoki
        {
          buttonClickCount++;          // licznik klikniec ++
          last_rst_click = currentTime;  // zeruj timeout
          start_click_addr = currentTime;  // ustaw start klikniecia do zmiany adresu
        }
        if(currentTime - last_rst_click >= SW_RST_TIMEOUT)
        {
          buttonClickCount = 0;
          last_rst_click = currentTime;
        }
        if(buttonClickCount >= SW_RST_COUNT)
        {
          for(int i=0; i<12; i++)
          {
            digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
            delay(100);
          }
          resetFunc(); //call reset
        }
      }


      // CHECK IF BUTTON IS PRESSED FOR ADDRESS CHANGE (BETWEEN 850 & 2400 ms)
      if(currentTime - start_click_addr >= 850 && currentTime - start_click_addr <= 2400)
      {
        if(buttonState == LOW)
        {
          // funkcja parowania z odbiornikiem!
          if(pickedAddress < 7) { pickedAddress++; }
          else { pickedAddress = 0; }

          radio.openWritingPipe(addressList[ pickedAddress ]);
          EEPROM.update(EEPROM_ADDRESS_PLACE, pickedAddress);

          for (int i = 0; i < ((pickedAddress+1)*2); i++)
          {
            digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
            delay(200);
          }
          start_click_addr = currentTime = millis();
        }
      }

      // jesli przycisk nie jest wcisniety lub zostal zwolniony
      if(buttonState == LOW)
      {
        btn_pressed_time = currentTime;  // ustaw obecny czas
        start_click_addr = currentTime;  // zeruj start timer dla zmiany adresu
        // jesli przycisk zostanie nacisniety ostatnia wartość stad nie bedzie nadpisywana
      }

      // jesli sie obudzi po przerwaniu a przycisk juz nie jest wcisniety -> deepsleep
      if(buttonState == LOW && isInLongsleep == true)
      {
        isInLongsleep = true;
        goToLongsleep = true;
        uc_state = UC_GO_SLEEP;
      }

      // jesli przycisk wcisniety gdy urzadzenie bylo wylaczone:
      if(buttonState == HIGH && isInLongsleep == true) // jesli guzik + nadajnik off
      {
        if(currentTime - btn_pressed_time >= SWITCH_TIMEOUT)
        {
          // pobudka
          btn_pressed_time = currentTime;

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

          isInLongsleep = false;

          // po dlugim snie moze przy checktimeout wpasc znow w deepsleep
          // dlatego gwizdStartTime = teraz
          gwizdStartTime = currentTime; 

          detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
          uc_state = UC_WAKE_AND_CHECK;
        }
        //uc_state = UC_GO_SLEEP;// nowe - przemyslec!
      }

      // jesli przycisk wcisniety a urzadzenie pracuje normalnie:
      else if(buttonState == true && isInLongsleep == false) // guzik + nadajnik ON
      {
        if(currentTime - btn_pressed_time >= SWITCH_TIMEOUT)
        {
          // spij
          isInLongsleep = true;
          goToLongsleep = true;

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

      // jesli przycisk nie jest wcisniety gdy urzadzenie pracuje -> loop
      if(buttonState == LOW && goToLongsleep  == false)
      {
        btn_pressed_time = currentTime; // to wlasciwie mozna usunac na rzecz tego na gorze?
        // i od razu w krotka kime
        uc_state = UC_GO_SLEEP;
      }
}


int Nadajnik::getAddress(){
  return pickedAddress;
}

void Nadajnik::setAddress(int address){
  pickedAddress = address;
}

void Nadajnik::setBmeTableNeedsInitialization(bool val) { bmeTableNeedsInit = val; }
bool Nadajnik::getBmeTableNeedsInitialization() { return bmeTableNeedsInit; }

// sets device to go to longsleep in nex loop
void Nadajnik::setGoToLongsleep(bool val){ goToLongsleep = val; }
bool Nadajnik::getGoToLongsleep(){ return goToLongsleep; }

// device longsleep state:
void Nadajnik::setIsLongsleep(bool val){ isInLongsleep = val; }
bool Nadajnik::isLongsleep(){ return isInLongsleep; }

// 
void Nadajnik::setSendSignal(bool val){ sendSignal = val; }
bool Nadajnik::getSendSignalState(){ return sendSignal; }

void Nadajnik::setRepeatSending(int iterations) { repeatSending = iterations; }
int Nadajnik::getRepeatSending() { return repeatSending; }

float Nadajnik::getBmeRawData(){ return bmeRaw; }

float Nadajnik::getBmeAverage(){ return bmeAverage; }

time_t Nadajnik::getCurrentTime(){ return currentTime = millis(); }

period_t Nadajnik::getSleepTime(){ return sleeptime; }

Nadajnik::uc_State Nadajnik::getState(){ return uc_state; }

