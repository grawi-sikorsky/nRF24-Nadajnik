#include "nadajnik.h"
#include "configuration.h"

// BME280 LIB
#define TINY_BME280_SPI
#include "TinyBME280.h"
tiny::BME280 bme;

extern RF24 radio; // CE, CSN

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
    rf_repeat = 0;

    pressureRead();      // odczyt z bme
    pressureInitialize();   // inicjalizacja tablicy AVG

    bme.setMode(00);
    bme.end();
}

void Nadajnik::ButtonPressed(){
  if(deviceIsLongsleep == true)  // jesli urzadzenie jest wylaczone
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
  if(bmeTableNeedsInitialization == true) delay(5);           // delay bo???? bo byc moze po deepsleep po resecie bme280 pierwsza wartosc odczytu moze byc nieprawdziwa

  bme_raw = bme.readFixedPressure();         // Odczyt z czujnika bme
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
    bme_tbl[i] = bme_raw;               // przypisz dane z nadajnika x AVG COUNT
    bme_avg += bme_tbl[i];              // dodaj do sredniej wartosc z tablicy[i]
  }
  bme_avg = bme_avg / BME_AVG_COUNT;    // dzielimy przez ilosc zapisanych wartosci w tablicy
  bmeTableNeedsInitialization = false;
  bme_avg_i = 0;

  #ifdef DEBUGSERIAL
    Serial.println(bme_avg);
  #endif
}

/*********************************************************************
 * FUNKCJA CZYSZCZACA TABLICE Z PROBKAMI CISNIENIA
 * USTAWIA bmeTableNeedsInitialization NA TRUE!
 * *******************************************************************/
void Nadajnik::clearPressureAvg()
{
  for (int i = 0; i < BME_AVG_COUNT; i++)
  {
    bme_tbl[i] = 0;
  }
  bme_avg = bme_avg_i = 0;
  bmeTableNeedsInitialization = true;
}

/*********************************************************************
 * ODBIOR DANYCH Z NADAJNIKA I USREDNIENIE WYNIKU DO DALSZYCH DZIALAN
 * URUCHAMIANE TYLKO KIEDY NADAJNIK ODBIERA BME
 * *******************************************************************/
void Nadajnik::managePressure()
{
  // ROZBIEG TABLICY SREDNIEGO CISNIENIA
  if(bmeTableNeedsInitialization == true)
  {
    clearPressureAvg();
    pressureInitialize();
    #ifdef DEBUGSERIAL
      Serial.println("pressure prepare");
    #endif
  }

  // START USREDNIANIA DANYCH
  // dopoki bme_avg_i jest mniejsze od ilosci probek BME_AVG_COUNT
  // czy tu czasem nie jest o jedna wartosc mniej poprzez  < zamiast <= ?
  if(bme_avg_i < BME_AVG_COUNT)
  {
    if((bme_raw < (bme_avg + BME_AVG_DIFF)) && (bme_raw > (bme_avg - BME_AVG_DIFF)))  // jesli obecna probka miesci sie w widelkach +-[BME_AVG_DIFF] 
    {
      bme_tbl[bme_avg_i] = bme_raw;   // dodaj nowa wartosc do tabeli
      bme_avg_i++;                    // zwieksz licznik
    }
  }
  else
  {
    bme_avg_i = 0;
  }

  bme_avg = 0;                          // zeruj srednia przed petla
  for(int i=0; i <= BME_AVG_COUNT; i++) // Usredniaj zgodnie z iloscia probek [BME_AVG_COUNT]
  {
    bme_avg += bme_tbl[i];              // dodaj do sredniej wartosc z tablicy[i]
  }
  bme_avg = bme_avg / BME_AVG_COUNT;    // dzielimy przez ilosc zapisanych wartosci w tablicy
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
    if(rf_off_repeat < RF_OFF_REPEAT)     //  jesli 
    {
      nrfdata.sendgwizd = 1;              // najpierw 0 jako informacja o wylaczeniu
      rf_off_repeat++;
    }
    else                                  // jak juz wystarczajaco duzo 0 poleci - ustaw transmisje na nieaktywna
    {
      nrfdata.sendgwizd = 2;
      no_gwizd = false;
    }
  }
  if(bme_raw > (bme_avg + BME_AVG_SENS))             // JESLI NOWA PROBKA JEST WIEKSZA OD SREDNIEJ [AVG + AVG_DIFF]
  {
    #ifdef DEBUGSERIAL
      Serial.println("GWIZD ON");
    #endif

    gwizd_on = true;                                  // ustaw gwizdek aktywny
    no_gwizd = false;
    nrfdata.sendgwizd = 1;                            // dane do wysylki
    gwizd_start_at = millis();                        // ustaw czas ostatniego gwizdniecia
  }
  else if((bme_raw < (bme_avg + BME_AVG_DIFF)) && (bme_raw > (bme_avg - BME_AVG_DIFF)) && gwizd_on == true)   // JESLI CISNIENIE WRACA DO WIDELEK [AVG +- AVG_DIFF] a gwizdek jest aktywny
  {
    #ifdef DEBUGSERIAL
      Serial.println("Gwizd OFF");
    #endif
    //nrfdata.sendgwizd = 2;
    gwizd_on = false;                                 // flaga gwizdka rowniez OFF
    no_gwizd = true;
    rf_off_repeat = 0;
  }
}
#else
void check_pressure()
{
  if(no_gwizd == true)                    //  jesli cisnienie wrocilo do normy-> najpierw kilka razy powtorz 0 a nastepnie 2 jako brak transmisji
  {
    if(rf_off_repeat < RF_OFF_REPEAT)     //  jesli 
    {
      nrfdata.sendgwizd = 0;              // najpierw 0 jako informacja o wylaczeniu
      rf_off_repeat++;
    }
    else                                  // jak juz wystarczajaco duzo 0 poleci - ustaw transmisje na nieaktywna
    {
      nrfdata.sendgwizd = 2;
      no_gwizd = false;
    }
  }
  if(bme_raw > (bme_avg + BME_AVG_SENS))             // JESLI NOWA PROBKA JEST WIEKSZA OD SREDNIEJ [AVG + AVG_DIFF]
  {
    #ifdef DEBUGSERIAL
      Serial.println("GWIZD ON");
    #endif

    gwizd_on = true;                                  // ustaw gwizdek aktywny
    no_gwizd = false;                                 // ustaw brak transmisji jako nieprawda
    nrfdata.sendgwizd = 1;                            // dane do wysylki
    gwizd_start_at = millis();                        // ustaw czas ostatniego gwizdniecia
  }
  else if((bme_raw < (bme_avg + BME_AVG_DIFF)) && (bme_raw > (bme_avg - BME_AVG_DIFF)) && gwizd_on == true)   // JESLI CISNIENIE WRACA DO WIDELEK [AVG +- AVG_DIFF] a gwizdek jest aktywny
  {
    #ifdef DEBUGSERIAL
      Serial.println("Gwizd OFF");
    #endif
    nrfdata.sendgwizd = 0;
    gwizd_on = false;                                 // flaga gwizdka rowniez OFF
    rf_off_repeat = 0;                                // zeruj licznik powtorzen wysylki 0
    no_gwizd = true;
  }
}
#endif

/*********************************************************************
 * Odlicza Czas do wylaczenia
 * *******************************************************************/
void Nadajnik::manageTimeout()
{
  current_time = millis();
  giwzd_timeout = current_time - gwizd_start_at;

  if(giwzd_timeout < TIMEOUT_1) // pierwszy prog
  {
    sleeptime = SLEEP_120MS;
  }
  else if(giwzd_timeout > TIMEOUT_1 && giwzd_timeout < TIMEOUT_2) // drugi prog
  {
    // zmniejsz probkowanie 2x/s
    sleeptime = SLEEP_250MS; // 1S
  } 
  else if(giwzd_timeout > TIMEOUT_2 )
  { 
    delegate_to_longsleep = true;
    deviceIsLongsleep = true;
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

int Nadajnik::getAddress(){
  return pickedAddress;
}

void Nadajnik::setAddress(int address){
  pickedAddress = address;
}