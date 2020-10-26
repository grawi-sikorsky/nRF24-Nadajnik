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
#define TIMEOUT_1       72000// 72000       // pierwszy timeiut // realnie wychodzi jakies (1 800 000 ms = 30 min) / 25 = 72000
#define TIMEOUT_2       144000//144000       // drugi prog = 5 400 000 = 90 min // z uwagi na sleep-millis: 60 min

#define RF_REPEAT       6           // ilosc powtorzen transmisji [w tym zawieraja sie tez ponizsze], [domyslne 0, dodatkowe 0, 2, 2]
#define RF_OFF_REPEAT   2           // ilosc powtorzen OFF  [jako dodatkowa poza domyslna jedna]

#define GWIZD_2S                    // jesli zdefiniowany to nadajnik wysyla tylko 1 i ew 2 do odbiornika, ledy wylaczaja sie w odbiorniku po uplynieciu 2s
                                    // jesli nie jest zdefiniowany, to nadajnik wysyla 1 gdy gwizd, nastepnie 0 aby wylaczyc ledy i 2 jako brak transmisji.

// BME280 LIB
#define TINY_BME280_SPI
#include "TinyBME280.h"
tiny::BME280 bme1; //Uses I2C address 0x76 (jumper closed)

// PRESSURE DEFINICJE I ZMIENNE
#define BME_AVG_COUNT 20      // wiecej -> dluzszy powrot avg do normy
#define BME_AVG_DIFF  800     // im mniej tym dluzej wylacza sie po dmuchaniu. Zbyt malo powoduje ze mimo wylaczenia sie gwizdka, wlacza sie ponownie gdy wartosci wracaja do normy i avg.
#define BME_AVG_SENS  50     // czulosc dmuchniecia
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

period_t sleeptime = SLEEP_120MS;
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

#define DEBUGSERIAL
//#define DEBUG
#define UNO


/*****************************************************
 * Obsluga przerwania przycisku
 * ***************************************************/
void ButtonPressed()
{
  if(device_in_longsleep == true)  // jesli urzadzenie jest wylaczone
  {
    btn_pressed_time = millis();

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
 * Uruchamia niezbedne peryferia po spaniu
 * ***************************************************/
void wakeUp()
{
  power_spi_enable(); // SPI
  radio.powerUp();
  bme1.begin();
  bme1.setMode(11);
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
  radio.powerDown();// delay(5);
  bme1.setMode(00);
  bme1.end();
  power_spi_disable(); // SPI
}

/*****************************************************
 * SW reset - dont reset peripherials
 * ***************************************************/
void softReset()
{
  //asm volatile ("  jmp 0");
  cli(); //irq's off
  wdt_enable(WDTO_60MS); //wd on,15ms
  while(1); //loop
}

void(* resetFunc) (void) = 0;//declare reset function at address 0


// ODCZYTUJE Z BME
void pressure_read()
{
  if(bme_rozbieg == true) delay(1);           // delay bo????

  bme_raw = bme1.readFixedPressure();         // Odczyt z czujnika bme
  #ifdef DEBUGSERIAL
    //Serial.println(bme_raw);
  #endif
}

/*********************************************************************
 * FUNKCJA ROZBIEGOWA TABLICY Z PROBKAMI CISNIENIA
 * URUCHAMIANA RAZ PO DLUZSZEJ (RF_OFF_TIME) NIEAKTYWNOSCI NADAJNIKA
 * *******************************************************************/
void pressure_prepare()
{
  // START ODBIORU I USREDNIANIE DANYCH
  for(int i=0; i < BME_AVG_COUNT; i++)  // w rozbiegu usredniaj wraz z rosnacym licznikiem i.
  {
    bme_tbl[i] = bme_raw;      // przypisz dane z nadajnika x AVG COUNT
    bme_avg += bme_tbl[i];              // dodaj do sredniej wartosc z tablicy[i]
  }
  bme_avg = bme_avg / BME_AVG_COUNT;    // dzielimy przez ilosc zapisanych wartosci w tablicy
  bme_rozbieg = false;
  bme_avg_i = 0;

  #ifdef DEBUGSERIAL
    Serial.println(bme_avg);
  #endif
}

/*********************************************************************
 * FUNKCJA CZYSZCZACA TABLICE Z PROBKAMI CISNIENIA
 * USTAWIA BME_ROZBIEG NA TRUE!
 * *******************************************************************/
void clear_pressure_avg()
{
  for (int i = 0; i < BME_AVG_COUNT; i++)
  {
    bme_tbl[i] = 0;
  }
  bme_avg = bme_avg_i = 0;
  bme_rozbieg = true;
}

/*********************************************************************
 * ODBIOR DANYCH Z NADAJNIKA I USREDNIENIE WYNIKU DO DALSZYCH DZIALAN
 * URUCHAMIANE TYLKO KIEDY NADAJNIK ODBIERA BME
 * *******************************************************************/
void manage_pressure()
{
  // ROZBIEG TABLICY SREDNIEGO CISNIENIA
  if(bme_rozbieg == true)
  {
    clear_pressure_avg();
    pressure_prepare();
    #ifdef DEBUGSERIAL
      Serial.println("pressure prepare");
    #endif
  }

  // START USREDNIANIA DANYCH
  if(bme_avg_i < BME_AVG_COUNT)
  {
    if((bme_raw < (bme_avg + BME_AVG_DIFF)) && (bme_raw > (bme_avg - BME_AVG_DIFF)))  // jesli obecne probka miesci sie w widelkach +-[BME_AVG_DIFF] 
    {
      bme_tbl[bme_avg_i] = bme_raw;  // dodaj nowa wartosc do tabeli
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
void check_pressure()
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
    nrfdata.sendgwizd = 2;
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
void manageTimeout()
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
    sleeptime = SLEEP_500MS; // 1S
  } 
  else if(giwzd_timeout > TIMEOUT_2 )
  { 
    delegate_to_longsleep = true;
    device_in_longsleep = true;
  }  
  #ifdef DEBUGSERIAL
    //Serial.print("gtimeout: "); Serial.println(giwzd_timeout);
    //Serial.print("gcur: "); Serial.println(current_time);
    //Serial.print("gat: "); Serial.println(gwizd_start_at);
  #endif
}

// PIERWSZA TRANSMISJA Z ODBIOREM ID OD ODBIORNIKA
// Przypisuje ID z odbiornika do nrfdata
// ustawia flage whistle_connected = true
bool SendRFData()
{
  bool result;
  result = radio.write(&nrfdata, sizeof(nrfdata));   // PIERWSZA TRANSMISJA DO ODBIORNIKA!
  /*
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
  */
}

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

  //#ifndef UNO
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
  //#endif

  bme1.beginSPI(10);

  radio.begin();
  radio.openWritingPipe(address);
  radio.enableAckPayload();
  radio.setRetries(1,8); // delay, count
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(95);
  radio.stopListening();

  sleeptime = SLEEP_120MS;
  rf_repeat = 0;

  pressure_read();      // odczyt z bme
  pressure_prepare();   // rozbieg tablicy avg

  radio.powerDown();
  bme1.setMode(00);
  bme1.end();

  for(int i=0; i<8; i++)
  {
    digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
    delay(50);
  }
}

void loop() {
  switch(uc_state)
  {
    case UC_GO_SLEEP:
    {
      if (delegate_to_longsleep == true)  // dluga kima
      {
        #ifdef DEBUGSERIAL
          Serial.println("longsleep"); delay(20);
        #endif
        prepareToSleep(); // wylacza zbedne peryferia na czas snu
        attachInterrupt(digitalPinToInterrupt(2), ISR_INT0_vect, RISING); // przerwanie sw
        LowPower.powerDown(SLEEP_FOREVER,ADC_OFF,BOD_OFF);
        wakeUp();
        bme_rozbieg = true;
      }
      else    // krotka kima
      {
        #ifdef DEBUGSERIAL
          //Serial.println("shortsleep"); delayMicroseconds(850);
        #endif
        prepareToSleep(); // wylacza zbedne peryferia na czas snu
        LowPower.powerDown(sleeptime,ADC_OFF,BOD_OFF);
        wakeUp();

        #ifndef UNO
          //digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
          //delay(5);
          //digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
          //delay(25);
        #endif

        uc_state = UC_WAKE_AND_CHECK; // pokimal to sprawdzic co sie dzieje->
      }
      break;
    }
    case UC_WAKE_AND_CHECK:
    {
        current_time = millis();
        pressure_read();
        manage_pressure();
        check_pressure();

        #ifdef DEBUGSERIAL
          //Serial.print("nrfgwi: "); Serial.println(nrfdata.sendgwizd);
          Serial.print("AVG: "); Serial.println(bme_avg);
        #endif
        nrfdata.raw = bme_raw;
        nrfdata.avg = bme_avg;

        if(gwizd_on)
        {
          SendRFData();
          rf_repeat = 0;
        }
        else
        {
          if ( rf_repeat < RF_REPEAT )
          {
            SendRFData();
            rf_repeat++;
          }
        }
        uc_state = UC_BTN_CHECK;
      break;
    }
    case UC_WAITING_FOR_SENDBACK:
    {
      //read_answer_from_receiver();
      break;
    }
    case UC_BTN_CHECK:
    {
      delegate_to_longsleep = false;  // inaczej pojdzie spac long
                                      // device_is_off pozostaje dla ustalenia czy 
                                      // nadajnik był wybudzony czy pracowal normalnie

      btn_last_state = btn_state;               // do rst
      btn_state = digitalReadFast(USER_SWITCH); // odczyt stanu guzika
      
      current_time = millis();

      if(btn_state != btn_last_state) // jezeli stan przycisku sie zmienil
      {
        if(btn_state == HIGH)         // jezeli jest wysoki
        {
          btn_rst_counter++;          // licznik klikniec ++
          last_rst_click = current_time;  // zeruj timeout
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
          //softReset();
          resetFunc(); //call reset
        }
      }

      // jesli przycisk nie jest wcisniety lub zostal zwolniony
      if(btn_state == LOW)
      {
        btn_pressed_time = current_time; // ustaw obecny czas
        // jesli przycisk zostanie nacisniety ostatnia wartość stad nie bedzie nadpisywana
      }

      // jesli przycisk nie jest wcisniety gdy urzadzenie pracuje -> loop
      if(btn_state == LOW && device_in_longsleep == false)
      {
        btn_pressed_time = current_time; // to wlasciwie mozna usunac na rzecz tego na gorze?
        // i od razu w krotka kime
        uc_state = UC_GO_SLEEP;
        break;
      }

      // jesli sie obudzi po przerwaniu a przycisk juz nie jest wcisniety -> deepsleep
      if(btn_state == LOW && device_in_longsleep == true)
      {
        device_in_longsleep = true;           // flaga off dla pewnosci
        delegate_to_longsleep = true;   // deleguj do glebokiego snu
        uc_state = UC_GO_SLEEP;
      }

      // jesli przycisk wcisniety gdy urzadzenie bylo wylaczone:
      if(btn_state == HIGH && device_in_longsleep == true) // jesli guzik + nadajnik off
      {
        if(current_time - btn_pressed_time >= SWITCH_TIMEOUT)
        {
          // pobudka
          btn_pressed_time = current_time;
          digitalWriteFast(LED_PIN,HIGH);
          delay(1000);
          digitalWriteFast(LED_PIN,LOW);

          device_in_longsleep = false;

          // po dlugim snie moze przy checktimeout wpasc znow w deepsleep
          // dlatego gwizd_start_at = teraz
          gwizd_start_at = current_time; 

          detachInterrupt(digitalPinToInterrupt(2));
          uc_state = UC_WAKE_AND_CHECK;
        }
        //uc_state = UC_GO_SLEEP;// nowe - przemyslec!
      }

      // jesli przycisk wcisniety a urzadzenie pracuje normalnie:
      else if(btn_state == true && device_in_longsleep == false) // guzik + nadajnik ON
      {
        if(current_time - btn_pressed_time >= SWITCH_TIMEOUT)
        {
          // spij
          device_in_longsleep = true;
          delegate_to_longsleep = true;          
          digitalWriteFast(LED_PIN,HIGH);
          delay(400);
          digitalWriteFast(LED_PIN,LOW);
          delay(400);
          digitalWriteFast(LED_PIN,HIGH);
          delay(400);
          digitalWriteFast(LED_PIN,LOW);
          uc_state = UC_GO_SLEEP;
        }

        // do parowania z odbiornikiem!
        // sprawdzamy czy po 1s od nacisniecia gwidka nie pojawilo sie dmuchniecie - jesli tak: parowanie.
        if(current_time - btn_pressed_time >= 1000)
        {
          if(gwizd_on == true)
          {
            // funkcja parowania z odbiornikiem!
          }
        }
      }
      else
      {
        // yyyyy...
      }

      break;
    }
  }
  manageTimeout();
}