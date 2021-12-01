#include "configuration.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <time.h>
#include "LowPower.h"
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>



byte const addressList[][5] = {"Odb0","Odb1","Odb2","Odb3","Odb4","Odb5","Odb6","Odb7"};  // dostepne adresy odbiornikow zgodnie ze zworkami 1-3

class Nadajnik{

    private:
        // PRESSURE ZMIENNE
        float bme_raw;                // dane raw z BME280
        float bme_tbl[BME_AVG_COUNT+1]; // tablica z probkami cisnienia 
        float bme_avg = 0;            // srednie cisnienie -> bme_avg / BME_AVG_COUNT
        int   bme_avg_i = 0;          // licznik AVG
        bool  bmeTableNeedsInitialization = true;     // info o pierwszym wypelnianiu tabeli AVG
        bool  gwizd_on    = false;    // info o aktywnym gwizdku
        bool  no_gwizd    = false;    // przestawia transmisje na 'nieaktywna' = 2 w kolejnej petli
        int   rf_repeat   = 0;        // ilosc powtorzen transmisji do odbiornika
        int   rf_off_repeat = 0;      // ilosc powtorzen wysylki 0 (off) do odbiornika
        time_t gwizd_start_at, giwzd_timeout;        // timeout gwizd

        int pickedAddress = 0;              // wybor adresu z tablicy powyzej
        bool  whistle_connected = false;

        period_t sleeptime = SLEEP_120MS;
        time_t current_time;
        time_t btn_current, btn_pressed_time, btn_timeout, last_rst_click;
        time_t start_click_addr;
        bool btn_state = LOW;
        bool btn_last_state = LOW;
        int btn_rst_counter = 0;

        bool deviceIsLongsleep = false;
        bool delegate_to_longsleep = false;

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

    public:

        // wiadomo..
        void init();

        /*****************************************************
         * Obsluga przerwania przycisku
         * ***************************************************/
        void ButtonPressed();

        /*****************************************************
         * Przerwanie dla przycisku
         * ***************************************************/
        void ISR_INT0_vect();

        /*****************************************************
         * Uruchamia niezbedne peryferia po spaniu
         * ***************************************************/
        void wakeUp();

        /*****************************************************
         * Wylacza wszystko do spania
         * ***************************************************/
        void prepareToSleep();

        /*****************************************************
         * Odczyt z BME
         * ***************************************************/
        void pressureRead();

        /*********************************************************************
         * FUNKCJA ROZBIEGOWA TABLICY Z PROBKAMI CISNIENIA
         * URUCHAMIANA RAZ PO DLUZSZEJ (RF_OFF_TIME) NIEAKTYWNOSCI NADAJNIKA
         * *******************************************************************/
        void pressureInitialize();

        /*********************************************************************
         * FUNKCJA CZYSZCZACA TABLICE Z PROBKAMI CISNIENIA
         * USTAWIA bmeTableNeedsInitialization NA TRUE!
         * *******************************************************************/
        void clearPressureAvg();

        /*********************************************************************
         * ODBIOR DANYCH Z NADAJNIKA I USREDNIENIE WYNIKU DO DALSZYCH DZIALAN
         * URUCHAMIANE TYLKO KIEDY NADAJNIK ODBIERA BME
         * *******************************************************************/
        void managePressure();

        /*********************************************************************
         * SPRAWDZA CZY NASTAPIL WZROST CISNIENIA W STOSUNKU DO SREDNIEJ AVG
         * CZULOSC BME_AVG_SENS
         * 0  - WYLACZ WYJSCIA
         * 1  - WLACZ WYJSCIA
         * 2  - BRAK TRANSMISJI [aby nie kolidowac z innym nadajnikiem]
         * *******************************************************************/
        void checkPressure();

        /*********************************************************************
         * Odlicza Czas do wylaczenia
         * *******************************************************************/
        void manageTimeout();

        /*********************************************************************
         * Transmisja do odbiornika przez NRF24
         * *******************************************************************/
        bool SendRFData();


        void setAddress(int address);
        int getAddress();

        void setBmeTableNeedsInitialization(bool val);
        void getBmeTableNeedsInitialization();

};