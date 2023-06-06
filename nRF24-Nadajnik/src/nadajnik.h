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
        float bmeRaw;                       // dane raw z BME280
        float bmeTable[BME_AVG_COUNT+1];    // tablica z probkami cisnienia
        bool  bmeTableNeedsInit = true;     // info o pierwszym wypelnianiu tabeli AVG
        float bmeAverage = 0;               // srednie cisnienie -> bme_avg / BME_AVG_COUNT
        int   bmeAverageIterator = 0;       // licznik AVG

        bool  sendSignal  = false;    // info o aktywnym gwizdku
        bool  pauseAfterGwizd    = false;    // jesli gwizdniecie wykryty - robi pauze na 1s
        int   repeatSending   = 0;    // ilosc powtorzen transmisji do odbiornika
        int   repeatSendingOffMsg = 0;      // ilosc powtorzen wysylki 0 (off) do odbiornika
        time_t gwizdStartTime;        // timeout gwizd

        int pickedAddress = 0;              // wybor adresu z tablicy powyzej
        bool  whistle_connected = false;

        period_t sleeptime = SLEEP_120MS;
        time_t currentTime;
        time_t btn_current, btn_pressed_time, btn_timeout, last_rst_click;
        time_t start_click_addr;
        bool buttonState = LOW;
        bool buttonLastState = LOW;
        int buttonClickCount = 0;

        bool isInLongsleep = false;         // device is in longsleep flag
        bool goToLongsleep = false;         // sets device to go to longsleep in nex loop

        enum EWhistleCommands{
            ELightsOn,
            ETimerStop,
            EDefaultState
        };

        enum EDevices{
            EWhistle,
            EController
        };

        struct WhistleData
        {
            int     device = EWhistle;
            int     command = EDefaultState;
        };
        WhistleData whistleData;

        bool ackOK;

    public:

        enum uc_State {
            UC_GO_SLEEP = 0,
            UC_WAKE_AND_CHECK = 1,
            UC_BTN_CHECK  = 2,
        };
        uc_State uc_state;

        // wiadomo..
        void init();

        void reset();

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

        /*********************************************************************
         * Obsluga przycisku i jego akcji
         * *******************************************************************/
        void manageButton();


        void setAddress(int address);
        int getAddress();

        void setBmeTableNeedsInitialization(bool val);
        bool getBmeTableNeedsInitialization();

        // sets device to go to longsleep in nex loop
        void setGoToLongsleep(bool val);
        bool getGoToLongsleep();
        
        // device longsleep state:
        void setIsLongsleep(bool val);
        bool isLongsleep();

        void setSendSignal(bool val);
        bool getSendSignalState();

        void setRepeatSending(int iterations);
        int getRepeatSending();

        void setPauseAfterGwizd(bool val);
        bool getPauseAfterGwizd();
        
        float getBmeRawData();
        float getBmeAverage();

        time_t getCurrentTime();

        period_t getSleepTime();

        uc_State getState();


};