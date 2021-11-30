

// PINY
#define LED_PIN         5
#define SPEAKER_PIN     7 //A2 // 7 minipro
#define TRANSMISION_PIN 0 // 4 w proto
#define BUTTON_PIN      2 // PD2 INT0 Button

#define EEPROM_ADDRESS_PLACE  128   //

// ustawienia
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

// Ustawienia czujnika BME
#define BME_AVG_COUNT 20      // wiecej -> dluzszy powrot avg do normy
#define BME_AVG_DIFF  800     // Im mniej tym dluzej wylacza sie po dmuchaniu. Zbyt malo powoduje ze mimo wylaczenia sie gwizdka, wlacza sie ponownie gdy wartosci wracaja do normy i avg.
#define BME_AVG_SENS  50      // Czulosc dmuchniecia