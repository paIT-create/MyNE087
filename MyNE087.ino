/*******************************************************************************************
 *  DWUKANAŁOWY TERMOMETR DS18B20 — WERSJA FSM + EEPROM MAPOWANIE (EDUCATIONAL)
 *
 *  Cel projektu:
 *  - 2 czujniki DS18B20 na jednej magistrali OneWire.
 *  - 2 kolory wyświetlania na 3-cyfrowym 7-seg (zielony / czerwony).
 *  - Naprzemienne pokazywanie temperatur A i B (z priorytetem na działający czujnik).
 *  - Wysoka odporność na hot-plug (odłączanie/podłączanie czujników w locie).
 *  - Trwałe mapowanie "który ROM jest A, który jest B" w zewnętrznej EEPROM I2C.
 *
 *  Kluczowe idee architektury:
 *  1) ISR (Timer2) robi tylko to, co musi być deterministyczne czasowo:
 *     - multiplex 3 cyfr,
 *     - wybór koloru,
 *     - proste animacje (blink / flow),
 *     - dithering jasności czerwonego (kompensacja różnicy jasności LED).
 *
 *     ISR nie:
 *     - czyta czujników,
 *     - nie używa I2C,
 *     - nie liczy średnich,
 *     - nie formatuje temperatur (poza DP).
 *
 *  2) Główna pętla loop() jest "schedulerem tickowym" bez delay():
 *     - co LDR_MS: aktualizacja jasności PWM,
 *     - co RESCAN_MS: rescan magistrali (hot-plug),
 *     - co FSM_TICK_MS: krok logiki (FSM).
 *
 *  3) Bufor wyświetlacza jest podwójny (double buffer):
 *     - loop()/FSM zapisuje do nieaktywnego bufora,
 *     - potem atomowo przełącza activeBuf,
 *     - ISR zawsze czyta stabilny bufor bez ryzyka "połowicznego" zapisu.
 *
 *  4) EEPROM trzyma mapę ROM->kanał:
 *     - magic/version/CRC, żeby wykryć śmieci/niezgodne dane,
 *     - zapis blokowy z podziałem na strony (page write), żeby nie zawijało adresów.
 *
 *  Tryb przypisywania (ASSIGN):
 *  - Start BEZ czujników => urządzenie wchodzi w procedurę mapowania:
 *    ASSIGN A (zielony flow) -> wykryj stabilnie 1 czujnik -> zapamiętaj jako A -> poproś o odłączenie
 *    ASSIGN B (czerwony flow) -> wykryj stabilnie 1 czujnik -> zapamiętaj jako B -> poproś o odłączenie
 *    SAVE -> zapis do EEPROM -> przejście do normalnej pracy
 *
 *  Uwaga o adresie EEPROM:
 *  - 24C01/24FC01: bazowo 0x50 + bity A2..A0
 *  - u Ciebie: A0=1, A1=1, A2=0 => 0x53
 *******************************************************************************************/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>

/* ===================== PINY / POŁĄCZENIA =====================

   Masz wyświetlacz 7-seg sterowany:
   - segmenty przez 74HC595 (shift register) + ULN2803,
   - anody (po 3 dla zielonego i 3 dla czerwonego) sterowane bezpośrednio z MCU -> 6x P-MOSFET AO3401A,
   - OE (Output Enable) 74HC595 na PWM (jasność) z LDR.

   To daje:
   - stabilny multiplex (ISR),
   - prostą regulację jasności (PWM na OE),
   - niezależne sterowanie kolorami przez anody.
*/

#define clockPin 13     // 74HC595 SHCP (Pin 11)
#define latchPin 12     // 74HC595 STCP (Pin 12)
#define dataPin 8       // 74HC595 DS   (Pin 14)
#define oePin 9         // 74HC595 OE   (Pin 13) + PWM brightness
#define ldrPin A0       // czujnik światła (LDR) do automatycznej jasności
#define ONE_WIRE_BUS 2  // magistrala OneWire do DS18B20

// Kolejność cyfr 0..2 powinna odpowiadać fizycznemu połączeniu anody.
const uint8_t anode_GREEN[3] = { 5, 7, 11 };
const uint8_t anode_RED[3] = { 6, 3, 10 };

/* ===================== EEPROM I2C =====================

   24C01 = 1 Kbit = 128 bajtów adresowania.

   Ważny detal praktyczny:
   Wiele małych EEPROM ma page-write np. 8 bajtów.
   Jeśli wyślesz więcej bajtów niż rozmiar strony, układ może "zawinąć" adres w obrębie strony.
   Dlatego zapis bloku dzielimy na bezpieczne kawałki, pilnując granic stron.
*/

#define EEPROM_I2C_ADDR 0x53
#define EEPROM_SIZE_BYTES 128
#define EEPROM_PAGE_SIZE 8

/* ===================== PARAMETRY LOGIKI / CZASU =====================

   dsResolution = 12:
   - DS18B20 w 12-bit robi konwersję maksymalnie ok. 750 ms.
   - Ponieważ ustawiamy setWaitForConversion(false), requestTemperatures() tylko startuje konwersję,
     a odczyt getTempC() bierze ostatnio ukończony wynik.

   FSM_TICK_MS = 7500:
   - to jest Twoje "tempo UX": co ile przełączasz pokazywaną temperaturę / robisz krok logiki.
   - 7.5 s jest spokojne, czytelne, bez migania nerwowego.

   RESCAN_MS = 3000:
   - co 3 s sprawdzamy magistralę (hot-plug),
   - ale nie robimy tego co pętlę, żeby nie obciążać OneWire i nie ryzykować glitchy.

   LDR_MS = 500:
   - jasność nie musi być super szybka,
   - pół sekundy daje płynne wrażenie i ogranicza szum od ADC.
*/

const uint8_t dsResolution = 12;

const unsigned long FSM_TICK_MS = 7500;
const unsigned long LDR_MS = 100;
const unsigned long RESCAN_MS = 3000;

// W trybie przypisywania skanujemy częściej, bo user robi czynności "tu i teraz".
const unsigned long ASSIGN_SCAN_MS = 350;

// Stabilność: wymagamy kilku kolejnych skanów z dokładnie 1 czujnikiem,
// aby odsiać krótkie "bouncowanie" kontaktów i przypadkowe stany przejściowe.
const uint8_t ASSIGN_STABLE_SCANS = 3;

const uint8_t AVG_SAMPLES = 6;     // małe uśrednianie = mniej "pływania" wyświetlacza
const uint8_t WARMUP_SAMPLES = 2;  // po błędzie odrzuć pierwsze próbki
const uint8_t FAILS_TO_ERROR = 3;  // ile błędów z rzędu uznać za stan błędu

// Kalibracja mapowania LDR->PWM (dopasuj do własnego zakresu LDR w obudowie)
long ldrInMin = 220;                      // MIN 220 [0]    przesuniecie granicy w celu wczesniejszego wlaczenia maksymalnej jasnosci
long ldrInMax = 990;                      // MAX 990 [1023] przesuniecie granicy w celu podniesienia minimalnej jasności w ciemnym pomieszczeniu
volatile uint8_t systemBrightness = 128;  // Aktualna jasność globalna (0-255)
// Kompensacja jasności czerwonego.
// Jeśli czerwony świeci mocniej niż zielony, można to wyrównać w ISR.
const uint8_t RED_LEVEL = 67;  // 0–255

/* ===================== SEGMENTY =====================
   Mapa segmentów dla cyfr 0..9 w standardowym układzie 7-seg.
   Zakładamy, że bit7 to DP (kropka dziesiętna).
*/

const byte segDigits[10] = {
  B00111111, B00000110, B01011011, B01001111, B01100110,
  B01101101, B01111101, B00000111, B01111111, B01101111
};

const byte segMinus = B01000000;
#define SEG_DP B10000000

// showDecimal jest ustawiane przez formatowanie temperatury (commitTemp),
// a czytane w ISR. Musi być volatile.
volatile bool showDecimal = false;

/* ===================== ONE WIRE / DS18B20 ===================== */

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/* ===================== ADRESY DS18B20 =====================

   addr[] to "aktywne" adresy przypisane do kanałów:
   - addr[0] = A (zielony, zewnętrzny)
   - addr[1] = B (czerwony, wewnętrzny)

   addrValid[] mówi, czy dany kanał ma dopasowany ROM, który faktycznie jest na magistrali.
   To ważne, bo hot-plug: zapisany ROM może chwilowo zniknąć.
*/

DeviceAddress addr[2];
bool addrValid[2] = { false, false };

// savedA/savedB to to, co wczytaliśmy z EEPROM.
// eepromMapValid mówi, czy mapa była poprawna (magic+version+CRC).
DeviceAddress savedA;
DeviceAddress savedB;
bool eepromMapValid = false;

/* ===================== ONE WIRE SCAN RESULT =====================

   scanBusAll() zwraca maksymalnie 4 adresy — to jest "bezpiecznik":
   - w praktyce masz max 2 czujniki,
   - ale jeśli ktoś podłączy więcej, nie chcesz rozwalić RAM ani logiki.
*/

struct ScanResult {
  uint8_t count = 0;
  DeviceAddress found[4];
};

/* ===================== DISPLAY BUFFER (DOUBLE BUFFER) =====================

   displayBuf[2][3]:
   - 2 bufory,
   - każdy to 3 znaki (po jednym na cyfrę).
   activeBuf mówi, który bufor aktualnie czyta ISR.

   Najważniejsze: przełączanie activeBuf robimy w sekcji krytycznej (noInterrupts),
   żeby ISR nie trafił w moment przełączania.
*/

volatile char displayBuf[2][3];
volatile uint8_t activeBuf = 0;

/* ===================== ANIMACJE W ISR =====================

   blinkCount / blinkMode / blinkColor:
   - proste mignięcia potwierdzające (ASSIGN).
   - to jest "UI feedback" niezależny od FSM ticka.

   flowPos:
   - pozycja "płynącego znacznika" podczas ASSIGN.
*/

volatile uint8_t flowPos = 0;  // 0..2
volatile int8_t flowDir = 1;   // +1 / -1
volatile uint16_t flowTick = 0;

volatile uint8_t blinkCount = 0;
volatile uint8_t blinkColor = 0;  // 0=green, 1=red, 2=both (w tej wersji nie używane wprost)
volatile uint8_t blinkPhase = 0;

enum BlinkMode : uint8_t {
  BLINK_SOLID,    // jeden kolor
  BLINK_SEQUENCE  // sekwencja G-R-G... (bazuje na blinkPhase)
};
volatile BlinkMode blinkMode = BLINK_SOLID;

/* ===================== FSM STATES =====================

   FSM odpowiada za:
   - wybór "co teraz pokazujemy",
   - przejścia w tryby błędów,
   - procedurę przypisywania A/B.

   ISR nie zna "biznesowej logiki" (poza tym, że wie czy jest STARTUP / ASSIGN,
   bo od tego zależy kolor/animacja).
*/

enum FSMState : uint8_t {
  FSM_STARTUP,
  FSM_SHOW_A,
  FSM_SHOW_B,
  FSM_ERR_A,
  FSM_ERR_B,

  FSM_ASSIGN_A,
  FSM_ASSIGN_A_REMOVE,
  FSM_ASSIGN_B,
  FSM_ASSIGN_B_REMOVE,
  FSM_ASSIGN_SAVE
};

volatile FSMState fsmState = FSM_STARTUP;

// nextIsA realizuje proste przełączanie naprzemienne A/B,
// ale zawsze z priorytetem na czujnik, który działa.
bool nextIsA = true;

/* ===================== SENSOR CONTEXT =====================

   Każdy kanał ma:
   - bufor próbek do średniej kroczącej,
   - licznik fails (ile kolejnych odczytów było błędnych),
   - warmup (ile kolejnych poprawnych próbek ignorować po błędzie),
   - everGood (czy kiedykolwiek widzieliśmy poprawny odczyt — debug/diagnostyka).
*/

struct Sensor {
  float buf[AVG_SAMPLES];
  uint8_t pos = 0;
  bool ready = false;  // czy bufor zapełniony i średnia ma stałą "masę"
  float avg = NAN;

  uint8_t fails = 0;
  uint8_t warmup = 0;
  bool everGood = false;
};

Sensor s[2];

/* ===================== TIMING (SCHEDULER) ===================== */

unsigned long lastFSMTick = 0;
unsigned long lastLdrTick = 0;
unsigned long lastScanTick = 0;
unsigned long lastAssignScanTick = 0;

/* ===================== ASSIGN CONTEXT =====================

   assignA/assignB to adresy zebrane podczas procedury przypisywania.
   assignStable liczy kolejne skany spełniające warunek "dokładnie 1 czujnik".
*/

DeviceAddress assignA;
DeviceAddress assignB;
uint8_t assignStable = 0;

/* =========================================================================================
 *  LOW LEVEL: 74HC595 + ANODY
 * ========================================================================================= */

// void write595(uint8_t v) {
//   // Shift register aktualizujemy w 3 krokach:
//   // - latch LOW: odłącz wyjścia od rejestru przesuwnego,
//   // - shiftOut: wprowadź bity,
//   // - latch HIGH: "zatrzaśnij" nowy stan na wyjściach.
//   digitalWrite(latchPin, LOW);
//   shiftOut(dataPin, clockPin, MSBFIRST, v);
//   digitalWrite(latchPin, HIGH);
// }
void write595(uint8_t v) {
  // Pętla wysyłająca 8 bitów zoptymalizowana bezpośrednio pod Port B
  for (uint8_t i = 0; i < 8; i++) {
    if (v & 0x80) {
      PORTB |= (1 << PB0);  // dataPin (8) -> HIGH
    } else {
      PORTB &= ~(1 << PB0);  // dataPin (8) -> LOW
    }

    PORTB |= (1 << PB5);   // clockPin (13) -> HIGH (zbocze narastające)
    v <<= 1;               // Przesunięcie do kolejnego bitu
    PORTB &= ~(1 << PB5);  // clockPin (13) -> LOW
  }

  PORTB |= (1 << PB4);   // latchPin (12) -> HIGH (zatrzaśnięcie danych)
  PORTB &= ~(1 << PB4);  // latchPin (12) -> LOW
}

void setSegments(char c) {
  // Minimalistyczny "font":
  // - cyfry 0..9
  // - minus
  // - spacja / inne => wygaszenie
  if (c >= '0' && c <= '9') write595(segDigits[c - '0']);
  else if (c == '-') write595(segMinus);
  else write595(0);
}

void disableAnodes() {
  // W multiplexie zawsze:
  // 1) gasimy wszystkie anody,
  // 2) ustawiamy segmenty,
  // 3) zapalamy jedną anodę wybranej cyfry.
  //
  // To minimalizuje ghosting (przeswity w trakcie przełączania).
  for (int i = 0; i < 3; i++) {
    digitalWrite(anode_GREEN[i], HIGH);
    digitalWrite(anode_RED[i], HIGH);
  }
}

/* =========================================================================================
 *  DISPLAY BUFFER HELPERS
 * ========================================================================================= */

void fillG(volatile char o[3]) {
  // W Twoim UX "brak danych / błąd" jest pokazywany jako ---.
  // (To jest łatwe do zauważenia i uniwersalne.)
  o[0] = o[1] = o[2] = '-';
}

void commit(void (*fn)(volatile char[3])) {
  // Double buffer:
  // - zapisuj do nieaktywnego bufora,
  // - potem atomowo przełącz.
  //
  // Dlaczego to jest ważne?
  // ISR czyta displayBuf[activeBuf] w środku multiplexu.
  // Jeśli loop() zmieniłby znaki "w locie", zobaczyłbyś losowe artefakty na displayu.
  uint8_t b = 1 - activeBuf;
  fn(displayBuf[b]);
  noInterrupts();
  activeBuf = b;
  interrupts();
}

void commitTemp(float t) {
  // Formatowanie temperatury do 3 znaków.
  //
  // Zasady:
  // - jeśli |t| >= 100 -> nie pokażemy, bo brak miejsca (traktuj jako błąd -> ---)
  // - jeśli NaN -> ---
  // - jeśli |t| < 10 -> format " sX.Y " (s = spacja lub '-'), DP na środkowej cyfrze
  // - jeśli |t| >= 10 -> format " sXY " bez DP, zaokrąglony do int
  //
  // Uwaga: showDecimal musi być spójne z zawartością bufora,
  // bo ISR będzie zapalał DP tylko na środku.
  uint8_t b = 1 - activeBuf;

  if (fabs(t) >= 100.0) {
    showDecimal = false;
    fillG(displayBuf[b]);
    goto commit_switch;
  }

  if (isnan(t)) {
    showDecimal = false;
    fillG(displayBuf[b]);
  } else {
    bool neg = (t < 0);
    float at = fabs(t);

    if (at < 10.0) {
      int whole = (int)at;
      int frac = (int)round((at - whole) * 10);

      // Korekta błędu float: 9.96 może dać frac=10.
      if (frac == 10) {
        frac = 0;
        whole++;
      }

      if (whole >= 10) {
        // Przejście 9.9 → 10.0: nie zmieści się z DP w 3 cyfrach.
        showDecimal = false;
        displayBuf[b][0] = neg ? '-' : ' ';
        displayBuf[b][1] = '1';
        displayBuf[b][2] = '0';
      } else {
        showDecimal = true;
        displayBuf[b][0] = neg ? '-' : ' ';
        displayBuf[b][1] = '0' + whole;
        displayBuf[b][2] = '0' + frac;
      }
    } else {
      int ti = (int)round(t);
      int absTi = abs(ti);

      showDecimal = false;
      displayBuf[b][0] = (ti < 0) ? '-' : ' ';
      displayBuf[b][1] = '0' + ((absTi / 10) % 10);
      displayBuf[b][2] = '0' + (absTi % 10);
    }
  }

commit_switch:
  noInterrupts();
  activeBuf = b;
  interrupts();
}

/* =========================================================================================
 *  AVERAGING / FILTRACJA
 * ========================================================================================= */

void resetAvg(uint8_t i) {
  // Reset stanu uśredniania jest kluczowy po:
  // - wykryciu serii błędów (fails),
  // - hot-plug,
  // - zakończeniu ASSIGN (czyste wejście w normalną pracę).
  for (int k = 0; k < AVG_SAMPLES; k++) s[i].buf[k] = NAN;
  s[i].pos = 0;
  s[i].ready = false;
  s[i].avg = NAN;
}

void pushAvg(uint8_t i, float v) {
  // Prosta średnia krocząca po buforze cyklicznym.
  // Dodatkowo:
  // - liczymy tylko próbki nie-NaN,
  // - dopóki bufor nie zapełni się, średnia bazuje na dostępnych próbkach.
  s[i].buf[s[i].pos++] = v;
  if (s[i].pos >= AVG_SAMPLES) {
    s[i].pos = 0;
    s[i].ready = true;
  }

  uint8_t n = s[i].ready ? AVG_SAMPLES : s[i].pos;

  float sum = 0;
  uint8_t cnt = 0;
  for (int k = 0; k < n; k++) {
    if (!isnan(s[i].buf[k])) {
      sum += s[i].buf[k];
      cnt++;
    }
  }
  s[i].avg = cnt ? sum / cnt : NAN;
}

/* =========================================================================================
 *  EEPROM LOW LEVEL (I2C)
 * ========================================================================================= */

static bool eepromWaitReady(uint16_t timeoutMs = 50) {
  // EEPROM po zapisie wewnętrznie programuje komórki.
  // W tym czasie nie odpowiada ACK na adres.
  // Typowy wzorzec: polling aż do ACK.
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    Wire.beginTransmission(EEPROM_I2C_ADDR);
    uint8_t rc = Wire.endTransmission();
    if (rc == 0) return true;
    delay(1);
  }
  return false;
}

static bool eepromWriteByte(uint8_t memAddr, uint8_t val) {
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write(memAddr);
  Wire.write(val);
  if (Wire.endTransmission() != 0) return false;
  return eepromWaitReady();
}

static bool eepromReadByte(uint8_t memAddr, uint8_t& val) {
  // Wzorzec losowego odczytu:
  // 1) write address (bez stop -> repeated start),
  // 2) requestFrom.
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write(memAddr);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t n = Wire.requestFrom((uint8_t)EEPROM_I2C_ADDR, (uint8_t)1);
  if (n != 1) return false;
  val = Wire.read();
  return true;
}

static bool eepromWriteBlock(uint8_t memAddr, const uint8_t* data, uint8_t len) {
  // Zapis blokowy z podziałem na strony.
  // Dzięki temu unikamy zawinięcia wewnątrz strony (page-write wrap).
  while (len) {
    uint8_t pageOff = memAddr % EEPROM_PAGE_SIZE;
    uint8_t chunk = min((uint8_t)(EEPROM_PAGE_SIZE - pageOff), len);

    Wire.beginTransmission(EEPROM_I2C_ADDR);
    Wire.write(memAddr);
    for (uint8_t i = 0; i < chunk; i++) Wire.write(data[i]);
    if (Wire.endTransmission() != 0) return false;
    if (!eepromWaitReady()) return false;

    memAddr += chunk;
    data += chunk;
    len -= chunk;
  }
  return true;
}

static bool eepromReadBlock(uint8_t memAddr, uint8_t* data, uint8_t len) {
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write(memAddr);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t got = Wire.requestFrom((uint8_t)EEPROM_I2C_ADDR, (uint8_t)len);
  if (got != len) return false;

  for (uint8_t i = 0; i < len; i++) data[i] = Wire.read();
  return true;
}

/* =========================================================================================
 *  EEPROM MAP STRUCT (MAGIC / VERSION / CRC)
 * ========================================================================================= */

struct EepromMap {
  uint16_t magic;    // stała rozpoznawcza: "to są nasze dane"
  uint8_t version;   // wersjonowanie formatu (na przyszłość)
  uint8_t addrA[8];  // ROM 64-bit DS18B20
  uint8_t addrB[8];
  uint8_t crc;  // prosta kontrola: XOR wszystkich poprzednich bajtów
};

static uint8_t crcXor(const uint8_t* p, uint8_t n) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < n; i++) c ^= p[i];
  return c;
}

bool loadMapFromEEPROM() {
  EepromMap m;
  if (sizeof(m) > EEPROM_SIZE_BYTES) return false;

  if (!eepromReadBlock(0, (uint8_t*)&m, sizeof(m))) return false;

  // Magic + version to szybkie sito na "śmieci" (np. świeża EEPROM = 0xFF)
  if (m.magic != 0xA55A) return false;
  if (m.version != 1) return false;

  // CRC kończy temat: jeśli nie pasuje -> nie ufamy mapie.
  uint8_t calc = crcXor((uint8_t*)&m, sizeof(m) - 1);
  if (calc != m.crc) return false;

  memcpy(savedA, m.addrA, 8);
  memcpy(savedB, m.addrB, 8);
  return true;
}

bool saveMapToEEPROM(const DeviceAddress a, const DeviceAddress b) {
  EepromMap m;
  m.magic = 0xA55A;
  m.version = 1;
  memcpy(m.addrA, a, 8);
  memcpy(m.addrB, b, 8);
  m.crc = crcXor((uint8_t*)&m, sizeof(m) - 1);

  return eepromWriteBlock(0, (uint8_t*)&m, sizeof(m));
}

/* =========================================================================================
 *  HELPERS: PORÓWNANIA ADRESÓW
 * ========================================================================================= */

bool sameAddr(const DeviceAddress a, const DeviceAddress b) {
  for (uint8_t i = 0; i < 8; i++)
    if (a[i] != b[i]) return false;
  return true;
}

bool isAddrAllFF(const DeviceAddress a) {
  // czasem przy błędach/błędnym odczycie można trafić 0xFF...FF,
  // ale w tej wersji to helper (niekoniecznie używany w logice).
  for (uint8_t i = 0; i < 8; i++)
    if (a[i] != 0xFF) return false;
  return true;
}

/* =========================================================================================
 *  SCAN ONE-WIRE
 * ========================================================================================= */

ScanResult scanBusAll() {
  // Skanujemy magistralę i zbieramy ROMy.
  //
  // Uwaga praktyczna:
  // sensors.begin() zwykle wywołuje reset/odkrywanie urządzeń.
  // Tu robimy to w scan, bo scan jest naszym "punktem prawdy" przy hot-plug.
  ScanResult r;

  sensors.begin();
  int n = sensors.getDeviceCount();

  DeviceAddress tmp;
  for (int i = 0; i < n && r.count < 4; i++) {
    if (sensors.getAddress(tmp, i)) {
      memcpy(r.found[r.count], tmp, 8);
      r.count++;
    }
  }

  // Rozdzielczość i non-blocking conversion ustawiamy konsekwentnie.
  sensors.setResolution(dsResolution);
  sensors.setWaitForConversion(false);

  // Start konwersji "na zapas" — dzięki temu kolejne odczyty mogą już mieć świeże dane.
  sensors.requestTemperatures();

  return r;
}

void applySavedMapToBus() {
  // Dopasuj aktualną magistralę do zapisanej mapy.
  // Efekt:
  // - jeśli czujnik A jest podłączony -> addrValid[0]=true
  // - jeśli nie -> false
  // analogicznie dla B.
  addrValid[0] = false;
  addrValid[1] = false;

  ScanResult r = scanBusAll();

  for (uint8_t i = 0; i < r.count; i++) {
    if (!addrValid[0] && sameAddr(r.found[i], savedA)) {
      memcpy(addr[0], r.found[i], 8);
      addrValid[0] = true;
    }
    if (!addrValid[1] && sameAddr(r.found[i], savedB)) {
      memcpy(addr[1], r.found[i], 8);
      addrValid[1] = true;
    }
  }
}

void takeFirstTwoFromBus() {
  // Tryb awaryjny / fallback:
  // Jeśli nie masz mapy w EEPROM, bierz pierwsze dwa czujniki jakie znajdziesz.
  // To nie gwarantuje stałości kolorów po zamianie czujników, ale umożliwia pracę.
  addrValid[0] = addrValid[1] = false;

  ScanResult r = scanBusAll();
  if (r.count > 0) {
    memcpy(addr[0], r.found[0], 8);
    addrValid[0] = true;
  }
  if (r.count > 1) {
    memcpy(addr[1], r.found[1], 8);
    addrValid[1] = true;
  }
}

/* =========================================================================================
 *  SENSOR UPDATE PIPELINE
 * ========================================================================================= */

float readSensor(uint8_t i) {
  // Od strony architektury to jest "najniższa warstwa":
  // - jeśli nie mamy adresu -> NAN
  // - jeśli urządzenie nie odpowiada -> NAN
  // - inaczej -> getTempC()
  //
  // DallasTemperature zwraca:
  // - poprawną temperaturę,
  // - albo specjalne wartości dla błędów (często -127).
  if (!addrValid[i]) return NAN;
  if (!sensors.isConnected(addr[i])) return NAN;
  return sensors.getTempC(addr[i]);
}

bool validTemp(float t) {
  // DS18B20 sensowny zakres: -55..125, ale biblioteki często używają -127 jako błąd.
  // Ty przyjąłeś warunek: -126..125 jako "OK".
  // Dzięki temu -127 i NaN wylatują.
  return !(isnan(t) || t < -126 || t > 125);
}

void updateSensor(uint8_t i) {
  // Ta funkcja realizuje pełny pipeline:
  // - odczyt raw,
  // - walidacja,
  // - licznik fails i reakcja na "serię błędów",
  // - warmup,
  // - uśrednianie.
  float raw = readSensor(i);

  if (!validTemp(raw)) {
    if (s[i].fails < 255) s[i].fails++;
    return;
  }

  // Jeśli mieliśmy wcześniej serię błędów, resetujemy średnią i dajemy warmup,
  // żeby pierwsze próbki po "powrocie" nie zrobiły skoku.
  if (s[i].fails >= FAILS_TO_ERROR) {
    resetAvg(i);
    s[i].warmup = WARMUP_SAMPLES;
  }

  s[i].fails = 0;
  s[i].everGood = true;

  if (s[i].warmup) {
    s[i].warmup--;
    return;
  }

  pushAvg(i, raw);

  // TEST ONLY (zostawiam jako opcję do szybkich testów wyświetlania)
  // pushAvg(0, -9.84);
  // pushAvg(1,  8.84);
}

/* =========================================================================================
 *  ASSIGN DISPLAY: FLOW
 * ========================================================================================= */

// Uwaga: commitAssignFlow() w tej wersji nie jest używany przez ISR,
// bo ISR renderuje flow bezpośrednio (na podstawie flowPos i stanu ASSIGN).
// Zostaje jako przykład alternatywy: "przygotuj bufor i przełącz".

// volatile uint8_t flowDigit = 0;

// void commitAssignFlow() {
//   uint8_t b = 1 - activeBuf;
//   displayBuf[b][0] = ' ';
//   displayBuf[b][1] = ' ';
//   displayBuf[b][2] = ' ';
//   displayBuf[b][flowDigit] = '-';
//   noInterrupts();
//   activeBuf = b;
//   interrupts();
// }

/* =========================================================================================
 *  FSM: NORMAL MODE
 * ========================================================================================= */

// void fsmStepNormal() {
//   // Zaczynamy od rozpoczęcia konwersji temperatur — non-blocking.
//   sensors.requestTemperatures();

//   // Aktualizujemy oba kanały: każdy sam zarządza fails/warmup/avg.
//   updateSensor(0);
//   updateSensor(1);

//   bool aOK = (s[0].fails < FAILS_TO_ERROR) && addrValid[0];
//   bool bOK = (s[1].fails < FAILS_TO_ERROR) && addrValid[1];

//   // anyGood: czy w ogóle mamy jakiś sensowny wynik do pokazania.
//   bool anyGood = (!isnan(s[0].avg) && aOK) || (!isnan(s[1].avg) && bOK);

//   if (!anyGood) {
//     // Jeśli nic nie ma sensownego, próbujemy przynajmniej pokazać to, co działa.
//     // Logika "preferencyjna" chroni UX przed ciągłym --- gdy jeden czujnik żyje.
//     if (!aOK && bOK) {
//       fsmState = FSM_SHOW_B;
//       commitTemp(s[1].avg);
//       return;
//     }
//     if (!bOK && aOK) {
//       fsmState = FSM_SHOW_A;
//       commitTemp(s[0].avg);
//       return;
//     }

//     // Jeśli oba martwe -> STARTUP jako stan "nie wiem co jest na magistrali".
//     fsmState = FSM_STARTUP;
//     commit(fillG);
//     return;
//   }

//   // Wyjście ze STARTUP:
//   // STARTUP to stan "nie wyświetlam temperatur, bo jeszcze nie wiem co działa".
//   // Jeśli już wiem, wybieram pierwszy dostępny kanał.
//   if (fsmState == FSM_STARTUP) {
//     if (aOK) fsmState = FSM_SHOW_A;
//     else if (bOK) fsmState = FSM_SHOW_B;

//     // commit(fillG) to krótkie "przecięcie" między trybami.
//     // Nie jest konieczne funkcjonalnie, ale UX-owo daje czytelne przejście.
//     commit(fillG);
//     return;
//   }

//   // NORMALNA PRACA: naprzemienność z priorytetem na sprawny kanał.
//   if (nextIsA) {
//     if (aOK) fsmState = FSM_SHOW_A;
//     else if (bOK) fsmState = FSM_SHOW_B;
//   } else {
//     if (bOK) fsmState = FSM_SHOW_B;
//     else if (aOK) fsmState = FSM_SHOW_A;
//   }
//   nextIsA = !nextIsA;

//   // Finalnie: jeśli wybrany kanał nie ma sensownych danych -> pokaż błąd kanału,
//   // w przeciwnym razie sformatuj i wyświetl temperaturę.
//   if (fsmState == FSM_SHOW_A) {
//     if (!aOK || isnan(s[0].avg)) {
//       fsmState = FSM_ERR_A;
//       commit(fillG);
//     } else {
//       commitTemp(s[0].avg);
//     }
//   } else if (fsmState == FSM_SHOW_B) {
//     if (!bOK || isnan(s[1].avg)) {
//       fsmState = FSM_ERR_B;
//       commit(fillG);
//     } else {
//       commitTemp(s[1].avg);
//     }
//   } else {
//     commit(fillG);
//   }
// }
void fsmStepNormal() {
  // Sprawdzamy statusy czujników na bazie danych zebranych w loop()
  bool aOK = (s[0].fails < FAILS_TO_ERROR) && addrValid[0];
  bool bOK = (s[1].fails < FAILS_TO_ERROR) && addrValid[1];

  // Czy którykolwiek kanał ma poprawną, przefiltrowaną średnią?
  bool anyGood = (!isnan(s[0].avg) && aOK) || (!isnan(s[1].avg) && bOK);

  if (!anyGood) {
    // Płynna degradacja: jeśli jeden padł, wymuś pokazywanie drugiego spranego
    if (!aOK && bOK) {
      fsmState = FSM_SHOW_B;
      commitTemp(s[1].avg);
      return;
    }
    if (!bOK && aOK) {
      fsmState = FSM_SHOW_A;
      commitTemp(s[0].avg);
      return;
    }

    // Całkowita awaria szyny -> kreski "---"
    fsmState = FSM_STARTUP;
    commit(fillG);
    return;
  }

  // Wyjście ze stanu STARTUP po wykryciu sprawnych czujników
  if (fsmState == FSM_STARTUP) {
    if (aOK) fsmState = FSM_SHOW_A;
    else if (bOK) fsmState = FSM_SHOW_B;
    commit(fillG);
    return;
  }

  // NAPRZEMIENNOŚĆ KANAŁÓW (Wybór wyświetlanego stanu biznesowego)
  if (nextIsA) {
    fsmState = aOK ? FSM_SHOW_A : FSM_SHOW_B;
  } else {
    fsmState = bOK ? FSM_SHOW_B : FSM_SHOW_A;
  }
  nextIsA = !nextIsA;

  // Renderowanie danych do nieaktywnego bufora (Double Buffering)
  if (fsmState == FSM_SHOW_A) {
    if (!aOK || isnan(s[0].avg)) {
      fsmState = FSM_ERR_A;
      commit(fillG);
    } else {
      commitTemp(s[0].avg);
    }
  } else if (fsmState == FSM_SHOW_B) {
    if (!bOK || isnan(s[1].avg)) {
      fsmState = FSM_ERR_B;
      commit(fillG);
    } else {
      commitTemp(s[1].avg);
    }
  } else {
    commit(fillG);
  }
}

/* =========================================================================================
 *  FSM: ASSIGN MODE
 * ========================================================================================= */

void fsmStepAssign() {
  // W ASSIGN skanujemy częściej i reagujemy natychmiast na działania użytkownika.
  unsigned long now = millis();
  if (now - lastAssignScanTick < ASSIGN_SCAN_MS) return;
  lastAssignScanTick = now;

  ScanResult r = scanBusAll();

  // Warunek stabilności: dokładnie 1 czujnik przez kilka kolejnych skanów.
  // To jest odpowiedź na:
  // - drgania styków,
  // - chwilowe stany, gdy user wkłada/wyjmuje wtyk.
  auto stableOneSensor = [&]() -> bool {
    if (r.count == 1) {
      if (assignStable < 255) assignStable++;
    } else {
      assignStable = 0;
    }
    return (assignStable >= ASSIGN_STABLE_SCANS);
  };

  if (fsmState == FSM_ASSIGN_A) {
    // Czekamy na stabilne "1 czujnik".
    if (stableOneSensor()) {
      memcpy(assignA, r.found[0], 8);
      assignStable = 0;

      // Potwierdzenie: kilka mignięć zielonym.
      blinkCount = 4;  // 4 "off transitions" -> 2 pełne mignięcia (zależnie od implementacji)
      blinkMode = BLINK_SOLID;
      blinkColor = 0;

      fsmState = FSM_ASSIGN_A_REMOVE;
    }
    return;
  }

  if (fsmState == FSM_ASSIGN_A_REMOVE) {
    // Wymuszamy odłączenie: dopiero gdy count==0, przechodzimy dalej.
    if (r.count == 0) {
      assignStable = 0;
      fsmState = FSM_ASSIGN_B;
    }
    return;
  }

  if (fsmState == FSM_ASSIGN_B) {
    if (stableOneSensor()) {
      // Zabezpieczenie przed przypisaniem tego samego czujnika do A i B.
      if (sameAddr(r.found[0], assignA)) {
        assignStable = 0;
        return;
      }
      memcpy(assignB, r.found[0], 8);
      assignStable = 0;

      // Potwierdzenie: mignięcia czerwonym.
      blinkCount = 4;
      blinkMode = BLINK_SOLID;
      blinkColor = 1;

      fsmState = FSM_ASSIGN_B_REMOVE;
    }
    return;
  }

  if (fsmState == FSM_ASSIGN_B_REMOVE) {
    if (r.count == 0) {
      fsmState = FSM_ASSIGN_SAVE;
    }
    return;
  }

  if (fsmState == FSM_ASSIGN_SAVE) {
    // Zapis mapy do EEPROM.
    bool ok = saveMapToEEPROM(assignA, assignB);
    eepromMapValid = ok;

    if (ok) {
      memcpy(savedA, assignA, 8);
      memcpy(savedB, assignB, 8);
    }

    // Sekwencja potwierdzająca: G-R-G (blinkPhase rośnie, ISR wybiera kolor fazy).
    blinkCount = 6;  // 3 pełne mignięcia "on/off"
    blinkMode = BLINK_SEQUENCE;
    blinkPhase = 0;

    // Po ASSIGN czyścimy uśrednianie i błędy — startujemy "jak nowi".
    resetAvg(0);
    resetAvg(1);
    s[0].fails = s[1].fails = 0;
    s[0].warmup = s[1].warmup = 0;

    // Wracamy do STARTUP: normalna praca ruszy gdy pojawią się czujniki.
    fsmState = FSM_STARTUP;
    commit(fillG);
    return;
  }
}

void fsmStep() {
  // Router: wybierz właściwe zachowanie zależnie od stanu.
  switch (fsmState) {
    case FSM_ASSIGN_A:
    case FSM_ASSIGN_A_REMOVE:
    case FSM_ASSIGN_B:
    case FSM_ASSIGN_B_REMOVE:
    case FSM_ASSIGN_SAVE:
      fsmStepAssign();
      return;

    default:
      fsmStepNormal();
      return;
  }
}

/* =========================================================================================
 *  ISR: TIMER2 COMPA — MULTIPLEX + UI FEEDBACK
 * =========================================================================================
 *
 *  Ustawienia w setup():
 *  - Timer2 w trybie CTC, preskaler 64, OCR2A=249
 *  - Na ATmega328P daje to ~1 kHz przerwań:
 *      f = 16MHz / 64 / (249+1) = 1000 Hz
 *
 *  W ISR przełączamy cyfrę co 1 ms:
 *  - 3 cyfry => ~333 Hz odświeżania każdej cyfry (flicker-free).
 *
 *  Dithering czerwonego:
 *  - czerwony może mieć inną jasność niż zielony.
 *  - Robimy prosty "frame skipping": w niektórych ramkach czerwony nie świeci.
 */

ISR(TIMER2_COMPA_vect) {
  static uint16_t blinkTick = 0;
  static bool blinkOn = false;

  // --- Blink engine ---
  if (blinkCount) {
    // blinkTick liczy przerwania (tu ~1 kHz).
    // 250 => ~250 ms na przełączenie ON/OFF.
    if (++blinkTick >= 250) {
      blinkTick = 0;
      blinkOn = !blinkOn;

      // Zmniejszamy blinkCount tylko na zboczu OFF,
      // żeby licznik odpowiadał "połówkom" mignięć w przewidywalny sposób.
      if (!blinkOn) {
        blinkCount--;
        if (blinkMode == BLINK_SEQUENCE) blinkPhase++;

        // Po zakończeniu mignięć inicjujemy flow od środka (ładniejszy UX).
        if (blinkCount == 0) {
          flowPos = 1;
          flowDir = 1;
          flowTick = 0;
        }
      }
    }
  }

  // --- Czy jesteśmy w trybie ASSIGN? ---
  bool isAssign =
    (fsmState == FSM_ASSIGN_A) || (fsmState == FSM_ASSIGN_A_REMOVE) || (fsmState == FSM_ASSIGN_B) || (fsmState == FSM_ASSIGN_B_REMOVE) || (fsmState == FSM_ASSIGN_SAVE);

  // --- Flow animation timing (niezależne od FSM ticka) ---
  if (isAssign && !blinkCount) {
    // flowTick liczy ISR ticki; 125 => ~125 ms.
    if (++flowTick >= 125) {
      flowTick = 0;
      flowPos += flowDir;
      if (flowPos == 2 || flowPos == 0) flowDir = -flowDir;
    }
  }

  // Multiplex state:
  static uint8_t d = 0;   // która cyfra 0..2
  static uint16_t t = 0;  // licznik do STARTUP blink
  static bool g = true;   // STARTUP: przełączaj zielony/czerwony

  // STARTUP blink:
  // Gdy nie wiemy co wyświetlać, migamy kolorami, żeby było widać "żyje".
  if (fsmState == FSM_STARTUP) {
    if (++t >= 500) {  // ~500 ms
      t = 0;
      g = !g;
    }
  } else {
    t = 0;
    g = true;
  }

  // 1) Zgaś wszystko
  disableAnodes();

  // 2) Ustaw segmenty zależnie od priorytetu:
  //    blink > assign flow > normalny bufor
  if (blinkCount) {
    setSegments('-');  // podczas potwierdzeń pokazujemy --- (czytelny sygnał "OK")
  } else if (isAssign) {
    // W ASSIGN pokazujemy przesuwający się '-' po jednej cyfrze.
    if (d == flowPos) setSegments('-');
    else setSegments(' ');
  } else {
    // Normalny rendering z bufora:
    byte seg = 0;
    char c = displayBuf[activeBuf][d];

    if (c >= '0' && c <= '9') seg = segDigits[c - '0'];
    else if (c == '-') seg = segMinus;
    else seg = 0;

    // Kropka dziesiętna tylko dla środkowej cyfry i tylko gdy showDecimal=true.
    if (showDecimal && d == 1) seg |= SEG_DP;

    write595(seg);
  }

  // 3) Zapal anodę właściwego koloru
  // --- SPRZĘTOWY WYBÓR KOLORU I KOREKTA JASNOŚCI W ISR ---
  uint8_t activeColor = 0;  // 0 = Zielony, 1 = Czerwony

  if (blinkCount) {
    activeColor = (blinkMode == BLINK_SOLID) ? blinkColor : (blinkPhase % 2);
  } else if (fsmState == FSM_STARTUP) {
    activeColor = g ? 0 : 1;
  } else if (isAssign) {
    activeColor = (fsmState == FSM_ASSIGN_A || fsmState == FSM_ASSIGN_A_REMOVE || fsmState == FSM_ASSIGN_SAVE) ? 0 : 1;
  } else {
    activeColor = (fsmState == FSM_SHOW_A || fsmState == FSM_ERR_A) ? 0 : 1;
  }

  // Sterowanie sprzętowym rejestrem Timer1 (Wysoka wartość = Ciemniejszy ekran)
  if (activeColor == 1) {
    // KOREKTA DLA CZERWONEGO:
    // Przeliczamy systemBrightness na postać "aktywnej intensywności świecenia" (255 - systemBrightness),
    // skalujemy ją współczynnikiem RED_LEVEL, a następnie odwracamy z powrotem do logiki rejestru OCR1A.
    uint8_t activeIntensity = 255 - systemBrightness;
    uint8_t scaledIntensity = ((uint16_t)activeIntensity * RED_LEVEL) >> 8;

    OCR1A = 255 - scaledIntensity;
  } else {
    // KOREKTA DLA ZIELONEGO:
    // Zielony pobiera bezpośrednią, wyliczoną w pętli loop wartość tłumienia
    OCR1A = systemBrightness;
  }

  // --- 4) Fizyczne załączenie właściwej anody ---
  if (blinkCount && !blinkOn) return;  // Wygaszenie w fazie OFF blinku

  if (activeColor == 0) {
    digitalWrite(anode_GREEN[d], LOW);
  } else {
    digitalWrite(anode_RED[d], LOW);
  }

  // Następna cyfra.
  d = (d + 1) % 3;
}

/* =========================================================================================
 *  SETUP / LOOP
 * ========================================================================================= */

void setup() {
  //Serial.begin(115200);
  // PWM na Timer1 (pin 9/10 w ATmega328P zależy od płytki).
  // Preskaler zmieniasz po to, by dopasować częstotliwość PWM do tego,
  // żeby OE nie powodowało słyszalnych/wyczuwalnych artefaktów.
  // TCCR1B = (TCCR1B & 0xF8) | 0x02;

  // --- KONFIGURACJA SPRZĘTOWA TIMER1 DLA PINU 9 (OE) ---
  TCCR1A = 0;
  TCCR1B = 0;

  // Tryb Fast PWM 8-bit, czyszczenie OC1A przy dopasowaniu (Clear on Compare Match)
  TCCR1A |= (1 << COM1A1) | (1 << WGM10);  // Standardowy Fast PWM 8-bit (bez COM1A0!)
  TCCR1B |= (1 << WGM12) | (1 << CS11);    // Preskaler 8 -> f = 16MHz / (8 * 256) = 7.812 kHz

  OCR1A = 0;  // 0 = stan niski na OE przez 100% czasu = maksymalna jasność startowa

  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(oePin, OUTPUT);

  for (int i = 0; i < 3; i++) {
    pinMode(anode_GREEN[i], OUTPUT);
    pinMode(anode_RED[i], OUTPUT);
    digitalWrite(anode_GREEN[i], HIGH);
    digitalWrite(anode_RED[i], HIGH);
  }

  // Bufory startowo w stanie błędu/blanku.
  fillG(displayBuf[0]);
  fillG(displayBuf[1]);

  Wire.begin();

  // Spróbuj wczytać mapę (jeśli nie ma/CRC nie pasuje -> eepromMapValid=false).
  eepromMapValid = loadMapFromEEPROM();

  // Skan na starcie:
  // - jeśli 0 czujników -> wejdź w ASSIGN
  // - jeśli >0 -> normalna praca (mapa albo fallback na "pierwsze dwa").
  ScanResult r = scanBusAll();

  if (r.count == 0) {
    fsmState = FSM_ASSIGN_A;
    assignStable = 0;
    lastAssignScanTick = millis();
  } else {
    if (eepromMapValid) applySavedMapToBus();
    else takeFirstTwoFromBus();
  }

  // Startowo czyścimy średnie.
  resetAvg(0);
  resetAvg(1);

  // Timer2: 1 kHz ISR do multiplexu.
  cli();
  TCCR2A = (1 << WGM21);   // CTC
  TCCR2B = (1 << CS22);    // preskaler 64
  OCR2A = 249;             // 1 kHz
  TIMSK2 = (1 << OCIE2A);  // enable compare match A interrupt
  sei();

  lastFSMTick = lastLdrTick = lastScanTick = millis();
}

void loop() {
  unsigned long now = millis();

  // --- Jasność zależna od LDR (Wysoki odczyt = Ciemność) ---
  if (now - lastLdrTick >= LDR_MS) {  // LDR_MS = 100 dla szybszej reakcji
    int raw = analogRead(ldrPin);

    // Diagnostyka na Serial Monitor (raz na sekundę)
    // static uint8_t serialDivider = 0;
    // if (++serialDivider >= 10) {
    //   serialDivider = 0;
    //   Serial.print(F("LDR Raw: "));
    //   Serial.print(raw);
    //   Serial.print(F(" | PWM Out: "));
    //   Serial.println(systemBrightness);
    // }

    // MAPOWANIE DIRECT DLA WYSOKIEGO STANU OE:
    // raw blisko ldrInMin (Jasno) -> 5   (Krótki stan HIGH na OE = Max jasność)
    // raw blisko ldrInMax (Ciemno) -> 240 (Długi stan HIGH na OE = Ekran mocno wygaszony)
    int targetPwm = constrain(map(raw, ldrInMin, ldrInMax, 0, 250), 0, 250);

    // Filtr dolnoprzepustowy wygładzający zmiany
    systemBrightness = (uint8_t)((systemBrightness * 7 + targetPwm) >> 3);

    lastLdrTick = now;
  }

  // --- Rescan hot-plug ---
  if (now - lastScanTick >= RESCAN_MS) {
    // Zasada:
    // - jeśli jest mapa: dopasowuj TYLKO savedA/savedB (kolory zawsze spójne)
    // - jeśli brak mapy: bierz pierwsze dwa (fallback)
    if (eepromMapValid) applySavedMapToBus();
    else takeFirstTwoFromBus();

    lastScanTick = now;
  }

  // --- Krok FSM (wolniejszy tick UX) ---
  if (now - lastFSMTick >= FSM_TICK_MS) {
    fsmStep();
    lastFSMTick = now;
  }
  // --- Zadanie 4: Nieblokujący odczyt temperatur i start nowej konwersji ---
  static unsigned long lastConversionTick = 0;
  if (now - lastConversionTick >= 1000) {  // Wykonuj dokładnie co 1 sekundę
    lastConversionTick = now;

    // 1. Zbieramy wyniki z poprzedniej konwersji (zleconej 1s temu)
    if (fsmState < FSM_ASSIGN_A) {  // Tylko w trybie normalnym
      updateSensor(0);              // Aktualizacja kanału A (Fails, Warmup, Avg)
      updateSensor(1);              // Aktualizacja kanału B (Fails, Warmup, Avg)
    }

    // 2. Natychmiast startujemy nową konwersję, która będzie gotowa za sekundę
    sensors.requestTemperatures();
  }
}
