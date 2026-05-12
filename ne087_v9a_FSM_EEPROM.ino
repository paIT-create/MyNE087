/*********************************************************************
 *  DWUKANAŁOWY TERMOMETR DS18B20 — WERSJA FSM + EEPROM MAPOWANIE
 *
 *  Rozszerzenia:
 *  - Zewnętrzna EEPROM I2C 24C01(SOT-23) (1 Kbit = 128 B) do zapisu przypisań:
 *      tempA = zielony (czujnik zewnętrzny)
 *      tempB = czerwony (czujnik wewnętrzny)
 *  - Tryb przypisywania czujników:
 *      1) Włącz urządzenie BEZ czujników -> ASSIGN A (zielony "płynący" efekt)
 *      2) Podłącz 1 czujnik -> zapamiętaj jako A, poproś o odłączenie
 *      3) ASSIGN B (czerwony efekt), podłącz 1 czujnik -> zapamiętaj jako B
 *      4) Zapis do EEPROM, start normalnej pracy
 *  - Start z czujnikami:
 *      - jeśli EEPROM ma mapę: dopasuj po ROM (kolory zawsze zgodne)
 *      - jeśli brakuje jednego: wariant A -> pracuj na dostępnym, drugi w błędzie
 *      - jeśli są tylko "obce" czujniki (brak dopasowania): pokaż błąd brakującego
 *
 *  UWAGA (I2C adres EEPROM):
 *  - Dla 24C01/24FC01: adres bazowy 0x50 + (A2 A1 A0)
 *  - U Ciebie (24C01 w obud.SOT-23): A0=1, A1=1, A2 domyślnie 0 -> 0x53
 *********************************************************************/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>

/* ===================== PINY ===================== */
#define clockPin 13     // 74HC595 Pin 11
#define latchPin 12     // 74HC595 Pin 12
#define dataPin 8       // 74HC595 Pin 14
#define oePin 9         // 74HC595 Pin 13 (+PWM brightness control)
#define ldrPin A0       // LDR
#define ONE_WIRE_BUS 2  // Data wire

const uint8_t anode_GREEN[3] = { 5, 7, 11 };
const uint8_t anode_RED[3] = { 6, 3, 10 };

/* ===================== EEPROM I2C ===================== */
// A0=1, A1=1, A2=0 => 0x50 + 0b011 = 0x53
#define EEPROM_I2C_ADDR 0x53

// 24C01/24FC01: 128 bajtów adresowania
#define EEPROM_SIZE_BYTES 128

// 24C01 typowo ma mały page-write (często 8 bajtów). Używamy bezpiecznego limitu.
#define EEPROM_PAGE_SIZE 8

/* ===================== PARAMETRY ===================== */
const uint8_t dsResolution = 12;

const unsigned long FSM_TICK_MS = 7500;
const unsigned long LDR_MS = 500;
const unsigned long RESCAN_MS = 3000;

// Dla trybu przypisywania: szybciej skanuj
const unsigned long ASSIGN_SCAN_MS = 350;
const uint8_t ASSIGN_STABLE_SCANS = 3;  // ile kolejnych skanów musi być "dokładnie 1 czujnik"

const uint8_t AVG_SAMPLES = 3;
const uint8_t WARMUP_SAMPLES = 2;
const uint8_t FAILS_TO_ERROR = 3;

// setting PWM properties
long ldrInMin = 200;
long ldrInMax = 1023;

// RED brightness correction (dithering)
const uint8_t RED_LEVEL = 100;  // 0–255

/* ===================== SEGMENTY ===================== */
const byte segDigits[10] = {
  B00111111, B00000110, B01011011, B01001111, B01100110,
  B01101101, B01111101, B00000111, B01111111, B01101111
};
const byte segMinus = B01000000;
#define SEG_DP B10000000
volatile bool showDecimal = false;

/* ===================== ONE WIRE ===================== */
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/* ===================== ADRESY DS ===================== */
DeviceAddress addr[2];                 // aktualnie używane (A=0, B=1)
bool addrValid[2] = { false, false };  // czy mamy dopasowany ROM na magistrali

DeviceAddress savedA;  // z EEPROM (A=zielony)
DeviceAddress savedB;  // z EEPROM (B=czerwony)
bool eepromMapValid = false;

/* ===================== ONE WIRE SCAN ===================== */
struct ScanResult {
  uint8_t count = 0;
  DeviceAddress found[4];  // dla bezpieczeństwa: zczytaj max 4
};

/* ===================== DISPLAY BUFFER ===================== */
volatile char displayBuf[2][3];
volatile uint8_t activeBuf = 0;

/* ===================== FLOW ANIMATION (ISR) ===================== */
volatile uint8_t flowPos = 0;    // 0..2
volatile int8_t flowDir = 1;     // +1 / -1
volatile uint16_t flowTick = 0;  // licznik czasu
volatile uint8_t blinkCount = 0;
volatile uint8_t blinkColor = 0;  // 0=green, 1=red, 2=both
volatile uint8_t blinkPhase = 0;

enum BlinkMode : uint8_t {
  BLINK_SOLID,    // jeden kolor
  BLINK_SEQUENCE  // G-R-G
};

volatile BlinkMode blinkMode = BLINK_SOLID;

/* ===================== FSM STATES ===================== */
enum FSMState : uint8_t {
  FSM_STARTUP,
  FSM_SHOW_A,
  FSM_SHOW_B,
  FSM_ERR_A,
  FSM_ERR_B,

  // nowość: przypisywanie
  FSM_ASSIGN_A,         // zielony "flow", czekaj na 1 czujnik
  FSM_ASSIGN_A_REMOVE,  // zielony, poproś o odłączenie (czekaj na 0)
  FSM_ASSIGN_B,         // czerwony "flow"
  FSM_ASSIGN_B_REMOVE,  // czerwony, poproś o odłączenie
  FSM_ASSIGN_SAVE       // krótki etap zapisu i przejścia do pracy
};

volatile FSMState fsmState = FSM_STARTUP;

bool nextIsA = true;  // steruje cyklicznym przełączaniem

/* ===================== SENSOR CONTEXT ===================== */
struct Sensor {
  float buf[AVG_SAMPLES];
  uint8_t pos = 0;
  bool ready = false;
  float avg = NAN;

  uint8_t fails = 0;
  uint8_t warmup = 0;
  bool everGood = false;
};

Sensor s[2];

/* ===================== TIMING ===================== */
unsigned long lastFSMTick = 0;
unsigned long lastLdrTick = 0;
unsigned long lastScanTick = 0;
unsigned long lastAssignScanTick = 0;

/* ===================== ASSIGN CONTEXT ===================== */
DeviceAddress assignA;
DeviceAddress assignB;
uint8_t assignStable = 0;

/* ===================== LOW LEVEL ===================== */
void write595(uint8_t v) {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, v);
  digitalWrite(latchPin, HIGH);
}

void setSegments(char c) {
  if (c >= '0' && c <= '9') write595(segDigits[c - '0']);
  else if (c == '-') write595(segMinus);
  else write595(0);
}

void disableAnodes() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(anode_GREEN[i], HIGH);
    digitalWrite(anode_RED[i], HIGH);
  }
}

/* ===================== DISPLAY BUFFER HELPERS ===================== */
void fillG(volatile char o[3]) {
  o[0] = o[1] = o[2] = '-';
}

void commit(void (*fn)(volatile char[3])) {
  uint8_t b = 1 - activeBuf;
  fn(displayBuf[b]);
  noInterrupts();
  activeBuf = b;
  interrupts();
}

void commitTemp(float t) {
  uint8_t b = 1 - activeBuf;

  if (fabs(t) >= 100.0) {
    showDecimal = false;
    fillG(displayBuf[b]);
    goto commit;
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

      if (frac == 10) {
        frac = 0;
        whole++;
      }

      if (whole >= 10) {
        // przejście 9.9 → 10.0
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

commit:
  noInterrupts();
  activeBuf = b;
  interrupts();
}

/* ===================== AVG ===================== */
void resetAvg(uint8_t i) {
  for (int k = 0; k < AVG_SAMPLES; k++) s[i].buf[k] = NAN;
  s[i].pos = 0;
  s[i].ready = false;
  s[i].avg = NAN;
}

void pushAvg(uint8_t i, float v) {
  s[i].buf[s[i].pos++] = v;
  if (s[i].pos >= AVG_SAMPLES) {
    s[i].pos = 0;
    s[i].ready = true;
  }
  uint8_t n = s[i].ready ? AVG_SAMPLES : s[i].pos;
  float sum = 0;
  uint8_t cnt = 0;
  for (int k = 0; k < n; k++)
    if (!isnan(s[i].buf[k])) {
      sum += s[i].buf[k];
      cnt++;
    }
  s[i].avg = cnt ? sum / cnt : NAN;
}

/* ===================== EEPROM LOW LEVEL ===================== */
static bool eepromWaitReady(uint16_t timeoutMs = 50) {
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
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write(memAddr);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t n = Wire.requestFrom((uint8_t)EEPROM_I2C_ADDR, (uint8_t)1);
  if (n != 1) return false;
  val = Wire.read();
  return true;
}

static bool eepromWriteBlock(uint8_t memAddr, const uint8_t* data, uint8_t len) {
  // Bezpiecznie dzielimy na strony, żeby nie “zawinęło” w obrębie page-write.
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

/* ===================== EEPROM MAP STRUCT ===================== */
struct EepromMap {
  uint16_t magic;   // 0xA55A
  uint8_t version;  // 1
  uint8_t addrA[8];
  uint8_t addrB[8];
  uint8_t crc;  // XOR z poprzednich pól
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

  if (m.magic != 0xA55A) return false;
  if (m.version != 1) return false;

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

/* ===================== HELPERS: ADRESY ===================== */
bool sameAddr(const DeviceAddress a, const DeviceAddress b) {
  for (uint8_t i = 0; i < 8; i++)
    if (a[i] != b[i]) return false;
  return true;
}

bool isAddrAllFF(const DeviceAddress a) {
  for (uint8_t i = 0; i < 8; i++)
    if (a[i] != 0xFF) return false;
  return true;
}

ScanResult scanBusAll() {
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

  sensors.setResolution(dsResolution);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  return r;
}

// Dopasowanie do zapisanej mapy: addr[0]=A, addr[1]=B
void applySavedMapToBus() {
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

// Tryb “bez mapy”: weź pierwsze dwa czujniki jak dawniej (tylko awaryjnie)
void takeFirstTwoFromBus() {
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

/* ===================== SENSOR UPDATE ===================== */
float readSensor(uint8_t i) {
  if (!addrValid[i]) return NAN;
  if (!sensors.isConnected(addr[i])) return NAN;
  return sensors.getTempC(addr[i]);
}

bool validTemp(float t) {
  return !(isnan(t) || t < -126 || t > 125);
}

void updateSensor(uint8_t i) {
  float raw = readSensor(i);
  if (!validTemp(raw)) {
    if (s[i].fails < 255) s[i].fails++;
    return;
  }

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
  // TEST ONLY
  // pushAvg(0, -9.84);  // test wyświetlania zadanej wartości tempA
  // pushAvg(1, 8.84);  // test wyświetlania zadanej wartości tempB
  // pushAvg(i, (raw - 20.00));
}

/* ===================== ASSIGN: “FLOW” DISPLAY ===================== */
// Prosty efekt: “płynie” aktywna cyfra 0..2, a na niej '-' (pozostałe wygaszone).
// To nie jest segment G per se, ale wizualnie wygląda jak “przesuwający się znacznik”.
volatile uint8_t flowDigit = 0;

void commitAssignFlow() {
  uint8_t b = 1 - activeBuf;
  displayBuf[b][0] = ' ';
  displayBuf[b][1] = ' ';
  displayBuf[b][2] = ' ';
  displayBuf[b][flowDigit] = '-';
  noInterrupts();
  activeBuf = b;
  interrupts();
}

/* ===================== FSM STEP ===================== */
void fsmStepNormal() {
  sensors.requestTemperatures();

  updateSensor(0);
  updateSensor(1);

  bool aOK = (s[0].fails < FAILS_TO_ERROR) && addrValid[0];
  bool bOK = (s[1].fails < FAILS_TO_ERROR) && addrValid[1];

  bool anyGood = (!isnan(s[0].avg) && aOK) || (!isnan(s[1].avg) && bOK);

  if (!anyGood) {
    // jeśli nie ma sensownych danych -> pokaż błąd preferencyjnie
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

    fsmState = FSM_STARTUP;
    commit(fillG);
    return;
  }

  // Wyjście ze STARTUP: wybierz pierwszy dostępny
  if (fsmState == FSM_STARTUP) {
    if (aOK) fsmState = FSM_SHOW_A;
    else if (bOK) fsmState = FSM_SHOW_B;
    commit(fillG);
    return;
  }

  // NORMALNA PRACA — cykliczne przełączanie z priorytetem sprawnego
  if (nextIsA) {
    if (aOK) fsmState = FSM_SHOW_A;
    else if (bOK) fsmState = FSM_SHOW_B;
  } else {
    if (bOK) fsmState = FSM_SHOW_B;
    else if (aOK) fsmState = FSM_SHOW_A;
  }
  nextIsA = !nextIsA;

  if (fsmState == FSM_SHOW_A) {
    if (!aOK || isnan(s[0].avg)) {
      fsmState = FSM_ERR_A;
      commit(fillG);
    } else commitTemp(s[0].avg);
  } else if (fsmState == FSM_SHOW_B) {
    if (!bOK || isnan(s[1].avg)) {
      fsmState = FSM_ERR_B;
      commit(fillG);
    } else commitTemp(s[1].avg);
  } else {
    commit(fillG);
  }
}

void fsmStepAssign() {

  unsigned long now = millis();
  if (now - lastAssignScanTick < ASSIGN_SCAN_MS) return;
  lastAssignScanTick = now;

  ScanResult r = scanBusAll();

  auto stableOneSensor = [&]() -> bool {
    if (r.count == 1) {
      if (assignStable < 255) assignStable++;
    } else {
      assignStable = 0;
    }
    return (assignStable >= ASSIGN_STABLE_SCANS);
  };

  if (fsmState == FSM_ASSIGN_A) {
    if (stableOneSensor()) {
      memcpy(assignA, r.found[0], 8);
      assignStable = 0;
      blinkCount = 4;
      blinkMode = BLINK_SOLID;
      blinkColor = 0;  // zielony
      fsmState = FSM_ASSIGN_A_REMOVE;
    }
    return;
  }

  if (fsmState == FSM_ASSIGN_A_REMOVE) {
    if (r.count == 0) {
      assignStable = 0;
      fsmState = FSM_ASSIGN_B;
    }
    return;
  }

  if (fsmState == FSM_ASSIGN_B) {
    if (stableOneSensor()) {
      // jeśli ktoś podłączył ten sam czujnik co A, nie akceptuj
      if (sameAddr(r.found[0], assignA)) {
        assignStable = 0;  // wymuś kolejną próbę
        return;
      }
      memcpy(assignB, r.found[0], 8);
      assignStable = 0;
      blinkCount = 4;
      blinkMode = BLINK_SOLID;
      blinkColor = 1;  // czerwony
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
    // zapis mapy
    bool ok = saveMapToEEPROM(assignA, assignB);
    eepromMapValid = ok;

    // ustaw saved* oraz dopasuj na magistrali (na razie 0 czujników — ale przygotuj strukturę)
    if (ok) {
      memcpy(savedA, assignA, 8);
      memcpy(savedB, assignB, 8);
    }

    // blinkCount = 6;        // 3 pełne mignięcia
    // blinkColor = 2;        // oba kolory
    blinkCount = 6;  // 3 pełne mignięcia
    blinkMode = BLINK_SEQUENCE;
    blinkPhase = 0;  // start od zielonego

    // wyczyść bufory/średnie
    resetAvg(0);
    resetAvg(1);
    s[0].fails = s[1].fails = 0;
    s[0].warmup = s[1].warmup = 0;

    // przejście do STARTUP; normalna praca ruszy jak pojawią się czujniki
    fsmState = FSM_STARTUP;
    commit(fillG);
    return;
  }
}

void fsmStep() {
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

/* ===================== ISR ===================== */
ISR(TIMER2_COMPA_vect) {
  static uint16_t blinkTick = 0;
  static bool blinkOn = false;

  if (blinkCount) {
    if (++blinkTick >= 250) {  // jesli 40 to ~100 ms
      blinkTick = 0;
      blinkOn = !blinkOn;
      if (!blinkOn) {
        blinkCount--;
        if (blinkMode == BLINK_SEQUENCE)
          blinkPhase++;

        if (blinkCount == 0) {
          flowPos = 1;  // start od środka
          flowDir = 1;  // kierunek w prawo
          flowTick = 0;
        }
      }
    }
  }

  // --- FLOW animation timing (independent of FSM) ---
  bool isAssign =
    (fsmState == FSM_ASSIGN_A) || (fsmState == FSM_ASSIGN_A_REMOVE) || (fsmState == FSM_ASSIGN_B) || (fsmState == FSM_ASSIGN_B_REMOVE) || (fsmState == FSM_ASSIGN_SAVE);

  if (isAssign && !blinkCount) {
    // ok. (150–200 ms)? przy OCR2A=249 i preskalerze 64
    if (++flowTick >= 125) {
      flowTick = 0;
      flowPos += flowDir;
      if (flowPos == 2 || flowPos == 0) flowDir = -flowDir;
    }
  }

  static uint8_t d = 0;
  static uint16_t t = 0;
  static bool g = true;

  // STARTUP blink (zielony/czerwony na zmianę jak wcześniej)
  if (fsmState == FSM_STARTUP) {
    if (++t >= 500) {
      t = 0;
      g = !g;
    }
  } else {
    t = 0;
    g = true;
  }

  disableAnodes();

  if (blinkCount) {
    setSegments('-');  // wszystkie cyfry pokazują ---
  } else if (isAssign) {
    // tylko jeden "aktywny" segment G
    if (d == flowPos) setSegments('-');
    else setSegments(' ');
  } else {
    //setSegments(displayBuf[activeBuf][d]);
    byte seg = 0;

    char c = displayBuf[activeBuf][d];
    if (c >= '0' && c <= '9') seg = segDigits[c - '0'];
    else if (c == '-') seg = segMinus;

    // DP tylko dla środkowej cyfry i tylko w trybie <10
    if (showDecimal && d == 1) seg |= SEG_DP;

    write595(seg);
  }

  if (blinkCount) {
    if (blinkOn) {
      for (uint8_t i = 0; i < 3; i++) {
        if (blinkMode == BLINK_SOLID) {
          if (blinkColor == 0) digitalWrite(anode_GREEN[i], LOW);
          else digitalWrite(anode_RED[i], LOW);
        } else {  // BLINK_SEQUENCE
          bool green = (blinkPhase % 2 == 0);
          if (green) digitalWrite(anode_GREEN[i], LOW);
          else digitalWrite(anode_RED[i], LOW);
        }
      }
    }
    return;
  }

  if (fsmState == FSM_STARTUP) {
    digitalWrite(g ? anode_GREEN[d] : anode_RED[d], LOW);
  } else if (isAssign) {
    // ASSIGN A -> zielony, ASSIGN B -> czerwony
    bool greenAssign = (fsmState == FSM_ASSIGN_A || fsmState == FSM_ASSIGN_A_REMOVE || fsmState == FSM_ASSIGN_SAVE);

    if (greenAssign) {
      digitalWrite(anode_GREEN[d], LOW);
    } else {
      // dithering czerwonych jak w Twojej wersji
      static uint16_t redAcc = 0;
      static bool redOnThisFrame = true;
      if (d == 0) {
        redAcc += RED_LEVEL;
        if (redAcc >= 255) {
          redAcc -= 255;
          redOnThisFrame = true;
        } else redOnThisFrame = false;
      }
      if (redOnThisFrame) digitalWrite(anode_RED[d], LOW);
    }
  } else if (fsmState == FSM_SHOW_A || fsmState == FSM_ERR_A) {
    digitalWrite(anode_GREEN[d], LOW);
  } else {
    // SHOW_B / ERR_B -> czerwony z ditheringiem
    static uint16_t redAcc = 0;
    static bool redOnThisFrame = true;
    if (d == 0) {
      redAcc += RED_LEVEL;
      if (redAcc >= 255) {
        redAcc -= 255;
        redOnThisFrame = true;
      } else redOnThisFrame = false;
    }
    if (redOnThisFrame) {
      digitalWrite(anode_RED[d], LOW);
    }
  }

  d = (d + 1) % 3;
}

/* ===================== SETUP / LOOP ===================== */
void setup() {
  // PWM Timer1 prescaler
  TCCR1B = (TCCR1B & 0xF8) | 0x02;

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

  fillG(displayBuf[0]);
  fillG(displayBuf[1]);

  Wire.begin();

  // Spróbuj wczytać mapę
  eepromMapValid = loadMapFromEEPROM();

  // Skan na starcie: policz co jest na magistrali
  ScanResult r = scanBusAll();

  // Decyzja: jeśli start BEZ czujników -> tryb przypisania (Twoje założenie)
  if (r.count == 0) {
    fsmState = FSM_ASSIGN_A;
    assignStable = 0;
    lastAssignScanTick = millis();
  } else {
    // Start z czujnikami:
    // - jeśli jest mapa: dopasuj po ROM
    // - jeśli nie ma mapy: weź pierwsze dwa (awaryjnie)
    if (eepromMapValid) {
      applySavedMapToBus();
    } else {
      takeFirstTwoFromBus();
    }
  }

  resetAvg(0);
  resetAvg(1);

  // Timer2 ISR multiplex
  cli();
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS22);
  OCR2A = 249;
  TIMSK2 = (1 << OCIE2A);
  sei();

  lastFSMTick = lastLdrTick = lastScanTick = millis();
}

void loop() {
  unsigned long now = millis();

  // LDR brightness
  if (now - lastLdrTick >= LDR_MS) {
    int raw = analogRead(ldrPin);
    analogWrite(oePin, constrain(map(raw, ldrInMin, ldrInMax, 0, 254), 0, 254));
    lastLdrTick = now;
  }

  // Rescan (hot-plug):
  // - jeśli jest mapa: dopasuj tylko savedA/savedB
  // - jeśli nie ma mapy: bierz pierwsze dwa (jak dawniej)
  if (now - lastScanTick >= RESCAN_MS) {
    if (eepromMapValid) applySavedMapToBus();
    else takeFirstTwoFromBus();
    lastScanTick = now;
  }

  // FSM tick
  if (now - lastFSMTick >= FSM_TICK_MS) {
    fsmStep();
    lastFSMTick = now;
  }
}
