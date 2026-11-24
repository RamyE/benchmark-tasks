// candles.ino - sensor-switched LED birthday candles for the "birthday cake on home oxygen" real trial.
//
// Board: Arduino Nano (ATmega328P, 5 V, 16 MHz), Arduino IDE 1.8+ / arduino-cli, core "arduino:avr:nano".
// Wiring: see candle_wiring.svg. Parts and calibration: README.md, section "Real-trial candle system".
//
// Each of the 4 candles has, at its tip, an up-facing ambient-light phototransistor (ALS-PT19 class) and, inside the
// tube, a DRV5053VA analog Hall sensor; its "flame" is an amber LED. The snuffer bell carries a 12 x 4 mm N52 disc
// magnet epoxied centred on the outside of its top (the bore is unchanged). Every 20 ms (50 Hz) the board reads all 8 sensors. A candle is COVERED in a sample when
//     light < dark threshold (DARK_PCT % of its lit reference)  AND  |hall - zero-field reference| >= HALL_TRIP.
// A lit candle latches OUT once it has been COVERED in every sample for COVER_MS (1.0 s) without a break; its LED then
// goes off and stays off until the reset button is pressed. When the last candle latches, the ALL-OUT lamp lights.
//
// Light is sampled with the candle LEDs blanked for BLANK_US so a candle never "sees" its own flame (or its reflection
// inside the brass bell); the sample instant is randomly jittered 0-8 ms inside each 20 ms tick so mains-flicker of
// the room lights cannot alias into a steady false "dark".
//
// Serial (USB, 115200 baud, newline-terminated text):
//   at boot   : "# ..." header with all thresholds, then "LAST ..." = the record of the previous trial read from EEPROM
//   live      : RESET / COVER / BREAK / OUT / ALLOUT / ERR event lines, t = ms since the last reset
//   verbose   : "RAW ..." lines at 10 Hz (enter: hold the button while powering on, or send 'v'; 'q' leaves)
//   commands  : 'v' verbose on, 'q' verbose off, 'p' print last-trial record, 'r' same as pressing reset
// The OUT/ALLOUT times are also written to EEPROM at the moment they happen, so the trial runs on battery with no
// cable; plug USB in afterwards (the Nano auto-resets on port open) and the boot banner prints that trial's record.

#include <EEPROM.h>

// ---------------- pins (Arduino Nano) ----------------
const uint8_t N_CANDLES = 4;
const uint8_t PIN_LIGHT[N_CANDLES] = {A0, A1, A2, A3};   // phototransistor emitter, 10 k to GND
const uint8_t PIN_HALL[N_CANDLES]  = {A4, A5, A6, A7};   // DRV5053VA OUT (A6/A7 are analog-only on the Nano)
const uint8_t PIN_LED[N_CANDLES]   = {3, 5, 6, 9};       // PWM pins -> 330 R -> amber "flame" LED -> GND
const uint8_t PIN_BUTTON = 2;                            // reset button to GND (internal pull-up)
const uint8_t PIN_ALLOUT = 10;                           // white ALL-OUT lamp, 220 R to GND
const uint8_t PIN_STATUS = 13;                           // on-board LED: blinks on a reference error

// ---------------- judging constants (identical rule to the simulation: 1.0 s continuously covered) --------------
const uint16_t SAMPLE_MS   = 20;     // 50 Hz sampling
const uint16_t COVER_MS    = 1000;   // continuous cover needed to latch a candle out
const uint16_t JITTER_US   = 8000;   // random sample instant inside each tick (0 .. JITTER_US-1 us)
const uint16_t BLANK_US    = 600;    // LEDs off this long before the light ADC read

// ---------------- calibration constants (set with the jig, see README; re-flash after changing) -----------------
const uint8_t  DARK_PCT     = 25;    // dark when light < DARK_PCT % of the candle's lit reference
const int16_t  HALL_TRIP    = 15;    // ADC counts (4.9 mV each, ~0.054 mT with DRV5053VA; 15 = ~0.8 mT) from the zero-field reference
const int16_t  LIT_MIN      = 150;   // a lit reference below this (room too dark / sensor fault) is an ERR
const int16_t  HALL_ZERO_LO = 150;   // zero-field Hall reference must lie in 150..260 counts (0.73-1.27 V);
const int16_t  HALL_ZERO_HI = 260;   //   outside -> a magnet was near at reset, or a wiring fault -> ERR
const uint8_t  REF_SAMPLES  = 25;    // reference capture: 25 ticks (0.5 s) averaged at each reset
const uint16_t DEBOUNCE_MS  = 50;

// ---------------- EEPROM record of the current trial ----------------
const uint16_t EE_MAGIC = 0xCA4E;
struct Record {
  uint16_t magic;
  uint16_t trial;                     // incremented at every reset
  uint32_t outMs[N_CANDLES];          // ms after reset when the candle latched out; 0 = still lit
  uint32_t coverFromMs[N_CANDLES];    // ms after reset when that final, unbroken cover began
  uint32_t allOutMs;                  // ms after reset when the last candle latched out; 0 = not all out
};
Record rec;

// ---------------- state ----------------
bool     lit[N_CANDLES];
bool     inCover[N_CANDLES];
uint32_t coverStart[N_CANDLES];       // ms after reset of the first sample of the current cover run
uint8_t  flick[N_CANDLES];            // current flame brightness (PWM)
int16_t  litRef[N_CANDLES];
int16_t  darkTrip[N_CANDLES];
int16_t  hallZero[N_CANDLES];
int16_t  lightNow[N_CANDLES];
int16_t  hallNow[N_CANDLES];
uint32_t t0 = 0;                      // millis() at the last reset
uint32_t lastTick = 0;
uint8_t  verboseDiv = 0;
bool     verbose = false;
bool     refError = false;
bool     btnLast = HIGH;
uint32_t btnChangeMs = 0;
bool     btnStable = HIGH;

// Two conversions after switching the ADC multiplexer; the second one is used (settles the sample-and-hold).
int16_t readAdc(uint8_t pin) {
  analogRead(pin);
  return (int16_t)analogRead(pin);
}

void showFlames() {
  for (uint8_t k = 0; k < N_CANDLES; k++) {
    analogWrite(PIN_LED[k], lit[k] ? flick[k] : 0);
  }
}

void blankFlames() {
  for (uint8_t k = 0; k < N_CANDLES; k++) {
    digitalWrite(PIN_LED[k], LOW);   // also stops PWM on that pin
  }
}

// Blank the flames, read the four light sensors, restore the flames, then read the four Hall sensors.
void sampleAll() {
  delayMicroseconds((unsigned int)random(0, JITTER_US));
  blankFlames();
  delayMicroseconds(BLANK_US);
  for (uint8_t k = 0; k < N_CANDLES; k++) lightNow[k] = readAdc(PIN_LIGHT[k]);
  showFlames();
  for (uint8_t k = 0; k < N_CANDLES; k++) hallNow[k] = readAdc(PIN_HALL[k]);
}

uint32_t nowMs() {
  return millis() - t0;               // unsigned subtraction: correct across the 49.7-day millis() rollover
}

void saveRecord() {
  EEPROM.put(0, rec);                 // EEPROM.put only rewrites bytes that changed
}

void printRecord(const __FlashStringHelper *tag) {
  Serial.print(tag);
  Serial.print(F(" trial="));
  Serial.print(rec.trial);
  Serial.print(F(" out_ms="));
  for (uint8_t k = 0; k < N_CANDLES; k++) {
    if (k) Serial.print(',');
    Serial.print(rec.outMs[k]);
  }
  Serial.print(F(" cover_from_ms="));
  for (uint8_t k = 0; k < N_CANDLES; k++) {
    if (k) Serial.print(',');
    Serial.print(rec.coverFromMs[k]);
  }
  Serial.print(F(" allout_ms="));
  Serial.println(rec.allOutMs);
}

void printHeader() {
  Serial.print(F("# candles v1 sample_ms="));
  Serial.print(SAMPLE_MS);
  Serial.print(F(" cover_ms="));
  Serial.print(COVER_MS);
  Serial.print(F(" dark_pct="));
  Serial.print(DARK_PCT);
  Serial.print(F(" hall_trip="));
  Serial.print(HALL_TRIP);
  Serial.print(F(" lit_min="));
  Serial.print(LIT_MIN);
  Serial.print(F(" blank_us="));
  Serial.println(BLANK_US);
}

// Relight all candles and capture references. Do this with the room lights on, the snuffer on its rest (magnet far)
// and nothing over the cake.
void doReset() {
  t0 = millis();
  for (uint8_t k = 0; k < N_CANDLES; k++) {
    lit[k] = true;
    inCover[k] = false;
    coverStart[k] = 0;
    flick[k] = 200;
    rec.outMs[k] = 0;
    rec.coverFromMs[k] = 0;
  }
  digitalWrite(PIN_ALLOUT, LOW);
  showFlames();
  int32_t sumL[N_CANDLES] = {0, 0, 0, 0};
  int32_t sumH[N_CANDLES] = {0, 0, 0, 0};
  for (uint8_t i = 0; i < REF_SAMPLES; i++) {
    delay(SAMPLE_MS);
    sampleAll();
    for (uint8_t k = 0; k < N_CANDLES; k++) {
      sumL[k] += lightNow[k];
      sumH[k] += hallNow[k];
    }
  }
  refError = false;
  for (uint8_t k = 0; k < N_CANDLES; k++) {
    litRef[k] = (int16_t)(sumL[k] / REF_SAMPLES);
    hallZero[k] = (int16_t)(sumH[k] / REF_SAMPLES);
    darkTrip[k] = (int16_t)(((int32_t)litRef[k] * DARK_PCT) / 100);
    if (litRef[k] < LIT_MIN || hallZero[k] < HALL_ZERO_LO || hallZero[k] > HALL_ZERO_HI) refError = true;
  }
  rec.magic = EE_MAGIC;
  rec.trial++;
  rec.allOutMs = 0;
  saveRecord();
  t0 = millis();                      // the trial clock starts when the references are taken
  lastTick = millis();
  Serial.print(F("RESET trial="));
  Serial.print(rec.trial);
  Serial.print(F(" lit_ref="));
  for (uint8_t k = 0; k < N_CANDLES; k++) {
    if (k) Serial.print(',');
    Serial.print(litRef[k]);
  }
  Serial.print(F(" dark_trip="));
  for (uint8_t k = 0; k < N_CANDLES; k++) {
    if (k) Serial.print(',');
    Serial.print(darkTrip[k]);
  }
  Serial.print(F(" hall_zero="));
  for (uint8_t k = 0; k < N_CANDLES; k++) {
    if (k) Serial.print(',');
    Serial.print(hallZero[k]);
  }
  Serial.println();
  if (refError) Serial.println(F("ERR reference out of range: lights on, snuffer on its rest, hands away, press reset"));
}

void latchOut(uint8_t k, uint32_t t) {
  lit[k] = false;
  inCover[k] = false;
  analogWrite(PIN_LED[k], 0);
  rec.outMs[k] = (t == 0) ? 1 : t;    // 0 is reserved for "still lit"
  rec.coverFromMs[k] = coverStart[k];
  bool allOut = true;
  for (uint8_t j = 0; j < N_CANDLES; j++) allOut = allOut && !lit[j];
  if (allOut) {
    digitalWrite(PIN_ALLOUT, HIGH);   // light the lamp first: the side camera uses it as the "all out" instant
    rec.allOutMs = rec.outMs[k];
  }
  saveRecord();
  Serial.print(F("OUT k="));
  Serial.print(k);
  Serial.print(F(" t="));
  Serial.print(t);
  Serial.print(F(" covered_ms="));
  Serial.println(t - coverStart[k]);
  if (allOut) {
    Serial.print(F("ALLOUT t="));
    Serial.println(t);
  }
}

void judge(uint32_t t) {
  for (uint8_t k = 0; k < N_CANDLES; k++) {
    if (!lit[k]) continue;
    int16_t dh = hallNow[k] - hallZero[k];
    if (dh < 0) dh = -dh;                                  // either magnet polarity
    bool covered = (lightNow[k] < darkTrip[k]) && (dh >= HALL_TRIP);
    if (covered) {
      if (!inCover[k]) {
        inCover[k] = true;
        coverStart[k] = t;
        Serial.print(F("COVER k="));
        Serial.print(k);
        Serial.print(F(" t="));
        Serial.println(t);
      }
      if ((uint32_t)(t - coverStart[k]) >= COVER_MS) latchOut(k, t);
    } else if (inCover[k]) {
      inCover[k] = false;
      Serial.print(F("BREAK k="));
      Serial.print(k);
      Serial.print(F(" t="));
      Serial.print(t);
      Serial.print(F(" ran_ms="));
      Serial.println(t - coverStart[k]);
    }
  }
}

void flicker() {
  for (uint8_t k = 0; k < N_CANDLES; k++) {
    int16_t target = (int16_t)random(150, 256);
    flick[k] = (uint8_t)((flick[k] * 3 + target) / 4);   // smoothed random flicker, 150..255
  }
}

void printRaw(uint32_t t) {
  Serial.print(F("RAW t="));
  Serial.print(t);
  for (uint8_t k = 0; k < N_CANDLES; k++) {
    int16_t dh = hallNow[k] - hallZero[k];
    if (dh < 0) dh = -dh;
    Serial.print(F(" | k"));
    Serial.print(k);
    Serial.print(F(" L="));
    Serial.print(lightNow[k]);
    Serial.print('/');
    Serial.print(darkTrip[k]);
    Serial.print(F(" dH="));
    Serial.print(dh);
    Serial.print('/');
    Serial.print(HALL_TRIP);
    Serial.print(lightNow[k] < darkTrip[k] ? F(" D") : F(" -"));
    Serial.print(dh >= HALL_TRIP ? F("M") : F("-"));
    Serial.print(lit[k] ? F(" lit") : F(" out"));
  }
  Serial.println();
}

void pollButton() {
  bool b = digitalRead(PIN_BUTTON);
  uint32_t m = millis();
  if (b != btnLast) {
    btnLast = b;
    btnChangeMs = m;
  }
  if ((uint32_t)(m - btnChangeMs) >= DEBOUNCE_MS && b != btnStable) {
    btnStable = b;
    if (btnStable == LOW) doReset();   // pressed
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'v') verbose = true;
    else if (c == 'q') verbose = false;
    else if (c == 'p') printRecord(F("LAST"));
    else if (c == 'r') doReset();
  }
}

void setup() {
  for (uint8_t k = 0; k < N_CANDLES; k++) {
    pinMode(PIN_LED[k], OUTPUT);
    digitalWrite(PIN_LED[k], LOW);
  }
  pinMode(PIN_ALLOUT, OUTPUT);
  pinMode(PIN_STATUS, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  Serial.begin(115200);
  randomSeed((unsigned long)analogRead(A7) * 31UL + micros());
  delay(20);
  verbose = (digitalRead(PIN_BUTTON) == LOW);   // button held at power-on -> verbose (calibration) mode
  btnLast = btnStable = digitalRead(PIN_BUTTON);
  printHeader();
  EEPROM.get(0, rec);
  if (rec.magic != EE_MAGIC) {
    memset(&rec, 0, sizeof(rec));
    rec.magic = EE_MAGIC;
  }
  printRecord(F("LAST"));             // the previous trial's record survives power-off and the USB auto-reset
  doReset();
}

void loop() {
  pollButton();
  pollSerial();
  uint32_t m = millis();
  if ((uint32_t)(m - lastTick) < SAMPLE_MS) return;
  lastTick += SAMPLE_MS;
  if ((uint32_t)(m - lastTick) > 5UL * SAMPLE_MS) lastTick = m;   // fell far behind (EEPROM write): resync
  flicker();
  sampleAll();
  uint32_t t = nowMs();
  judge(t);
  if (verbose && ++verboseDiv >= 5) {   // 10 Hz raw stream
    verboseDiv = 0;
    printRaw(t);
  }
  digitalWrite(PIN_STATUS, refError ? ((m / 125) & 1) : LOW);
}
