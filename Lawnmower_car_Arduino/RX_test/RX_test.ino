/*
  RC Input (PWM only) — Arduino MEGA
  - Auto-detects header row (EVEN vs ODD)
  - Per-channel polarity lock (HIGH-going vs LOW-going)
  - Scaling to match your example:
      CH1 -> [-100..+100]
      CH2 -> [-100..+100]
      CH3 -> [0..155]
      CH4 -> [-100..+100]
      CH5 -> [-100..+100]  and switch {0,1}
      CH6 -> [0..255] and %

  Prints RAW(us) and SCALED values every PRINT_PERIOD_MS.
*/

#include <Arduino.h>

// ---------------- CONFIG ----------------
#define USE_PULLUPS 1                       // 1 = INPUT_PULLUP
static unsigned long PRINT_PERIOD_MS = 500;

// Two candidate pin maps (MEGA right-side headers)
static uint8_t PINS_EVEN[6] = {46, 44, 42, 40, 38, 36};
static uint8_t PINS_ODD [6] = {45, 43, 41, 39, 37, 35};

// Active pins (chosen at boot)
static uint8_t CH_PINS[6];

// RC pulse window
static const uint16_t RC_MIN_US = 900;
static const uint16_t RC_MAX_US = 2100;

// Timeouts (µs)
static const unsigned long READ_TIMEOUT_US   = 40000UL;  // normal read
static const unsigned long PROBE_TIMEOUT_US  = 20000UL;  // quick probe/learn

// ---------------- STATE ----------------
volatile uint16_t chRaw[6] = {1500,1500,1500,1500,1500,1500};

// Per-channel polarity: 0=unknown, 1=HIGH pulse, 2=LOW pulse
enum { POL_UNKNOWN=0, POL_HIGH=1, POL_LOW=2 };
uint8_t chPol[6] = {POL_UNKNOWN,POL_UNKNOWN,POL_UNKNOWN,POL_UNKNOWN,POL_UNKNOWN,POL_UNKNOWN};

// ---------------- HELPERS ----------------
static inline bool inRangeRC(uint32_t us) {
  return (us >= RC_MIN_US) && (us <= RC_MAX_US);
}

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// float map (for scaled floats)
static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  float t = (x - in_min) / (in_max - in_min);
  return out_min + t * (out_max - out_min);
}

// integer mapping with rounding and clamping
static inline int map_i_round(uint16_t us, int out_min, int out_max) {
  float f = mapf((float)us, 1000.0f, 2000.0f, (float)out_min, (float)out_max);
  int v = (int)lroundf(f);
  if (v < out_min) v = out_min;
  if (v > out_max) v = out_max;
  return v;
}

// Measure a pulse with a specific polarity; 0 on timeout
static inline uint16_t readPulseUS_Polarity(uint8_t pin, uint8_t polarity, unsigned long timeout_us) {
  uint32_t us = (polarity == POL_HIGH) ? pulseIn(pin, HIGH, timeout_us)
                                       : pulseIn(pin,  LOW,  timeout_us);
  return (uint16_t)us;
}

// Try both polarities; return valid width or 0
static inline uint16_t readPulseUS_Both(uint8_t pin, unsigned long timeout_us) {
  uint16_t h = readPulseUS_Polarity(pin, POL_HIGH, timeout_us);
  if (inRangeRC(h)) return h;
  uint16_t l = readPulseUS_Polarity(pin, POL_LOW, timeout_us);
  if (inRangeRC(l)) return l;
  return 0;
}

// ---------------- AUTO-DETECT PIN ROW ----------------
uint16_t probePinSet(uint8_t pins[6], unsigned long ms_per_pin = 250) {
  for (uint8_t i=0;i<6;i++) {
  #if USE_PULLUPS
    pinMode(pins[i], INPUT_PULLUP);
  #else
    pinMode(pins[i], INPUT);
  #endif
  }
  uint16_t score = 0;
  for (uint8_t i=0;i<6;i++) {
    unsigned long t0 = millis();
    uint16_t cnt = 0;
    while (millis() - t0 < ms_per_pin) {
      if (readPulseUS_Both(pins[i], PROBE_TIMEOUT_US)) cnt++;
      delay(2);
    }
    score += (cnt > 0);
  }
  return score; // 0..6
}

void selectActivePins() {
  Serial.println(F("Auto-detecting pin row..."));
  uint16_t score_even = probePinSet(PINS_EVEN, 200);
  uint16_t score_odd  = probePinSet(PINS_ODD,  200);

  const char* chosen = "EVEN";
  uint8_t* src = PINS_EVEN;
  if (score_odd > score_even) { chosen = "ODD"; src = PINS_ODD; }

  for (uint8_t i=0;i<6;i++) CH_PINS[i] = src[i];

  // Set final modes for the chosen set
  for (uint8_t i=0;i<6;i++) {
  #if USE_PULLUPS
    pinMode(CH_PINS[i], INPUT_PULLUP);
  #else
    pinMode(CH_PINS[i], INPUT);
  #endif
  }

  Serial.print(F("Selected pin row: "));
  Serial.print(chosen);
  Serial.print(F("  (even=")); Serial.print(score_even);
  Serial.print(F(", odd="));   Serial.print(score_odd);
  Serial.println(F(")"));

  Serial.print(F("Pins: "));
  for (uint8_t i=0;i<6;i++){ Serial.print("D"); Serial.print(CH_PINS[i]); if (i<5) Serial.print(", "); }
  Serial.println();
}

// ---------------- POLARITY LEARN ----------------
uint8_t learnPolarityForPin(uint8_t pin, uint16_t samples = 24) {
  uint16_t high_ok = 0, low_ok = 0;
  for (uint16_t k=0;k<samples;k++){
    uint16_t h = readPulseUS_Polarity(pin, POL_HIGH, PROBE_TIMEOUT_US);
    if (inRangeRC(h)) high_ok++;
    uint16_t l = readPulseUS_Polarity(pin, POL_LOW,  PROBE_TIMEOUT_US);
    if (inRangeRC(l)) low_ok++;
    delay(2);
  }
  if (high_ok == 0 && low_ok == 0) return POL_UNKNOWN;
  return (high_ok >= low_ok) ? POL_HIGH : POL_LOW;
}

void learnAllPolarities() {
  Serial.println(F("Learning channel polarities..."));
  for (uint8_t i=0;i<6;i++) {
    chPol[i] = learnPolarityForPin(CH_PINS[i]);
    Serial.print(F("  CH")); Serial.print(i+1);
    Serial.print(F(" (D")); Serial.print(CH_PINS[i]); Serial.print(F(") -> "));
    if (chPol[i]==POL_HIGH) Serial.println(F("HIGH pulses"));
    else if (chPol[i]==POL_LOW) Serial.println(F("LOW pulses"));
    else Serial.println(F("UNKNOWN (no valid pulses)"));
  }
}

// ---------------- READ LOOP ----------------
void readChannels() {
  for (uint8_t i=0;i<6;i++) {
    uint16_t us = 0;

    if (chPol[i] == POL_HIGH || chPol[i] == POL_LOW) {
      us = readPulseUS_Polarity(CH_PINS[i], chPol[i], READ_TIMEOUT_US);
      if (!inRangeRC(us)) {
        // quick re-learn if this channel looks bad
        uint8_t newPol = learnPolarityForPin(CH_PINS[i], 8);
        if (newPol != POL_UNKNOWN && newPol != chPol[i]) {
          chPol[i] = newPol;
          us = readPulseUS_Polarity(CH_PINS[i], chPol[i], READ_TIMEOUT_US);
        }
      }
    } else {
      us = readPulseUS_Both(CH_PINS[i], READ_TIMEOUT_US);
      if (inRangeRC(us)) {
        // Lock whichever worked
        uint8_t pol = POL_HIGH;
        if (!inRangeRC(readPulseUS_Polarity(CH_PINS[i], POL_HIGH, PROBE_TIMEOUT_US))) pol = POL_LOW;
        chPol[i] = pol;
      }
    }

    if (inRangeRC(us)) chRaw[i] = us;
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  selectActivePins();
  learnAllPolarities();

  Serial.print(F("Print period (ms): "));
  Serial.println(PRINT_PERIOD_MS);
  Serial.println(F("Scaling: CH1/2/4 -> [-100..100], CH3 -> [0..155], CH5 -> [-100..100]+switch, CH6 -> [0..255]+%"));
  Serial.println();
}

// ---------------- MAIN LOOP ----------------
void loop() {
  readChannels();

  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = now;

    // RAW snapshot
    uint16_t c1 = chRaw[0];
    uint16_t c2 = chRaw[1];
    uint16_t c3 = chRaw[2];
    uint16_t c4 = chRaw[3];
    uint16_t c5 = chRaw[4];
    uint16_t c6 = chRaw[5];

    // === SCALING (like your fsi6x example) ===
    // CH1: [-100..+100] steering
    int rcCH1 = map_i_round(c1, -100, 100);
    // CH2: [-100..+100] throttle (sign = dir)
    int rcCH2 = map_i_round(c2, -100, 100);
    // CH3: [0..155] acceleration
    int rcCH3 = map_i_round(c3, 0, 155);
    // CH4: [-100..+100] (unused/mode)
    int rcCH4 = map_i_round(c4, -100, 100);
    // CH5: both signed and switch
    int     rcCH5_signed = map_i_round(c5, -100, 100);
    uint8_t rcCH5_sw     = (c5 >= 1500) ? 1 : 0;
    // CH6: pot -> 0..255 and %
    int     rcCH6_255 = map_i_round(c6, 0, 255);
    float   rcCH6_pct = (rcCH6_255 / 255.0f) * 100.0f;

    // Print
    Serial.print(F("[RAW(us)] "));
    Serial.print(c1); Serial.print(' ');
    Serial.print(c2); Serial.print(' ');
    Serial.print(c3); Serial.print(' ');
    Serial.print(c4); Serial.print(' ');
    Serial.print(c5); Serial.print(' ');
    Serial.print(c6);

    Serial.print(F("  | [SCALED] "));
    Serial.print(F("CH1="));  Serial.print(rcCH1);
    Serial.print(F(" CH2="));  Serial.print(rcCH2);
    Serial.print(F(" CH3="));  Serial.print(rcCH3);
    Serial.print(F(" CH4="));  Serial.print(rcCH4);
    Serial.print(F(" CH5="));  Serial.print(rcCH5_signed);
    Serial.print(F(" SW5="));  Serial.print(rcCH5_sw);
    Serial.print(F(" CH6="));  Serial.print(rcCH6_255);
    Serial.print(F(" ("));      Serial.print(rcCH6_pct,1); Serial.print('%'); Serial.print(')');
    Serial.println();
  }
}
