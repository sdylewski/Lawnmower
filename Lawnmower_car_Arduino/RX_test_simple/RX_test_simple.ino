/*
  RC Input (PWM only) — MEGA
  New scaling:
    - CH3 (base throttle):   0.0 .. +0.5
    - CH2 (add throttle):   -0.5 .. +0.5
    -> Combined throttle:   -0.5 .. +1.0
       (reverse capped at -0.5 of max; forward up to +1.0)

  Prints every PRINT_PERIOD_MS:
    RAW(us): ch1..ch6
    SCALED:  CH1_LR, CH3_TH_BASE, CH2_TH_ADD, TH_COMB, PWM_COMB, CH5_SW, CH6_POT
*/

#include <Arduino.h>

// ------------ CONFIG ------------
unsigned long PRINT_PERIOD_MS = 500;

// Use pullups only if your RX outputs are open-drain / long leads.
#define USE_PULLUPS 0

// MEGA pins for the 6 PWM channels
const uint8_t CH_PINS[6] = {46, 44, 42, 40, 38, 36};

// ------------ STATE ------------
volatile uint16_t chRaw[6] = {1500,1500,1500,1500,1000,1000}; // sane defaults

// ------------ HELPERS ------------
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  float t = (x - in_min) / (in_max - in_min);
  return out_min + t * (out_max - out_min);
}

// Wait while line is HIGH (avoid half-pulse), then measure next HIGH pulse.
static inline uint16_t readPulseUS_waitIfHigh(uint8_t pin, unsigned long timeout_us = 30000UL) {
  unsigned long t0 = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - t0 >= timeout_us) return 0; // timed out waiting to go LOW
    delayMicroseconds(100);
  }
  unsigned long us = pulseIn(pin, HIGH, timeout_us);
  if (us == 0) return 0;
  if (us < 900)  us = 900;
  if (us > 2100) us = 2100;
  return (uint16_t)us;
}

// ------------ READER ------------
void readChannelsPWM() {
  for (uint8_t i = 0; i < 6; i++) {
    uint16_t us = readPulseUS_waitIfHigh(CH_PINS[i], 30000UL);
    if (us != 0) chRaw[i] = us; // keep last good on timeout
  }
}

// ------------ SETUP / LOOP ------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  for (uint8_t i = 0; i < 6; i++) {
#if USE_PULLUPS
    pinMode(CH_PINS[i], INPUT_PULLUP);
#else
    pinMode(CH_PINS[i], INPUT);
#endif
  }

  Serial.println(F("RC Input: PWM (wait-while-HIGH then pulseIn(HIGH))"));
  Serial.print(F("Pins: "));
  for (uint8_t i = 0; i < 6; i++) { Serial.print("D"); Serial.print(CH_PINS[i]); if (i<5) Serial.print(", "); }
  Serial.println();
  Serial.println(F("Scaling: CH3 base=[0..0.5], CH2 add=[-0.5..0.5], combined=[-0.5..1.0]"));
  Serial.println();
}

void loop() {
  readChannelsPWM();

  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = now;

    // Snapshot raw microseconds
    uint16_t c1 = chRaw[0];
    uint16_t c2 = chRaw[1];
    uint16_t c3 = chRaw[2];
    uint16_t c4 = chRaw[3];
    uint16_t c5 = chRaw[4];
    uint16_t c6 = chRaw[5];

    // Scaled channels
    float ch1_LR      = clampf(mapf(c1, 1000.0f, 2000.0f, -1.0f,  1.0f), -1.0f, 1.0f); // steering
    float ch3_th_base = clampf(mapf(c3, 1000.0f, 2000.0f,  0.0f,  0.5f),  0.0f, 0.5f);  // base throttle
    float ch2_th_add  = clampf(mapf(c2, 1000.0f, 2000.0f, -0.5f, +0.5f), -0.5f, 0.5f); // add throttle

    // Combined throttle: -0.5 .. +1.0 (reverse limited to 50% of max, forward up to 100%)
    float th_comb     = clampf(ch3_th_base + ch2_th_add, -0.5f, 1.0f);

    // Suggested signed PWM (-127..+255) simply by scaling with 255
    int pwm_comb = (int)lround(th_comb * 255.0f);
    if (pwm_comb < -128) pwm_comb = -128; // clamp just in case
    if (pwm_comb >  255) pwm_comb =  255;

    uint8_t ch5_sw       = (c5 >= 1500) ? 1 : 0;                                  // switch
    int     ch6_pot_255  = constrain(map((int)c6, 1000, 2000, 0, 255), 0, 255);   // 0..255
    float   ch6_pot_pct  = (ch6_pot_255 / 255.0f) * 100.0f;

    // RAW line
    Serial.print(F("[SRC=PWM] RAW(us): "));
    Serial.print(c1); Serial.print(' ');
    Serial.print(c2); Serial.print(' ');
    Serial.print(c3); Serial.print(' ');
    Serial.print(c4); Serial.print(' ');
    Serial.print(c5); Serial.print(' ');
    Serial.print(c6); Serial.print(' ');

    // SCALED + combined throttle and suggested PWM
    Serial.print(F(" | SCALED: "));
    Serial.print(F("CH1_LR="));       Serial.print(ch1_LR, 3);
    Serial.print(F(", CH3_TH_BASE="));Serial.print(ch3_th_base, 3);
    Serial.print(F(", CH2_TH_ADD=")); Serial.print(ch2_th_add, 3);
    Serial.print(F(", TH_COMB="));    Serial.print(th_comb, 3);
    Serial.print(F(", PWM_COMB="));   Serial.print(pwm_comb);
    Serial.print(F(", CH5_SW="));     Serial.print(ch5_sw);
    Serial.print(F(", CH6_POT="));    Serial.print(ch6_pot_255);
    Serial.print(F(" ("));            Serial.print(ch6_pot_pct, 1); Serial.print(F("%)"));
    Serial.println();
  }
}
