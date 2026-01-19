/*
  4-Wheel Feed-Forward Sweep (Arduino MEGA)
  - One selectable wheel spins through a PWM sweep; others stay off.
  - Quadrature x1 per wheel: A on an external interrupt pin, B read in ISR.
  - Prints Δpulses, measured RPM, expected RPM, plus cps and fA.

  UPDATE THESE ARRAYS with your actual wiring.
  IMPORTANT (MEGA): A_PINS must be interrupt-capable pins: {2,3,18,19,20,21}.
*/

#include <Arduino.h>

// ---------------- USER CONFIG (mechanics) ----------------
static const float CPR_MOTOR     = 16.0f;     // encoder CPR @ motor shaft
static const float GEAR_RATIO    = 150.0f;    // gearbox ratio
static const float PPR           = CPR_MOTOR * GEAR_RATIO; // pulses/rev on ONE channel (A or B)
static const uint16_t MAX_PWM    = 255;
static const uint16_t PWM_STEP   = 25;
static const uint16_t SETTLE_MS  = 200;       // settle time at each PWM
static const uint16_t MEAS_MS    = 1800;      // measurement window
static const float MAX_WHEEL_RPM = 220.0f;    // for "expected" display

// Per-wheel direction sign (set +1/-1 to normalize forward)
int8_t DIR_SIGN[4] = { +1, +1, +1, +1 }; // {FL, FR, BL, BR}
int8_t ENC_SIGN[4]   = { +1, +1, +1, +1 };  // affects encoder count direction only


// ---------------- SELECT WHICH WHEEL TO RUN ----------------
const char* ACTIVE_WHEEL = "FR";  // "FL", "FR", "BL", or "BR"

// ---------------- PIN MAPS (PLACEHOLDERS — EDIT ME) ----------------
// Encoders: A must be interrupt-capable on MEGA; B can be any digital pin.
const uint8_t ENC_A_PINS[4] = { 19, 18, 21, 20 };     // FL, FR, BL, BR  (INT pins) must be 2, 3, 18, 19, 20, 21
const uint8_t ENC_B_PINS[4] = { 56, 58, 64, 66 };   // FL, FR, BL, BR  (any pins)

// Motor driver pins per wheel (TB6612/L298-style: IN1/IN2 + PWM)
const uint8_t PWM_PINS[4] = { 8, 2, 9, 10 };      // FL, FR, BL, BR  (PWM-capable)
const uint8_t IN1_PINS[4] = { 6, 4, 16, 22 };     // FL, FR, BL, BR
const uint8_t IN2_PINS[4] = { 7, 3, 15, 23 };     // FL, FR, BL, BR

// Optional STBY per wheel (TB6612). Use 255 if tied HIGH or not present.
// If multiple wheels share one STBY pin, repeat the same pin number in each slot.
const uint8_t STBY_PINS[4] = { 5, 5, 17, 17 };

// ---------------- ENCODER STATE ----------------
volatile long ENC_CNT[4] = {0,0,0,0};  // counts (x1 on A rising)
static inline void onAEdge(uint8_t i) {
  int8_t step = digitalRead(ENC_B_PINS[i]) ? -1 : +1; // A rising; B=1 => one way, B=0 => other
  ENC_CNT[i] += ENC_SIGN[i] * step;
}

// ISR wrappers for A pins (fixed to 4 wheels)
void isrA0(){ onAEdge(0); } // FL
void isrA1(){ onAEdge(1); } // FR
void isrA2(){ onAEdge(2); } // BL
void isrA3(){ onAEdge(3); } // BR

// ---------------- HELPERS ----------------
int wheelIndexFromName(const char* s) {
  if (!s) return 0;
  if      (!strcmp(s, "FL")) return 0;
  else if (!strcmp(s, "FR")) return 1;
  else if (!strcmp(s, "BL")) return 2;
  else if (!strcmp(s, "BR")) return 3;
  return 0; // default to FL
}

void motorEnable(uint8_t w, bool en) {
  if (STBY_PINS[w] != 255) digitalWrite(STBY_PINS[w], en ? HIGH : LOW);
}

void motorSetPWM(uint8_t w, int pwmSigned) {
  int cmd = pwmSigned * MOTOR_SIGN[w];
  bool fwd = (cmd >= 0);
  uint8_t mag = (uint8_t)min(abs(cmd), 255);
  digitalWrite(IN1_PINS[w], fwd ? HIGH : LOW);
  digitalWrite(IN2_PINS[w], fwd ? LOW  : HIGH);
  analogWrite(PWM_PINS[w], mag);
}

void motorStop(uint8_t w) {
  digitalWrite(IN1_PINS[w], LOW);
  digitalWrite(IN2_PINS[w], LOW);
  analogWrite(PWM_PINS[w], 0);
}

void stopAllMotors() {
  for (uint8_t w=0; w<4; w++) motorStop(w);
}

float rpmFromCounts(long dcounts, float dt_s) {
  // x1 counting on channel A: counts_per_rev = PPR
  const float counts_per_rev = PPR;
  float rps = (dcounts / counts_per_rev) / dt_s;
  return rps * 60.0f;
}

float fA_Hz_from_RPM(float rpm) { return rpm * (PPR / 60.0f); }
float cps_from_counts(long dcounts, float dt_s) { return dcounts / dt_s; }

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  for (uint8_t w=0; w<4; w++) {
    pinMode(ENC_A_PINS[w], INPUT_PULLUP);
    pinMode(ENC_B_PINS[w], INPUT_PULLUP);

    pinMode(IN1_PINS[w], OUTPUT);
    pinMode(IN2_PINS[w], OUTPUT);
    pinMode(PWM_PINS[w], OUTPUT);

    if (STBY_PINS[w] != 255) { pinMode(STBY_PINS[w], OUTPUT); motorEnable(w, true); }
  }

  // Attach interrupts for the four A channels
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[0]), isrA0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[1]), isrA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[2]), isrA2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[3]), isrA3, RISING);

  Serial.println(F("=== 4-Wheel test (x1 on A, MEGA) ==="));
  Serial.print(F("Active wheel: ")); Serial.println(ACTIVE_WHEEL);
  Serial.println();
  Serial.println(F(" PWM | Δpulses |  RPM_meas |  RPM_exp"));
  Serial.println(F("----------------------------------------------"));
}

// ---------------- LOOP ----------------
void loop() {
  const uint8_t W = (uint8_t)wheelIndexFromName(ACTIVE_WHEEL);

  // Ensure only the active wheel is driven
  stopAllMotors();

  for (uint16_t pwm = 0; pwm <= MAX_PWM; pwm += PWM_STEP) {
    // Command selected wheel
    motorSetPWM(W, (int)pwm);

    // Settle
    unsigned long t0 = millis();
    while (millis() - t0 < SETTLE_MS) {}

    // Start measurement
    long startCount;
    noInterrupts(); startCount = ENC_CNT[W]; interrupts();

    unsigned long t1 = millis();
    while (millis() - t1 < MEAS_MS) {}
    float dt_s = (millis() - t1) / 1000.0f;

    long endCount;
    noInterrupts(); endCount = ENC_CNT[W]; interrupts();

    long dp = endCount - startCount;         // delta counts (x1)
    float rpm_meas = rpmFromCounts(dp, dt_s);
    float rpm_exp  = (pwm / (float)MAX_PWM) * MAX_WHEEL_RPM;

    float cps  = cps_from_counts(dp, dt_s);  // count rate (Hz)
    float fA   = fA_Hz_from_RPM(rpm_meas);   // channel A pulse freq (Hz)

    // Primary line (matches your previous style)
    Serial.print(pwm);              Serial.print(" | ");
    Serial.print(dp);               Serial.print(" | ");
    Serial.print(rpm_meas, 2);      Serial.print(" | ");
    Serial.println(rpm_exp, 2);

    // Diagnostic line
    Serial.print(F("cps≈")); Serial.print(cps, 1); Serial.print(F(" Hz | "));
    Serial.print(F("fA≈"));  Serial.print(fA, 1);  Serial.println(F(" Hz"));

    delay(10); // small pause between rows
  }

  stopAllMotors();
  Serial.println();
  Serial.println(F("Sweep complete for selected wheel. Restarting in 5s..."));
  delay(5000);
}
