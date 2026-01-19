/*
  PID_Calibrate_4Wheels (Arduino MEGA) — CLEAN + DIAGNOSTIC
  - Per-wheel relay autotune (Åström–Hägglund) → Ku, Tu → ZN PID gains
  - Encoders: x1 (A on interrupt-capable pin; B any digital)
  - Motor drivers: IN1/IN2 + PWM per wheel
  - Kill switch on D52 (INPUT_PULLUP). Pull to GND to stop.

  HOW TO USE:
    • Set RUN_WHEEL[] to choose which wheels to tune.
    • Start with SP_RPM modest (e.g. 50–120). Start RELAY_H_PWM around 20–40.
    • Watch serial output for:
        - Bias, (u_low/u_high), Preview (PWM→RPM) table
        - If “Setpoint NOT crossable…”, use the suggested SP & H.
        - Final ZN PID gains to copy-paste.
*/

// ---------- Types must be defined BEFORE use ----------
struct TuneResult {
  bool  ok;
  float Ku;   // ultimate gain
  float Tu;   // ultimate period (s)
  float Kp;   // ZN PID
  float Ki;
  float Kd;
};

// ---------- Includes & small helpers ----------
#include <Arduino.h>
#include <math.h>

// ---------- Tuning rules ----------
enum TuningRule : uint8_t { RULE_ZN_CLASSIC = 0, RULE_TYREUS_LUYBEN = 1 };

// Choose which gains the autotune returns in TuneResult:
#define DEFAULT_TUNING_RULE RULE_TYREUS_LUYBEN   // <--- change to RULE_ZN_CLASSIC if you want

// Compute {Kp,Ki,Kd} from Ku,Tu for a given rule (parallel form: u = Kp*e + Ki*∫e dt + Kd*de/dt)
static inline void pidFromKuTu(float Ku, float Tu, TuningRule rule,
                               float &Kp, float &Ki, float &Kd) {
  switch (rule) {
    case RULE_ZN_CLASSIC: {
      // Ziegler–Nichols (classic PID):  Kp=0.6Ku, Ti=0.5Tu, Td=0.125Tu
      const float Kp0 = 0.6f * Ku;
      const float Ti  = 0.5f * Tu;
      const float Td  = 0.125f * Tu;
      Kp = Kp0;
      Ki = Kp0 / Ti;
      Kd = Kp0 * Td;
      break;
    }
    case RULE_TYREUS_LUYBEN: {
      // Tyreus–Luyben (PID): Kp=0.454Ku, Ti=2.2Tu, Td=0.159Tu
      // (gentler: less overshoot, better disturbance rejection)
      const float Kp0 = 0.454f * Ku;
      const float Ti  = 2.2f   * Tu;
      const float Td  = 0.159f * Tu;
      Kp = Kp0;
      Ki = Kp0 / Ti;
      Kd = Kp0 * Td;
      break;
    }
  }
}

static inline const __FlashStringHelper* ruleName(TuningRule r) {
  return (r == RULE_TYREUS_LUYBEN) ? F("Tyreus–Luyben") : F("Ziegler–Nichols");
}


static inline uint8_t u8min(uint8_t a, uint8_t b) { return (a < b) ? a : b; }
static inline uint8_t u8max(uint8_t a, uint8_t b) { return (a > b) ? a : b; }

// ---------- Verbosity toggles ----------
#define VERBOSE_REACH  1   // prints preview (PWM→RPM) and reachability
#define VERBOSE_RELAY  1   // prints live amplitude/period once detected
#define RELAY_DEBUG   1     // periodic rpm/set/u during relay run
#define USE_ABS_RPM   1     // tune on speed magnitude (ignores sign)

// ---------- USER CONFIG ----------
static const uint8_t ENABLE_PIN      = 52;     // Kill switch (INPUT_PULLUP). LOW=STOP.
static const float   SP_RPM          = 60.0f;  // Target RPM for tuning (per wheel)
static const uint8_t RELAY_H_PWM     = 0;     // Relay half-span (PWM counts); pass 0 to auto-pick
static const uint16_t BIAS_SETTLE_MS = 400;    // Settle during bias sweep
static const uint16_t BIAS_MEAS_MS   = 300;    // Measure window during bias sweep
static const uint16_t RELAY_MAX_MS   = 40000;  // Max relay run duration
static const float   HYST_RPM        = 1.5f;   // Hysteresis on error around setpoint
static const uint8_t MIN_BIAS_PWM    = 45;     // Avoid deadzone
static const uint8_t MAX_BIAS_PWM    = 230;    // Leave headroom for ±H

// Which wheels to tune? Order: {FL, FR, BL, BR}
bool RUN_WHEEL[4] = { true, true, true, true };

// Motor/encoder polarity (match your robot)
int8_t MOTOR_SIGN[4] = { -1, -1, -1, -1 };   // Flip if “forward” runs backward
int8_t ENC_SIGN[4]   = { -1, +1, -1, +1 };   // Flip if measured RPM sign is wrong

// Mechanics (x1 counting on channel A)
static const float CPR_MOTOR  = 16.0f;
static const float GEAR_RATIO = 150.0f;
static const float PPR        = CPR_MOTOR * GEAR_RATIO;   // pulses per rev (single channel)

// ---------- PINS (MEGA) — EDIT AS NEEDED ----------
// Encoders: A must be interrupt-capable {2,3,18,19,20,21}; B can be any digital.
const uint8_t ENC_A_PINS[4] = { 19, 18, 21, 20 };  // FL, FR, BL, BR
const uint8_t ENC_B_PINS[4] = { 56, 58, 64, 66 };  // FL, FR, BL, BR

// Motor driver pins per wheel: IN1/IN2 + PWM (PWM pins: 2..13, 44..46 on MEGA)
const uint8_t PWM_PINS[4]   = { 8, 2, 9, 10 };     // FL, FR, BL, BR
const uint8_t IN1_PINS[4]   = { 6, 4, 16, 22 };    // FL, FR, BL, BR
const uint8_t IN2_PINS[4]   = { 7, 3, 15, 23 };    // FL, FR, BL, BR

// Optional TB6612 STBY/EN pins per wheel (255 if tied HIGH or not present)
const uint8_t STBY_PINS[4]  = { 5, 5, 17, 17 };

// ---------- STATE ----------
volatile long ENC_CNT[4] = {0,0,0,0};

inline bool  killActive() { return digitalRead(ENABLE_PIN) == LOW; }
inline float rpmFromCounts(long dcounts, float dt_s) {
  const float counts_per_rev = PPR;     // x1 on A
  float rps = (dcounts / counts_per_rev) / dt_s;
  return rps * 60.0f;
}

// ---------- Encoder ISRs ----------
static inline void onAEdge(uint8_t i) {
  int8_t step = digitalRead(ENC_B_PINS[i]) ? -1 : +1; // B=1 one dir, B=0 other
  ENC_CNT[i] += ENC_SIGN[i] * step;
}
void isrA0(){ onAEdge(0); } // FL
void isrA1(){ onAEdge(1); } // FR
void isrA2(){ onAEdge(2); } // BL
void isrA3(){ onAEdge(3); } // BR

// ---------- Motor control ----------
void enableAllDrivers() {
  for (uint8_t w=0; w<4; w++) {
    if (STBY_PINS[w] != 255) {
      pinMode(STBY_PINS[w], OUTPUT);
      digitalWrite(STBY_PINS[w], HIGH);
    }
  }
}
void motorSetPWM(uint8_t w, int pwmSigned) {
  int cmd = pwmSigned * MOTOR_SIGN[w];
  bool fwd = (cmd >= 0);
  uint8_t mag = (uint8_t)((abs(cmd) > 255) ? 255 : abs(cmd));
  digitalWrite(IN1_PINS[w], fwd ? HIGH : LOW);
  digitalWrite(IN2_PINS[w], fwd ? LOW  : HIGH);
  analogWrite(PWM_PINS[w], mag);
}
void motorStop(uint8_t w) {
  digitalWrite(IN1_PINS[w], LOW);
  digitalWrite(IN2_PINS[w], LOW);
  analogWrite(PWM_PINS[w], 0);
}
void stopAllMotors(){ for (uint8_t w=0; w<4; w++) motorStop(w); }

// ---------- RPM measurement ----------
float measureRPM(uint8_t w, uint16_t meas_ms) {
  long c0, c1;
  noInterrupts(); c0 = ENC_CNT[w]; interrupts();
  unsigned long t0 = millis();
  while (millis() - t0 < meas_ms) { if (killActive()) return 0.0f; }
  noInterrupts(); c1 = ENC_CNT[w]; interrupts();
  float dt = (millis() - t0) / 1000.0f; if (dt <= 0) dt = meas_ms / 1000.0f;
  return rpmFromCounts(c1 - c0, dt);
}

// Quick sampling at a fixed PWM (settle then measure)
float rpmAt(uint8_t w, uint8_t u, uint16_t settle_ms=300, uint16_t meas_ms=300) {
  motorSetPWM(w, u);
  unsigned long t0 = millis();
  while (millis()-t0 < settle_ms) { if (killActive()) { motorStop(w); return 0; } }
  float rpm = measureRPM(w, meas_ms);
  motorStop(w);
  return rpm;
}

// ---------- Bias PWM finder ----------
uint8_t findBiasPWM(uint8_t w, float set_rpm) {
  uint8_t bestPWM = MIN_BIAS_PWM;
  float bestErr = 1e9;

  for (uint8_t pwm = MIN_BIAS_PWM; pwm <= MAX_BIAS_PWM; pwm += 10) {
    motorSetPWM(w, pwm);
    unsigned long t0 = millis();
    while (millis() - t0 < BIAS_SETTLE_MS) { if (killActive()) { motorStop(w); return 0; } }
    float rpm = measureRPM(w, BIAS_MEAS_MS);
    float e = fabsf(rpm - set_rpm);
    if (e < bestErr) { bestErr = e; bestPWM = pwm; }
  }
  motorStop(w);
  return bestPWM;
}

// Small preview table to pick a sane setpoint & relay span
void previewReach(uint8_t w, uint8_t u0) {
#if VERBOSE_REACH
  Serial.println(F("  Preview (PWM → RPM):"));
  const int8_t offs[] = { -40, -20, -10, 0, 10, 20, 40 };
  const uint8_t N = sizeof(offs)/sizeof(offs[0]);
  for (uint8_t k=0; k<N; ++k) {
    int16_t u = (int16_t)u0 + (int16_t)offs[k];
    if (u < 0) u = 0; if (u > 255) u = 255;
    float rpm = rpmAt(w, (uint8_t)u);
    Serial.print(F("    ")); Serial.print((int)u); Serial.print(F(" → "));
    Serial.println(rpm,1);
  }
#endif
}

// ---------- Relay autotune (Åström–Hägglund) ----------
// If H_pwm == 0, an automatic value (~22% of bias) is chosen and printed.
TuneResult runRelayAutotune(uint8_t w, float set_rpm, uint8_t H_pwm)
{
  TuneResult tr = {false,0,0,0,0,0};

  // --- 1) Bias near setpoint (no feed-forward seed) ---
  float sp = set_rpm;
#if USE_ABS_RPM
  if (sp < 0) sp = -sp;                         // tune on magnitude only
#endif

  Serial.print(F("  Searching bias for wheel ")); Serial.println(w);
  uint8_t u0 = findBiasPWM(w, sp);
  if (u0 == 0) { Serial.println(F("  Bias search aborted (kill?).")); return tr; }

  // --- 2) Choose relay span H and compute u_low/u_high ONCE ---
  uint8_t H = H_pwm;
  if (H == 0) {
    int h = (int)lroundf(0.22f * (float)u0);    // ~22% of bias
    if (h < 10) h = 10;
    if (h > 60) h = 60;
    H = (uint8_t)h;
  }
  if ((int)u0 + (int)H > 250) H = (uint8_t)(250 - u0); // top guard
  if (H > u0)                  H = u0;                  // bottom guard
  if (H < 6)                   H = 6;                   // keep some width

  const uint8_t u_high = (uint8_t)(u0 + H);
  const uint8_t u_low  = (uint8_t)(u0 - H);
  const float   Hc     = 0.5f * ( (int16_t)u_high - (int16_t)u_low ); // control amp (PWM)

  Serial.print(F("  Bias PWM=")); Serial.print(u0);
  Serial.print(F(", relay levels: ")); Serial.print(u_low);
  Serial.print(F(" / "));              Serial.print(u_high);
  Serial.print(F(" (H="));             Serial.print(Hc,1); Serial.println(F(")"));

  // --- 3) Preview around the bias (PWM → RPM) ---
#if VERBOSE_REACH
  Serial.println(F("  Preview (PWM → RPM):"));
  static const int8_t offs[] = { -40, -20, -10, 0, 10, 20, 40 };
  const uint8_t N_OFFS = (uint8_t)(sizeof(offs)/sizeof(offs[0]));
  for (uint8_t i=0; i<N_OFFS; ++i) {
    int u = (int)u0 + (int)offs[i];
    if (u < 0)   u = 0;
    if (u > 255) u = 255;
    float rpm = rpmAt(w, (uint8_t)u);
#if USE_ABS_RPM
    if (rpm < 0) rpm = -rpm;
#endif
    Serial.print(F("    ")); Serial.print(u); Serial.print(F(" → "));
    Serial.println(rpm,1);
  }
#endif

  // --- 4) Reachability check at u_low / u_high ---
  float rpm_low  = rpmAt(w, u_low);
  float rpm_high = rpmAt(w, u_high);
#if USE_ABS_RPM
  if (rpm_low  < 0) rpm_low  = -rpm_low;
  if (rpm_high < 0) rpm_high = -rpm_high;
#endif
  Serial.print(F("  Reachability: rpm_low="));  Serial.print(rpm_low,1);
  Serial.print(F(" rpm_high="));                 Serial.println(rpm_high,1);

  const float tol = 0.5f;                        // small tolerance
  if (!((rpm_low <= sp + tol) && (rpm_high >= sp - tol))) {
    float sp_suggest = 0.5f * (rpm_low + rpm_high);
    int   h_suggest  = (int)lroundf(0.20f * (float)u0);
    if (h_suggest < 10) h_suggest = 10;
    if (h_suggest > 60) h_suggest = 60;
    Serial.println(F("  Setpoint NOT crossable by (u_low,u_high)."));
    Serial.print  (F("  -> Try SP_RPM ≈ ")); Serial.print(sp_suggest,1);
    Serial.print  (F(", RELAY_H_PWM ≈ "));    Serial.println(h_suggest);
    return tr;                                   // user should adjust SP/H
  }

  // --- 5) Relay run: error-sign switching with hysteresis + stall guard ---
  const uint16_t sample_ms = 50;
  unsigned long t_start = millis();
  bool highState = true;
  motorSetPWM(w, u_high);
  unsigned long lastFlip = millis();

  // Extrema tracking
  const uint8_t MAX_EXT = 12;
  float peaks[MAX_EXT] = {0}, troughs[MAX_EXT] = {0};
  unsigned long t_peaks[MAX_EXT] = {0}, t_troughs[MAX_EXT] = {0};
  uint8_t nPeaks = 0, nTroughs = 0, nFlips = 0;
  float last1=0, last2=0, last3=0;

  while (millis() - t_start < RELAY_MAX_MS) {
    if (killActive()) { motorStop(w); return tr; }

    float rpm = measureRPM(w, sample_ms);
#if USE_ABS_RPM
    if (rpm < 0) rpm = -rpm;
#endif
    float e = sp - rpm;                           // +ve when below setpoint

    // Schmitt trigger on error with hysteresis
    if (highState) {
      if (e <= -HYST_RPM) { highState = false; motorSetPWM(w, u_low);  lastFlip=millis(); nFlips++; }
    } else {
      if (e >=  HYST_RPM) { highState = true;  motorSetPWM(w, u_high); lastFlip=millis(); nFlips++; }
    }

    // Stall guard: force a flip if stuck too long at one level
    if (millis() - lastFlip > 1500) {
      highState = !highState;
      motorSetPWM(w, highState ? u_high : u_low);
      lastFlip = millis();
      nFlips++;
      Serial.println(F("  [forced flip to avoid stall]"));
    }

    // Peak/trough detection (simple 3-sample test)
    last3 = last2; last2 = last1; last1 = rpm;
    if (last2 > last1 && last2 > last3) {
      if (nPeaks < MAX_EXT) { peaks[nPeaks] = last2; t_peaks[nPeaks] = millis(); nPeaks++; }
    }
    if (last2 < last1 && last2 < last3) {
      if (nTroughs < MAX_EXT) { troughs[nTroughs] = last2; t_troughs[nTroughs] = millis(); nTroughs++; }
    }

#if RELAY_DEBUG
    static unsigned long lastDbg = 0;
    if (millis() - lastDbg >= 120) { lastDbg = millis();
      Serial.print(F("rpm=")); Serial.print(rpm,1);
      Serial.print(F(" set=")); Serial.print(sp,1);
      Serial.print(F(" u="));   Serial.println(highState ? u_high : u_low);
    }
#endif

    // Enough flips & extrema? Estimate amplitude and period.
    if (nFlips >= 6 && nPeaks >= 3 && nTroughs >= 3) {
      float p_avg = (peaks[nPeaks-1] + peaks[nPeaks-2] + peaks[nPeaks-3]) / 3.0f;
      float t_avg = (troughs[nTroughs-1] + troughs[nTroughs-2] + troughs[nTroughs-3]) / 3.0f;
      float a = 0.5f * (p_avg - t_avg);          // output amplitude (RPM)

      float Tu_ms = 0;
      if (nPeaks >= 3) {
        unsigned long d1 = t_peaks[nPeaks-1] - t_peaks[nPeaks-2];
        unsigned long d2 = t_peaks[nPeaks-2] - t_peaks[nPeaks-3];
        Tu_ms = 0.5f * (d1 + d2);
      } else {
        unsigned long d1 = t_troughs[nTroughs-1] - t_troughs[nTroughs-2];
        unsigned long d2 = t_troughs[nTroughs-2] - t_troughs[nTroughs-3];
        Tu_ms = 0.5f * (d1 + d2);
      }
      float Tu = Tu_ms / 1000.0f;

      if (a > 0.3f && Tu > 0.06f) {
        #if VERBOSE_REACH
        Serial.print(F("  live a=")); Serial.print(a,2);
        Serial.print(F(" Tu="));      Serial.print(Tu,3);
        Serial.print(F(" Hc="));      Serial.println(Hc,1);
        #endif
        // Describing function for square relay: Ku = (4*Hc)/(pi*a)
        float Ku = (4.0f * Hc) / (PI * a);

        // Compute PID gains per selected rule
        float Kp, Ki, Kd;
        pidFromKuTu(Ku, Tu, (TuningRule)DEFAULT_TUNING_RULE, Kp, Ki, Kd);

        tr.ok = true; tr.Ku = Ku; tr.Tu = Tu; tr.Kp = Kp; tr.Ki = Ki; tr.Kd = Kd;
        motorStop(w);
        return tr;

      }
    }
  }

  motorStop(w);
  Serial.println(F("  Relay autotune timed out; try larger RELAY_H_PWM or lower SP_RPM."));
  return tr;
}

// ---------- Run one wheel ----------
void tuneWheel(uint8_t w) {
  Serial.println();
  const char* names[4] = {"FL","FR","BL","BR"};
  Serial.print(F("=== Autotune wheel ")); Serial.print(names[w]); Serial.println(F(" ==="));

  // Safety: ensure other wheels are off
  for (uint8_t i = 0; i < 4; ++i) { if (i != w) motorStop(i); }

  // Run the relay autotune (this will compute Ku, Tu, and then Kp,Ki,Kd using DEFAULT_TUNING_RULE)
  TuneResult tr = runRelayAutotune(w, SP_RPM, RELAY_H_PWM);
  if (!tr.ok) {
    Serial.println(F("Tune FAILED. Suggestions:"));
    Serial.println(F(" - Increase RELAY_H_PWM (e.g., 30..40)"));
    Serial.println(F(" - Change SP_RPM to a speed where wheel runs cleanly"));
    Serial.println(F(" - Check encoder polarity/signals"));
    return;
  }

  // Report ultimate gain/period
  Serial.print(F("Result: Ku=")); Serial.print(tr.Ku, 4);
  Serial.print(F(", Tu="));       Serial.print(tr.Tu, 4); Serial.println(F(" s"));

  // Print both rule recommendations for comparison
  float kpZN, kiZN, kdZN, kpTL, kiTL, kdTL;
  pidFromKuTu(tr.Ku, tr.Tu, RULE_ZN_CLASSIC,    kpZN, kiZN, kdZN);
  pidFromKuTu(tr.Ku, tr.Tu, RULE_TYREUS_LUYBEN, kpTL, kiTL, kdTL);

  Serial.print(F("PID (Ziegler–Nichols): Kp=")); Serial.print(kpZN, 4);
  Serial.print(F(", Ki="));                      Serial.print(kiZN, 4);
  Serial.print(F(", Kd="));                      Serial.println(kdZN, 4);

  Serial.print(F("PID (Tyreus–Luyben):   Kp=")); Serial.print(kpTL, 4);
  Serial.print(F(", Ki="));                      Serial.print(kiTL, 4);
  Serial.print(F(", Kd="));                      Serial.println(kdTL, 4);

  // Indicate which one is actually returned in TuneResult (per DEFAULT_TUNING_RULE)
  Serial.print(F(">> Returning rule: "));
  Serial.println(ruleName((TuningRule)DEFAULT_TUNING_RULE));

  // Copy-paste line for your main code (uses the gains returned in TuneResult)
  Serial.println(F("\n// Paste into your PID_GAIN[] array (order: FL, FR, BL, BR):"));
  Serial.print(F("// ")); Serial.print(names[w]); Serial.print(F(": { "));
  Serial.print(tr.Kp, 4); Serial.print(F(", "));
  Serial.print(tr.Ki, 4); Serial.print(F(", "));
  Serial.print(tr.Kd, 4); Serial.println(F(" }"));
}


// ---------- SETUP / LOOP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(ENABLE_PIN, INPUT_PULLUP);

  for (uint8_t w=0; w<4; w++) {
    pinMode(ENC_A_PINS[w], INPUT_PULLUP);
    pinMode(ENC_B_PINS[w], INPUT_PULLUP);
    pinMode(IN1_PINS[w], OUTPUT);
    pinMode(IN2_PINS[w], OUTPUT);
    pinMode(PWM_PINS[w], OUTPUT);
  }
  enableAllDrivers();

  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[0]), isrA0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[1]), isrA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[2]), isrA2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[3]), isrA3, RISING);

  Serial.println(F("PID_Calibrate_4Wheels — MEGA"));
  Serial.println(F("Pull D52 to GND to STOP at any time.\n"));
}

void loop() {
  if (killActive()) {
    stopAllMotors();
    static unsigned long t0=0;
    if (millis()-t0 > 500){ t0=millis(); Serial.println(F("[KILL active] Tie D52 HIGH to start.")); }
    delay(20);
    return;
  }

  for (uint8_t w=0; w<4; w++) {
    if (!RUN_WHEEL[w]) continue;
    tuneWheel(w);
    stopAllMotors();
    Serial.println(F("\nWaiting 2s before next wheel..."));
    unsigned long t0 = millis();
    while (millis() - t0 < 2000) { if (killActive()) break; }
  }

  Serial.println(F("\nAll selected wheels processed. Re-run in 5s..."));
  unsigned long t1 = millis();
  while (millis() - t1 < 5000) { if (killActive()) break; }
}
