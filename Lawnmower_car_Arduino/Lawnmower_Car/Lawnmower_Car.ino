/*
  MEGA 4WD Driver — RC + (optional) PID + Blade (IBT-2)
  - RC input (PWM) using: wait-while-HIGH → pulseIn(HIGH)
  - Channels (FlySky style):
      CH1: steering                → [-1..+1]   (with DEADBAND & smoothing)
      CH3: base throttle           → [ 0..+0.5]
      CH2: additive throttle       → [-0.5..+0.5]
      CH4: spin in place           → [-1..+1]   (only when |combined throttle| is small)
      CH5: blade enable switch     → {0,1}
      CH6: blade speed pot         → [0..255]
  - Combined throttle = CH3 + CH2 → [-0.5..+1.0]  (reverse capped to half-speed)
  - Kill switch: D52 INPUT_PULLUP, LOW=STOP everything
  - Optional PID (Tyreus–Luyben gains). Leave USE_PID=0 to test open-loop.

  QUICK TOGGLES:
    #define USE_PID          0   // 1 to use PID on wheel RPM, 0 for open-loop PWM
    #define USE_FEEDFORWARD  0   // keep 0 (not used now)
*/

#include <Arduino.h>
#include <math.h>

// =================== QUICK TOGGLES ===================
#define USE_PID 1          // 0=open-loop testing; 1=closed-loop with PID
#define USE_FEEDFORWARD 0  // keep 0 for now

// =================== HARDWARE PINS ===================
const uint8_t ENABLE_PIN = 52;  // Kill switch (INPUT_PULLUP). LOW = STOP.

// Encoders: A (interrupt-capable), B (any digital)
const uint8_t ENC_A_PINS[4] = { 19, 18, 21, 20 };  // FL, FR, BL, BR
const uint8_t ENC_B_PINS[4] = { 56, 58, 64, 66 };  // FL, FR, BL, BR

// Motor driver pins (TB6612/L298 style): IN1/IN2 + PWM (MEGA PWM: 2..13, 44..46)
const uint8_t PWM_PINS[4] = { 8, 2, 9, 10 };  // FL, FR, BL, BR
const uint8_t IN1_PINS[4] = { 6, 4, 16, 22 };
const uint8_t IN2_PINS[4] = { 7, 3, 15, 23 };

// Optional STBY/EN (255 if tied high or N/A)
const uint8_t STBY_PINS[4] = { 5, 5, 17, 17 };

// Blade (IBT-2 / BTS7960): separate PWM for FWD (REV omitted per request) + STBY/EN
const uint8_t BLADE_PWM_FWD = 12;   // PWM-capable
//const uint8_t BLADE_PWM_REV = 11; // (not used)
const uint8_t BLADE_STBY    = 13;   // or 255 if not used/tied HIGH

// RC input pins (6 PWM channels on MEGA)
const uint8_t CH_PINS[6] = { 46, 44, 42, 40, 38, 36 };  // CH1..CH6

// =================== MECHANICS / SIGNS ===================
static const float CPR_MOTOR = 16.0f;
static const float GEAR_RATIO = 150.0f;
static const float PPR = CPR_MOTOR * GEAR_RATIO;  // pulses per rev (A channel only)
static const float MAX_WHEEL_RPM = 75.0f;         // tweak to your wheel top-speed
static const float SPIN_MAX_RPM = 65.0f;          // spin-in-place target max RPM

// Motor/encoder polarity
int8_t MOTOR_SIGN[4] = { -1, -1, -1, -1 };  // set so “forward” makes robot go forward
int8_t ENC_SIGN[4] = { -1, +1, -1, +1 };    // set so measured RPM increases forward

// RC polarity (flip stick directions if needed)
int8_t RC_POL_CH1_STEER   = -1;  // +1 normal, -1 invert steering
int8_t RC_POL_CH2_TH_ADD  = +1;  // +1 normal, -1 invert add-throttle
int8_t RC_POL_CH3_TH_BASE = +1;  // usually +1 (0..0.5)
int8_t RC_POL_CH4_SPIN    = +1;  // +1 normal, -1 invert spin

// =================== CONTROL LOOP TIMING ===================
static const uint16_t CTRL_PERIOD_MS = 50;  // ~20 Hz main loop

// =================== RC SCALING / FILTERS ===================
#define USE_PULLUPS 0  // RX inputs; set 1 if you need internal pull-ups

// Deadbands
static const float DB_STEER   = 0.10f;
static const float DB_TH_BASE = 0.03f;  // unipolar (0..0.5)
static const float DB_TH_ADD  = 0.05f;
static const float DB_SPIN    = 0.05f;

// Smoothing alphas (0..1): higher = snappier
static const float ALPHA_STEER   = 0.25f;
static const float ALPHA_TH_BASE = 0.25f;
static const float ALPHA_TH_ADD  = 0.25f;
static const float ALPHA_SPIN    = 0.25f;

// Spin is active only when |combined throttle| < SPIN_THRESH
static const float SPIN_THRESH = 0.10f;

// ---- RC trims & optional auto-centering at boot ----
static float STEER_TRIM  = 0.0f;
static float TH_ADD_TRIM = 0.0f;
static float SPIN_TRIM   = 0.0f;
#define AUTO_CENTER_ON_BOOT 1

// 32-bit accumulators to avoid overflow
static uint32_t _acc1 = 0, _acc2 = 0, _acc4 = 0;
static uint16_t _nacc = 0;

// ---------- Traction helpers ----------
static const uint8_t DZ_BIAS_PWM = 18;  // minimum PWM whenever |SP|>0 to beat stiction
static const float STALL_RPM = 2.0f;    // if |RPM| below this...
static const uint16_t STALL_MS = 250;   // ...for this long → do a kick
static const uint16_t KICK_MS = 150;    // kick duration
static const uint8_t KICK_PWM = 200;    // kick strength (applied in SP direction)

// Slip limiting (per side: FL+BL, FR+BR)
static const float SLIP_RPM = 10.0f;       // if fast−slow RPM > this → slip detected
static const uint8_t SLIP_CAP_PWM = 120;   // cap fast wheel PWM to this (abs)
static const uint8_t SLOW_BOOST_PWM = 30;  // add this much PWM to the slow wheel

#define TRACTION_DEBUG 1  // set to 0 to remove all slip/kick prints

#if TRACTION_DEBUG
static unsigned long lastKickLogMs[4] = { 0, 0, 0, 0 };  // per wheel
static unsigned long lastSlipLogMs[2] = { 0, 0 };        // per side: 0=L, 1=R
#endif

// =================== PID GAINS (Tyreus–Luyben) ===================
struct PIDG { float Kp, Ki, Kd; };
PIDG PID_GAIN[4] = {
  /* FL */ { 3.8537f, 8.7148f, 0.1232f },
  /* FR */ { 3.4195f, 7.7328f, 0.1093f },
  /* BL */ { 3.4683f, 7.8433f, 0.1108f },
  /* BR */ { 3.4195f, 7.7328f, 0.1093f },
};

// =================== ZERO/IDLE BEHAVIOR ===================
static const float  SP_ZERO_RPM      = 1.0f;  // |SP| below → treat as zero
static const uint8_t PWM_IDLE_CUTOFF = 14;    // |PWM| < cutoff → 0 (kill creep)
static const float  I_LEAK_NEAR_ZERO = 0.85f; // bleed integrator when near zero

// =================== STATE ===================
volatile long ENC_CNT[4] = { 0, 0, 0, 0 };
long prevCnt[4] = { 0, 0, 0, 0 };
unsigned long lastRPMus = 0;
float RPM[4] = { 0, 0, 0, 0 };

float Iterm[4] = { 0, 0, 0, 0 };
float lastErr[4] = { 0, 0, 0, 0 };

// Anti-stall state
unsigned long kick_until_ms[4] = { 0, 0, 0, 0 };
uint16_t stall_accum_ms[4] = { 0, 0, 0, 0 };

// RC raw microseconds (defaults chosen so throttle base=0 at boot)
volatile uint16_t chRaw[6] = { 1500, 1500, 1000, 1500, 1000, 1000 };

// RC filtered/scaled
float   ch1_steer = 0, ch2_th_add = 0, ch3_th_base = 0, ch4_spin = 0;
uint8_t ch5_blade = 0;
int     ch6_blade_pwm = 0;

// =================== UTILS ===================
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo; if (x > hi) return hi; return x;
}
static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  float t = (x - in_min) / (in_max - in_min);
  return out_min + t * (out_max - out_min);
}
static inline bool killActive() { return digitalRead(ENABLE_PIN) == LOW; }

// =================== ENCODER ISRs ===================
static inline void onAEdge(uint8_t i) {
  int8_t step = digitalRead(ENC_B_PINS[i]) ? -1 : +1;
  ENC_CNT[i] += ENC_SIGN[i] * step;
}
void isrA0(){ onAEdge(0); }
void isrA1(){ onAEdge(1); }
void isrA2(){ onAEdge(2); }
void isrA3(){ onAEdge(3); }

// =================== MOTOR LOW-LEVEL ===================
void enableAllDrivers() {
  for (uint8_t w = 0; w < 4; w++)
    if (STBY_PINS[w] != 255) { pinMode(STBY_PINS[w], OUTPUT); digitalWrite(STBY_PINS[w], HIGH); }
  if (BLADE_STBY != 255) { pinMode(BLADE_STBY, OUTPUT); digitalWrite(BLADE_STBY, HIGH); }
}
void motorSetPWM(uint8_t w, int pwmSigned) {
  int cmd = pwmSigned * MOTOR_SIGN[w];
  bool fwd = (cmd >= 0);
  uint8_t mag = (uint8_t)constrain(abs(cmd), 0, 255);
  digitalWrite(IN1_PINS[w], fwd ? HIGH : LOW);
  digitalWrite(IN2_PINS[w], fwd ? LOW  : HIGH);
  analogWrite(PWM_PINS[w], mag);
}
void motorStop(uint8_t w) {
  digitalWrite(IN1_PINS[w], LOW);
  digitalWrite(IN2_PINS[w], LOW);
  analogWrite(PWM_PINS[w], 0);
}
void stopAllMotors() { for (uint8_t w = 0; w < 4; w++) motorStop(w); }

void bladeSetPWM(int pwm, bool forward = true){
  pwm = constrain(pwm, 0, 255);
  if (BLADE_STBY != 255) digitalWrite(BLADE_STBY, HIGH);
  if (forward) {
    analogWrite(BLADE_PWM_FWD, pwm);
    //analogWrite(BLADE_PWM_REV, 0);
  } else {
    analogWrite(BLADE_PWM_FWD, 0);
    //analogWrite(BLADE_PWM_REV, pwm);
  }
}
void bladeStop(){
  analogWrite(BLADE_PWM_FWD, 0);
  //analogWrite(BLADE_PWM_REV, 0);
}

static inline int sgnf(float x){ return (x > 0) - (x < 0); }

// =================== RPM UPDATE ===================
void updateRPM() {
  unsigned long now = micros();
  if (lastRPMus == 0) { lastRPMus = now; return; }
  float dt = (now - lastRPMus) / 1e6f;
  if (dt <= 0.0f) return;

  for (uint8_t w = 0; w < 4; w++) {
    long c; noInterrupts(); c = ENC_CNT[w]; interrupts();
    long d = c - prevCnt[w];
    prevCnt[w] = c;
    float rps = (d / PPR) / dt;
    float rpm = rps * 60.0f;
    RPM[w] = 0.6f*RPM[w] + 0.4f*rpm;
  }
  lastRPMus = now;
}

// =================== RC READER ===================
static inline uint16_t readPulseUS_waitIfHigh(uint8_t pin, unsigned long timeout_us = 30000UL) {
  unsigned long t0 = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - t0 >= timeout_us) return 0;
    delayMicroseconds(100);
  }
  unsigned long us = pulseIn(pin, HIGH, timeout_us);
  if (us == 0) return 0;
  if (us < 900)  us = 900;
  if (us > 2100) us = 2100;
  return (uint16_t)us;
}
void readChannelsPWM() {
  for (uint8_t i = 0; i < 6; i++) {
    uint16_t us = readPulseUS_waitIfHigh(CH_PINS[i], 30000UL);
    if (us != 0) chRaw[i] = us;
  }
}

// =================== PID ===================
int wheelPID(uint8_t w, float sp_rpm, float fb_rpm, float dt) {
#if USE_PID
  const float Kp = PID_GAIN[w].Kp;
  const float Ki = PID_GAIN[w].Ki;
  const float Kd = PID_GAIN[w].Kd;

  if (fabs(sp_rpm) < SP_ZERO_RPM) Iterm[w] *= I_LEAK_NEAR_ZERO;

  float e = sp_rpm - fb_rpm;
  Iterm[w] += Ki * e * dt;
  Iterm[w] = clampf(Iterm[w], -255.0f, 255.0f);
  float dterm = (e - lastErr[w]) / dt;
  lastErr[w] = e;
  float u = Kp * e + Iterm[w] + Kd * dterm;

  int pwm = (int)lround(clampf(u, -255.0f, 255.0f));
  if (abs(pwm) < PWM_IDLE_CUTOFF) pwm = 0;
  return pwm;
#else
  (void)w; (void)sp_rpm; (void)fb_rpm; (void)dt;
  return 0;
#endif
}

// =================== RC AUTO-CENTER ===================
void calibrateRCNeutral() {
  _acc1 = _acc2 = _acc4 = _nacc = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 1200) {
    readChannelsPWM();
    _acc1 += chRaw[0];
    _acc2 += chRaw[1];
    _acc4 += chRaw[3];
    _nacc++;
    delay(10);
  }
  if (_nacc == 0) return;
  float avg1 = _acc1 / (float)_nacc;
  float avg2 = _acc2 / (float)_nacc;
  float avg4 = _acc4 / (float)_nacc;

  STEER_TRIM  = clampf((avg1 - 1500.0f) / 500.0f, -0.30f, 0.30f);
  TH_ADD_TRIM = clampf(((avg2 - 1500.0f) / 1000.0f) * 0.5f, -0.25f, 0.25f);
  SPIN_TRIM   = clampf((avg4 - 1500.0f) / 500.0f, -0.30f, 0.30f);

  Serial.print(F("Auto-center trims: steer=")); Serial.print(STEER_TRIM, 3);
  Serial.print(F(" th_add="));                  Serial.print(TH_ADD_TRIM, 3);
  Serial.print(F(" spin="));                    Serial.println(SPIN_TRIM, 3);
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(ENABLE_PIN, INPUT_PULLUP);

  for (uint8_t i = 0; i < 6; i++) {
#if USE_PULLUPS
    pinMode(CH_PINS[i], INPUT_PULLUP);
#else
    pinMode(CH_PINS[i], INPUT);
#endif
  }

#if AUTO_CENTER_ON_BOOT
  Serial.println(F("Auto-centering RC… keep sticks centered for ~1s"));
  calibrateRCNeutral();
#endif

  for (uint8_t w = 0; w < 4; w++) {
    pinMode(ENC_A_PINS[w], INPUT_PULLUP);
    pinMode(ENC_B_PINS[w], INPUT_PULLUP);
    pinMode(IN1_PINS[w], OUTPUT);
    pinMode(IN2_PINS[w], OUTPUT);
    pinMode(PWM_PINS[w], OUTPUT);
  }
  pinMode(BLADE_PWM_FWD, OUTPUT);
  //pinMode(BLADE_PWM_REV, OUTPUT);
  if (BLADE_STBY != 255) pinMode(BLADE_STBY, OUTPUT);

  enableAllDrivers();

  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[0]), isrA0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[1]), isrA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[2]), isrA2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[3]), isrA3, RISING);

  Serial.println(F("MEGA 4WD Driver — RC + (optional) PID + Blade (IBT-2)"));
  Serial.print(F("Modes: USE_PID="));         Serial.print((int)USE_PID);
  Serial.print(F(", USE_FEEDFORWARD="));      Serial.println((int)USE_FEEDFORWARD);
  Serial.println();
}

// =================== MAIN LOOP ===================
void loop() {
  if (killActive()) {
    stopAllMotors();
    bladeStop();
    static unsigned long t0 = 0;
    if (millis() - t0 > 500) { t0 = millis(); Serial.println(F("[KILL] D52 LOW — holding.")); }
    delay(10);
    return;
  }

  // 1) Read RC
  readChannelsPWM();

  // 2) Scale + polarity + smoothing + trims + deadbands
  float raw_steer   = clampf(mapf(chRaw[0], 1000, 2000, -1.0f, 1.0f), -1.0f, 1.0f) * RC_POL_CH1_STEER;
  static float ch1f = 0; ch1f = (1 - ALPHA_STEER) * ch1f + ALPHA_STEER * raw_steer; ch1f -= STEER_TRIM;
  if (fabs(ch1f) < DB_STEER) ch1f = 0.0f; ch1_steer = clampf(ch1f, -1.0f, 1.0f);

  float raw_th_add  = clampf(mapf(chRaw[1], 1000, 2000, -0.5f, +0.5f), -0.5f, 0.5f) * RC_POL_CH2_TH_ADD;
  static float ch2f = 0; ch2f = (1 - ALPHA_TH_ADD) * ch2f + ALPHA_TH_ADD * raw_th_add; ch2f -= TH_ADD_TRIM;
  if (fabs(ch2f) < DB_TH_ADD) ch2f = 0.0f; ch2_th_add = clampf(ch2f, -0.5f, 0.5f);

  float raw_th_base = clampf(mapf(chRaw[2], 1000, 2000, 0.0f, 0.5f), 0.0f, 0.5f) * RC_POL_CH3_TH_BASE;
  raw_th_base = clampf(raw_th_base, 0.0f, 0.5f);
  static float ch3f = 0; ch3f = (1 - ALPHA_TH_BASE) * ch3f + ALPHA_TH_BASE * raw_th_base;
  if (ch3f < DB_TH_BASE) ch3f = 0.0f; ch3_th_base = clampf(ch3f, 0.0f, 0.5f);

  float raw_spin    = clampf(mapf(chRaw[3], 1000, 2000, -1.0f, 1.0f), -1.0f, 1.0f) * RC_POL_CH4_SPIN;
  static float ch4f = 0; ch4f = (1 - ALPHA_SPIN) * ch4f + ALPHA_SPIN * raw_spin; ch4f -= SPIN_TRIM;
  if (fabs(ch4f) < DB_SPIN) ch4f = 0.0f; ch4_spin = clampf(ch4f, -1.0f, 1.0f);

  ch5_blade     = (chRaw[4] >= 1500) ? 1 : 0;
  ch6_blade_pwm = constrain(map((int)chRaw[5], 1000, 2000, 0, 255), 0, 255);

  // 3) Combine throttle and create RPM setpoints
  float th_comb = clampf(ch3_th_base + ch2_th_add, -0.5f, 1.0f);
  float sp_left = 0.0f, sp_right = 0.0f;

  if (fabs(th_comb) < SPIN_THRESH && ch4_spin != 0.0f) {
    sp_left  =  ch4_spin * SPIN_MAX_RPM;
    sp_right = -ch4_spin * SPIN_MAX_RPM;
  } else {
    float baseRPM  = th_comb   * MAX_WHEEL_RPM;
    float steerRPM = ch1_steer * (0.5f * MAX_WHEEL_RPM);
    sp_left  = clampf(baseRPM - steerRPM, -MAX_WHEEL_RPM, MAX_WHEEL_RPM);
    sp_right = clampf(baseRPM + steerRPM, -MAX_WHEEL_RPM, MAX_WHEEL_RPM);
  }

  // 4) Update measured RPMs
  updateRPM();

  // 4.5) Near-zero handling for wheels only (blade is independent)
  bool nearZero = (fabs(sp_left) < SP_ZERO_RPM) && (fabs(sp_right) < SP_ZERO_RPM) && (ch4_spin == 0.0f);
  if (nearZero) {
    for (uint8_t w = 0; w < 4; w++) { Iterm[w] = 0; lastErr[w] = 0; }
    stopAllMotors();
  }

  // 5) Command wheels — PER-WHEEL control + anti-stall + slip limit
  float dt = CTRL_PERIOD_MS / 1000.0f;

  // Per-wheel setpoints: left side (0,2), right side (1,3)
  float SET_RPM[4] = { sp_left, sp_right, sp_left, sp_right };
  int   U[4]       = { 0, 0, 0, 0 };

  // Compute raw control per wheel (PID or open-loop)
  for (uint8_t w = 0; w < 4; w++) {
#if USE_PID
    U[w] = wheelPID(w, SET_RPM[w], RPM[w], dt);  // signed PWM
#else
    float m = clampf(SET_RPM[w] / MAX_WHEEL_RPM, -1.0f, 1.0f);
    U[w] = constrain((int)lround(m * 255.0f), -255, 255);
#endif
  }

  // Anti-stall: detect stall and schedule kick
  for (uint8_t w = 0; w < 4; w++) {
    if (fabs(SET_RPM[w]) >= SP_ZERO_RPM) {
      if (fabs(RPM[w]) < STALL_RPM) {
        stall_accum_ms[w] += CTRL_PERIOD_MS;
        if (stall_accum_ms[w] >= STALL_MS) {
          kick_until_ms[w] = millis() + KICK_MS;  // schedule kick
          stall_accum_ms[w] = 0;

#if TRACTION_DEBUG
          if (millis() - lastKickLogMs[w] > 250) {  // rate-limit
            lastKickLogMs[w] = millis();
            Serial.print(F("[KICK] w="));  Serial.print(w);
            Serial.print(F(" SP="));       Serial.print(SET_RPM[w], 1);
            Serial.print(F(" RPM="));      Serial.print(RPM[w], 1);
            Serial.print(F(" U="));        Serial.print(U[w]);
            Serial.print(F(" -> "));       Serial.println(sgnf(SET_RPM[w]) * KICK_PWM);
          }
#endif
        }
      } else {
        stall_accum_ms[w] = 0;  // reset when moving
      }
    } else {
      stall_accum_ms[w] = 0;
    }
  }

  // Apply deadzone bias and kick
  for (uint8_t w = 0; w < 4; w++) {
    if (fabs(SET_RPM[w]) >= SP_ZERO_RPM) {
      if (abs(U[w]) < DZ_BIAS_PWM) U[w] = sgnf(SET_RPM[w]) * DZ_BIAS_PWM;  // minimum bias
      if ((long)(kick_until_ms[w] - millis()) > 0) {                        // kick window
        int want = sgnf(SET_RPM[w]) * KICK_PWM;
        if (abs(U[w]) < abs(want)) U[w] = want;
#if USE_PID
        Iterm[w] *= 0.95f;  // bleed integrator during kick
#endif
      }
    } else {
      if (abs(U[w]) < PWM_IDLE_CUTOFF) U[w] = 0; // near zero: enforce cutoff
    }
  }

  // Slip limiting per side
  auto limitSlipPair = [&](uint8_t a, uint8_t b) {
    if (sgnf(SET_RPM[a]) == 0 || sgnf(SET_RPM[a]) != sgnf(SET_RPM[b])) return; // same direction only
    float ra = fabs(RPM[a]), rb = fabs(RPM[b]);
    uint8_t fast = (ra >= rb) ? a : b;
    uint8_t slow = (fast == a) ? b : a;
    float diff = fabs(ra - rb);
    if (diff > SLIP_RPM) {
#if TRACTION_DEBUG
      uint8_t sideIdx = (a == 0 && b == 2) ? 0 : 1; // 0=L, 1=R
      if (millis() - lastSlipLogMs[sideIdx] > 200) {
        lastSlipLogMs[sideIdx] = millis();
        Serial.print(F("[SLIP] side=")); Serial.print(sideIdx ? 'R' : 'L');
        Serial.print(F(" fast w="));      Serial.print(fast);
        Serial.print(F(" slow w="));      Serial.print(slow);
        Serial.print(F(" rpm=("));        Serial.print(RPM[fast],1);
        Serial.print('/');                Serial.print(RPM[slow],1);
        Serial.print(F(") diff="));       Serial.print(diff,1);
        Serial.print(F(" cap→"));         Serial.print(U[fast]);
        Serial.print(F(" boost→"));       Serial.println(U[slow]);
      }
#endif
      int capped = sgnf(U[fast]) * (int)SLIP_CAP_PWM;    // cap fast wheel
      if (abs(U[fast]) > abs(capped)) {
        U[fast] = capped;
#if USE_PID
        Iterm[fast] *= 0.95f; // bleed integrator when clipping
#endif
      }
      U[slow] = constrain(U[slow] + sgnf(SET_RPM[slow]) * (int)SLOW_BOOST_PWM, -255, 255); // nudge slow
    }
  };
  limitSlipPair(0, 2); // left side pair
  limitSlipPair(1, 3); // right side pair

  // Idle cutoff one more time (after slip logic)
  for (uint8_t w = 0; w < 4; w++) if (abs(U[w]) < PWM_IDLE_CUTOFF) U[w] = 0;

  // Send to motors
  motorSetPWM(0, U[0]);  // FL
  motorSetPWM(1, U[1]);  // FR
  motorSetPWM(2, U[2]);  // BL
  motorSetPWM(3, U[3]);  // BR

  // 6) Blade control — INDEPENDENT of driving
  if (ch5_blade) bladeSetPWM(ch6_blade_pwm, /*forward=*/true);
  else           bladeStop();

  // 7) Periodic print
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 250) {
    lastPrint = millis();
    Serial.print(F("RAW(us): "));
    for (int i = 0; i < 6; i++) { Serial.print(chRaw[i]); if (i < 5) Serial.print(' '); }
    Serial.print(F(" | RC: s="));  Serial.print(ch1_steer, 2);
    Serial.print(F(" tb="));       Serial.print(ch3_th_base, 2);
    Serial.print(F(" ta="));       Serial.print(ch2_th_add, 2);
    Serial.print(F(" th="));       Serial.print(th_comb, 2);
    Serial.print(F(" sp="));       Serial.print(ch4_spin, 2);
    Serial.print(F(" | SP[L/R]=(")); Serial.print(sp_left, 1); Serial.print('/'); Serial.print(sp_right, 1); Serial.print(')');
    Serial.print(F("  RPM[L/R]=("));  Serial.print(RPM[0], 1);  Serial.print('/'); Serial.print(RPM[1], 1);  Serial.print(')');
    Serial.print(F("  Blade="));      Serial.print(ch5_blade ? ch6_blade_pwm : 0);
    Serial.print(F("  PWM[FL,FR,BL,BR]=("));
    Serial.print(U[0]); Serial.print(',');
    Serial.print(U[1]); Serial.print(',');
    Serial.print(U[2]); Serial.print(',');
    Serial.print(U[3]); Serial.println(')');
  }

  delay(CTRL_PERIOD_MS);
}
