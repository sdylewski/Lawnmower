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
  - Optional PID (Tyreus–Luyben gains you measured). Leave USE_PID=0 to test open-loop.

  QUICK TOGGLES:
    #define USE_PID          0   // 1 to use PID on wheel RPM, 0 for open-loop PWM
    #define USE_FEEDFORWARD  0   // keep 0 (you said you don’t need FF now)
*/

#include <Arduino.h>

// =================== QUICK TOGGLES ===================
#define USE_PID          1        // 0=open-loop testing; 1=closed-loop with PID
#define USE_FEEDFORWARD  1        // keep 0 for now

// =================== HARDWARE PINS ===================
const uint8_t ENABLE_PIN = 52;     // Kill switch (INPUT_PULLUP). LOW = STOP.

// Encoders: A (interrupt-capable), B (any digital)
const uint8_t ENC_A_PINS[4] = { 19, 18, 21, 20 };  // FL, FR, BL, BR
const uint8_t ENC_B_PINS[4] = { 56, 58, 64, 66 };  // FL, FR, BL, BR

// Motor driver pins (TB6612/L298 style): IN1/IN2 + PWM (MEGA PWM: 2..13, 44..46)
const uint8_t PWM_PINS[4]   = { 8,  2,  9, 10 };   // FL, FR, BL, BR
const uint8_t IN1_PINS[4]   = { 6,  4, 16, 22 };
const uint8_t IN2_PINS[4]   = { 7,  3, 15, 23 };

// Optional STBY/EN (255 if tied high or N/A)
const uint8_t STBY_PINS[4]  = { 5, 5, 17, 17 };

// Blade (IBT-2 / BTS7960): separate PWM for FWD and REV + STBY/EN
const uint8_t BLADE_PWM_FWD = 11;   // set to a PWM-capable pin
const uint8_t BLADE_PWM_REV = 12;   // set to a PWM-capable pin
const uint8_t BLADE_STBY    = 13;   // or 255 if not used/tied HIGH

// RC input pins (6 PWM channels on MEGA)
const uint8_t CH_PINS[6] = { 46, 44, 42, 40, 38, 36 }; // CH1..CH6

// =================== MECHANICS / SIGNS ===================
static const float CPR_MOTOR  = 16.0f;
static const float GEAR_RATIO = 150.0f;
static const float PPR        = CPR_MOTOR * GEAR_RATIO; // pulses per rev (A channel only)
static const float MAX_WHEEL_RPM = 75.0f;               // tweak to your wheel top-speed
static const float SPIN_MAX_RPM  = 65.0f;               // spin-in-place target max RPM

// Motor/encoder polarity
int8_t MOTOR_SIGN[4] = { -1, -1, -1, -1 };   // set so “forward” makes robot go forward
int8_t ENC_SIGN[4]   = { -1, +1, -1, +1 };   // set so measured RPM increases forward

// RC polarity (flip stick directions if needed)
int8_t RC_POL_CH1_STEER   = -1;  // +1 normal, -1 invert steering
int8_t RC_POL_CH2_TH_ADD  = +1;  // +1 normal, -1 invert add-throttle
int8_t RC_POL_CH3_TH_BASE = +1;  // usually +1 (0..0.5)
int8_t RC_POL_CH4_SPIN    = +1;  // +1 normal, -1 invert spin

// =================== CONTROL LOOP TIMING ===================
static const uint16_t CTRL_PERIOD_MS = 50;   // ~20 Hz main loop

// =================== RC SCALING / FILTERS ===================
#define USE_PULLUPS 0 // RX inputs; set 1 if you need internal pull-ups

// Deadbands
static const float DB_STEER   = 0.05f;
static const float DB_TH_BASE = 0.03f;   // unipolar (0..0.5)
static const float DB_TH_ADD  = 0.05f;
static const float DB_SPIN    = 0.05f;

// Smoothing alphas (0..1): higher = snappier
static const float ALPHA_STEER   = 0.25f;
static const float ALPHA_TH_BASE = 0.25f;
static const float ALPHA_TH_ADD  = 0.25f;
static const float ALPHA_SPIN    = 0.25f;

// Spin is active only when |combined throttle| < SPIN_THRESH
static const float SPIN_THRESH = 0.10f;

// =================== PID GAINS (Tyreus–Luyben you measured) ===================
struct PIDG { float Kp, Ki, Kd; };
PIDG PID_GAIN[4] = {
/* FL */ { 3.8537f, 8.7148f, 0.1232f },
/* FR */ { 3.4195f, 7.7328f, 0.1093f },
/* BL */ { 3.4683f, 7.8433f, 0.1108f },
/* BR */ { 3.4195f, 7.7328f, 0.1093f },
};

// =================== STATE ===================
// Encoders
volatile long ENC_CNT[4] = {0,0,0,0};
long prevCnt[4] = {0,0,0,0};
unsigned long lastRPMus = 0;
float RPM[4] = {0,0,0,0};        // measured RPM (sign reflects direction)

// PID accumulators
float Iterm[4] = {0,0,0,0};
float lastErr[4] = {0,0,0,0};

// RC raw microseconds (defaults chosen so throttle base=0 at boot)
volatile uint16_t chRaw[6] = {1500,1500,1000,1500,1000,1000};

// RC filtered/scaled
float ch1_steer = 0, ch2_th_add = 0, ch3_th_base = 0, ch4_spin = 0;
uint8_t ch5_blade = 0;
int ch6_blade_pwm = 0;

// =================== UTILS ===================
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo; if (x > hi) return hi; return x;
}
static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  float t = (x - in_min) / (in_max - in_min);
  return out_min + t * (out_max - out_min);
}
static inline bool killActive(){ return digitalRead(ENABLE_PIN) == LOW; }

// =================== ENCODER ISRs ===================
static inline void onAEdge(uint8_t i) {
  int8_t step = digitalRead(ENC_B_PINS[i]) ? -1 : +1; // B=1 one dir, B=0 other
  ENC_CNT[i] += ENC_SIGN[i] * step;
}
void isrA0(){ onAEdge(0); } // FL
void isrA1(){ onAEdge(1); } // FR
void isrA2(){ onAEdge(2); } // BL
void isrA3(){ onAEdge(3); } // BR

// =================== MOTOR LOW-LEVEL ===================
void enableAllDrivers() {
  for (uint8_t w=0; w<4; w++) if (STBY_PINS[w] != 255) { pinMode(STBY_PINS[w], OUTPUT); digitalWrite(STBY_PINS[w], HIGH); }
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
void stopAllMotors() { for (uint8_t w=0; w<4; w++) motorStop(w); }
void bladeSetPWM(int pwm, bool forward=true){
  pwm = constrain(pwm, 0, 255);
  if (BLADE_STBY != 255) digitalWrite(BLADE_STBY, HIGH);
  if (forward) {
    analogWrite(BLADE_PWM_FWD, pwm);
    analogWrite(BLADE_PWM_REV, 0);
  } else {
    analogWrite(BLADE_PWM_FWD, 0);
    analogWrite(BLADE_PWM_REV, pwm);
  }
}
void bladeStop(){
  analogWrite(BLADE_PWM_FWD, 0);
  analogWrite(BLADE_PWM_REV, 0);
}

// =================== RPM UPDATE (non-blocking) ===================
void updateRPM() {
  unsigned long now = micros();
  if (lastRPMus == 0) { lastRPMus = now; return; }
  float dt = (now - lastRPMus) / 1e6f;
  if (dt <= 0.0f) return;

  for (uint8_t w=0; w<4; w++) {
    long c; noInterrupts(); c = ENC_CNT[w]; interrupts();
    long d = c - prevCnt[w];
    prevCnt[w] = c;
    float rps = (d / PPR) / dt;   // counts→revs then /dt
    float rpm = rps * 60.0f;
    // light smoothing for display
    RPM[w] = 0.6f*RPM[w] + 0.4f*rpm;
  }
  lastRPMus = now;
}

// =================== RC READER (wait-while-HIGH → pulseIn(HIGH)) ===================
static inline uint16_t readPulseUS_waitIfHigh(uint8_t pin, unsigned long timeout_us = 30000UL) {
  unsigned long t0 = micros();
  while (digitalRead(pin) == HIGH) {         // avoid catching a half pulse
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
  for (uint8_t i=0; i<6; i++) {
    uint16_t us = readPulseUS_waitIfHigh(CH_PINS[i], 30000UL);
    if (us != 0) chRaw[i] = us; // keep last good on timeout
  }
}

// =================== PID ===================
int wheelPID(uint8_t w, float sp_rpm, float fb_rpm, float dt) {
#if USE_PID
  const float Kp = PID_GAIN[w].Kp;
  const float Ki = PID_GAIN[w].Ki;
  const float Kd = PID_GAIN[w].Kd;
  // Using signed RPM error:
  float e = sp_rpm - fb_rpm;                         // error (RPM)
  Iterm[w] += Ki * e * dt;                           // integrate
  Iterm[w] = clampf(Iterm[w], -255.0f, 255.0f);      // anti-windup
  float dterm = (e - lastErr[w]) / dt;               // derivative
  lastErr[w] = e;
  float u = Kp*e + Iterm[w] + Kd*dterm;              // control effort (PWM counts)
  // optional FF not used (you said no)
  int pwm = (int)lround(clampf(u, -255.0f, 255.0f));
  return pwm;
#else
  (void)w; (void)sp_rpm; (void)fb_rpm; (void)dt;
  return 0;
#endif
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(ENABLE_PIN, INPUT_PULLUP);

  // RC inputs
  for (uint8_t i=0; i<6; i++) {
#if USE_PULLUPS
    pinMode(CH_PINS[i], INPUT_PULLUP);
#else
    pinMode(CH_PINS[i], INPUT);
#endif
  }

  // Motors & encoders
  for (uint8_t w=0; w<4; w++) {
    pinMode(ENC_A_PINS[w], INPUT_PULLUP);
    pinMode(ENC_B_PINS[w], INPUT_PULLUP);
    pinMode(IN1_PINS[w], OUTPUT);
    pinMode(IN2_PINS[w], OUTPUT);
    pinMode(PWM_PINS[w], OUTPUT);
  }
  pinMode(BLADE_PWM_FWD, OUTPUT);
  pinMode(BLADE_PWM_REV, OUTPUT);
  if (BLADE_STBY != 255) pinMode(BLADE_STBY, OUTPUT);

  enableAllDrivers();

  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[0]), isrA0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[1]), isrA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[2]), isrA2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[3]), isrA3, RISING);

  Serial.println(F("MEGA 4WD Driver — RC + (optional) PID + Blade (IBT-2)"));
  Serial.print(F("Modes: USE_PID=")); Serial.print((int)USE_PID);
  Serial.print(F(", USE_FEEDFORWARD=")); Serial.println((int)USE_FEEDFORWARD);
  Serial.println();
}

// =================== MAIN LOOP ===================
void loop() {
  // KILL holds everything off
  if (killActive()) {
    stopAllMotors();
    bladeStop();
    static unsigned long t0=0;
    if (millis()-t0 > 500) { t0=millis(); Serial.println(F("[KILL] D52 LOW — holding.")); }
    delay(10);
    return;
  }

  // 1) Read RC
  readChannelsPWM();

  // 2) Scale + polarity + smoothing + deadbands
  // steering [-1..+1]
  float raw_steer = clampf(mapf(chRaw[0], 1000, 2000, -1.0f,  1.0f), -1.0f, 1.0f) * RC_POL_CH1_STEER;
  ch1_steer = (1-ALPHA_STEER)*ch1_steer + ALPHA_STEER*raw_steer;
  if (fabs(ch1_steer) < DB_STEER) ch1_steer = 0.0f;

  // additive throttle [-0.5..+0.5]
  float raw_th_add = clampf(mapf(chRaw[1], 1000, 2000, -0.5f, +0.5f), -0.5f, 0.5f) * RC_POL_CH2_TH_ADD;
  ch2_th_add = (1-ALPHA_TH_ADD)*ch2_th_add + ALPHA_TH_ADD*raw_th_add;
  if (fabs(ch2_th_add) < DB_TH_ADD) ch2_th_add = 0.0f;

  // base throttle [0..0.5]
  float raw_th_base = clampf(mapf(chRaw[2], 1000, 2000, 0.0f, 0.5f), 0.0f, 0.5f) * RC_POL_CH3_TH_BASE;
  // if polarity flips it negative, clamp back to [0..0.5]
  raw_th_base = clampf(raw_th_base, 0.0f, 0.5f);
  ch3_th_base = (1-ALPHA_TH_BASE)*ch3_th_base + ALPHA_TH_BASE*raw_th_base;
  if (ch3_th_base < DB_TH_BASE) ch3_th_base = 0.0f;

  // spin [-1..+1]
  float raw_spin = clampf(mapf(chRaw[3], 1000, 2000, -1.0f,  1.0f), -1.0f, 1.0f) * RC_POL_CH4_SPIN;
  ch4_spin = (1-ALPHA_SPIN)*ch4_spin + ALPHA_SPIN*raw_spin;
  if (fabs(ch4_spin) < DB_SPIN) ch4_spin = 0.0f;

  // blade switch & speed
  ch5_blade = (chRaw[4] >= 1500) ? 1 : 0;
  ch6_blade_pwm = constrain(map((int)chRaw[5], 1000, 2000, 0, 255), 0, 255);

  // 3) Combine throttle and create RPM setpoints (signed)
  float th_comb = clampf(ch3_th_base + ch2_th_add, -0.5f, 1.0f);    // [-0.5..+1.0]
  float sp_left = 0.0f, sp_right = 0.0f;

  if (fabs(th_comb) < SPIN_THRESH && ch4_spin != 0.0f) {
    // Spin in place: opposite RPMs (ignore throttle)
    sp_left  =  ch4_spin * SPIN_MAX_RPM;
    sp_right = -ch4_spin * SPIN_MAX_RPM;
  } else {
    // Arcade-style mix with modest steer influence (±0.5 of max)
    float baseRPM = th_comb * MAX_WHEEL_RPM;
    float steerRPM = ch1_steer * (0.5f * MAX_WHEEL_RPM);
    // Note sign: left reduces with +steer, right increases
    sp_left  = clampf(baseRPM - steerRPM, -MAX_WHEEL_RPM, MAX_WHEEL_RPM);
    sp_right = clampf(baseRPM + steerRPM, -MAX_WHEEL_RPM, MAX_WHEEL_RPM);
  }

  // 4) Update measured RPMs
  updateRPM();

  // 5) Command wheels (open-loop or PID)
  //   Left side = wheels 0 (FL) & 2 (BL)
  //   Right side= wheels 1 (FR) & 3 (BR)
  float dt = CTRL_PERIOD_MS / 1000.0f;

  int pwmL = 0, pwmR = 0;

#if USE_PID
  // PID per side → same setpoint shared to front/back on each side
  pwmL = wheelPID(0, sp_left,  RPM[0], dt);
  pwmR = wheelPID(1, sp_right, RPM[1], dt);
  // simple side sharing (optional: average the feedbacks)
  // apply to both wheels on each side
  motorSetPWM(0, pwmL);
  motorSetPWM(2, pwmL);
  motorSetPWM(1, pwmR);
  motorSetPWM(3, pwmR);
#else
  // Open-loop: map desired RPM to PWM directly
  auto rpm2pwm = [&](float sp)->int {
    float m = clampf(sp / MAX_WHEEL_RPM, -1.0f, 1.0f);
    int u = (int)lround(m * 255.0f);
    return constrain(u, -255, 255);
  };
  pwmL = rpm2pwm(sp_left);
  pwmR = rpm2pwm(sp_right);
  motorSetPWM(0, pwmL);
  motorSetPWM(2, pwmL);
  motorSetPWM(1, pwmR);
  motorSetPWM(3, pwmR);
#endif

  // 6) Blade control
  if (ch5_blade) bladeSetPWM(ch6_blade_pwm, /*forward=*/true);
  else           bladeStop();

  // 7) Periodic print
  static unsigned long lastPrint=0;
  if (millis()-lastPrint >= 250) {
    lastPrint = millis();
    Serial.print(F("RAW(us): "));
    for (int i=0;i<6;i++){ Serial.print(chRaw[i]); if(i<5) Serial.print(' '); }
    Serial.print(F(" | RC: s="));  Serial.print(ch1_steer,2);
    Serial.print(F(" tb="));       Serial.print(ch3_th_base,2);
    Serial.print(F(" ta="));       Serial.print(ch2_th_add,2);
    Serial.print(F(" th="));       Serial.print(th_comb,2);
    Serial.print(F(" sp="));       Serial.print(ch4_spin,2);
    Serial.print(F(" | SP[L/R]=(")); Serial.print(sp_left,1); Serial.print('/'); Serial.print(sp_right,1); Serial.print(')');
    Serial.print(F("  RPM[L/R]=(")); Serial.print(RPM[0],1);  Serial.print('/'); Serial.print(RPM[1],1);  Serial.print(')');
    Serial.print(F("  Blade="));    Serial.print(ch5_blade ? ch6_blade_pwm : 0);
    Serial.print(F("  PWM[L/R]=(")); Serial.print(pwmL); Serial.print('/'); Serial.print(pwmR); Serial.println(')');
  }

  delay(CTRL_PERIOD_MS);
}
