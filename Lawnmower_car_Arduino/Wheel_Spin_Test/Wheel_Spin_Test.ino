/*
  MEGA 4WD Driver + RC + PID(on |RPM|) + Blade (IBT-2) — TESTABLE MODES

  - RC PWM inputs: CH1..CH6 on D46,44,42,40,38,36
  - Mix: throttle(CH2) + steer(CH1) + left-add(CH3). CH4 spins in place only when near zero speed.
  - Encoders: x1 per wheel (A on interrupt pin; B any digital)
  - Motor drivers (wheels): TB6612/L298 style IN1/IN2 + PWM
  - Blade: IBT-2 (BTS7960) with separate FWD/REV PWM pins (+ optional STBY)
  - Kill switch: D52 (INPUT_PULLUP). LOW = STOP.

  Quick test switches:
    USE_PID         -> 0 = open-loop (no PID), 1 = PID enabled
    USE_FEEDFORWARD -> 0 = none, 1 = use PW M ≈ a*RPM + b (when USE_PID==0 or to pre-bias PID)

  Notes:
    - PID regulates absolute RPM, then applies sign from setpoint => reverse works cleanly.
    - MIN_CMD_PWM helps overcome stiction (set 20..40 if needed).
*/

#include <Arduino.h>

//============================= TOGGLES ==================================
#define USE_PID          0   // 1 = PID on |RPM|, 0 = open-loop
#define USE_FEEDFORWARD  0   // 1 = use FF (a,b) when open-loop or as prebias, 0 = disabled

//=========================== RC CONFIG ==================================
// RC pins (MEGA)
const uint8_t CH_PINS[6] = {46, 44, 42, 40, 38, 36}; // CH1..CH6 (D46,D44,D42,D40,D38,D36)

// RC smoothing + deadband
static const float RC_ALPHA = 0.15f;  // 0..1, higher = snappier
static const float DB_STEER = 0.04f;  // deadband for steering
static const float DB_THROT = 0.04f;  // deadband for throttle
static const float DB_LADD  = 0.04f;  // deadband for left additive
static const float DB_SPIN  = 0.04f;  // deadband for in-place spin command (CH4)

// Print rate
static const uint16_t PRINT_MS = 200;

//=========================== KILL SWITCH ================================
static const uint8_t ENABLE_PIN = 52; // INPUT_PULLUP; LOW = STOP

//=========================== BLADE (IBT-2) ==============================
// Define your actual blade pins here:
static const uint8_t BLADE_PWM_FWD_PIN = 24; // RPWM -> PWM
static const uint8_t BLADE_PWM_REV_PIN = 25; // LPWM -> PWM
static const uint8_t BLADE_STBY_PIN    = 255; // 255 = unused; else OUTPUT HIGH to enable

//=========================== WHEELS / ENCODERS ==========================
// Wheel order: 0=FL, 1=FR, 2=BL, 3=BR

// Encoders: A must be interrupt-capable; B any digital.
const uint8_t ENC_A_PINS[4] = { 19, 18, 21, 20 }; // FL,FR,BL,BR
const uint8_t ENC_B_PINS[4] = { 56, 58, 64, 66 }; // FL,FR,BL,BR

// Motor driver pins per wheel (TB6612/L298 style)
const uint8_t PWM_PINS[4] = { 8, 2, 9, 10 };      // PWM pins
const uint8_t IN1_PINS[4] = { 6, 4, 16, 22 };     // IN1
const uint8_t IN2_PINS[4] = { 7, 3, 15, 23 };     // IN2

// Optional TB6612 STBY pins (255 if tied HIGH)
const uint8_t STBY_PINS[4] = { 5, 5, 17, 17 };

// Motor / encoder polarity
int8_t MOTOR_SIGN[4] = { -1, -1, -1, -1 }; // flip if forward is backward for that wheel
int8_t ENC_SIGN[4]   = { +1, +1, -1, +1 }; // flip if measured sign is wrong

// Mechanics (x1 on A)
static const float CPR_MOTOR  = 16.0f;
static const float GEAR_RATIO = 150.0f;
static const float PPR        = CPR_MOTOR * GEAR_RATIO; // pulses per rev (single channel)

//=========================== CONTROL LOOP ===============================
static const float CONTROL_HZ = 50.0f;     // main control loop rate
static const float Ts         = 1.0f / CONTROL_HZ; // loop period (s)

// Command scaling
static const float MAX_CMD_RPM = 80.0f;    // top speed (per wheel) used for mapping

// Open-loop helper (if PID off)
static const int MIN_CMD_PWM   = 0;        // try 20..40 to overcome stiction, 0 = disabled

//=========================== PID GAINS (Tyreus–Luyben) ==================
// Paste from your autotune runs; order: FL, FR, BL, BR
struct PIDG { float Kp, Ki, Kd; };
PIDG PID_GAIN[4] = {
  { 3.8537f, 8.7148f, 0.1232f }, // FL
  { 3.4195f, 7.7328f, 0.1093f }, // FR
  { 3.4683f, 7.8433f, 0.1108f }, // BL
  { 3.4195f, 7.7328f, 0.1093f }, // BR
};

// PID (on |RPM|) state
static float integ_abs[4]     = {0,0,0,0};
static float d_abs[4]         = {0,0,0,0};
static float meas_abs_prev[4] = {0,0,0,0};
static const float Td_over_N_abs = 0.02f;    // derivative filter time (s/N)
static const float Imax_abs       = 40.0f;   // integrator clamp
static const float SP_STOP_RPM    = 0.5f;    // setpoint near 0 => integrator reset

//=========================== FEED-FORWARD (optional) ====================
// PWM ≈ a * RPM + b  (used when USE_FEEDFORWARD=1)
// If these are all zeros, FF is effectively disabled.
struct FF { float a; float b; };
FF FEEDFORWARD[4] = {
  {0,0}, {0,0}, {0,0}, {0,0}
};

//=========================== STATE ======================================
volatile long ENC_CNT[4] = {0,0,0,0};
long encCntPrev[4] = {0,0,0,0};
float rpm_meas[4]  = {0,0,0,0};

// RC raw and scaled
uint16_t rcRaw[6] = {1500,1500,1500,1500,1000,1000}; // defaults
float ch1_lr = 0, ch2_th = 0, ch3_ladd = 0, ch4_spin = 0;
uint8_t ch5_blade = 0;
uint8_t ch6_blade_pwm = 0;

// Smoothed commands (for nicer driving)
float sm_steer = 0, sm_throt = 0, sm_ladd = 0, sm_spin = 0;

// Setpoints per side (RPM, signed)
float sp_left_rpm  = 0;
float sp_right_rpm = 0;

//=========================== HELPERS ====================================
static inline float clampf(float x, float lo, float hi){ return (x<lo)?lo:((x>hi)?hi:x); }
static inline float deadbandf(float x, float db)       { return (fabs(x) < db) ? 0.0f : x; }
static inline uint16_t pulseInSmart(uint8_t pin, unsigned long tout=30000UL) {
  // Try HIGH pulses first (typical), then LOW pulses (some receivers invert)
  uint32_t us = pulseIn(pin, HIGH, tout);
  if (us == 0) us = pulseIn(pin, LOW, tout);
  return (uint16_t)us;
}
inline bool killActive() { return digitalRead(ENABLE_PIN) == LOW; }

// Encoder ISR (x1 on A rising; direction by B)
static inline void onAEdge(uint8_t i) {
  int8_t step = digitalRead(ENC_B_PINS[i]) ? -1 : +1; // B=1 one dir, B=0 other
  ENC_CNT[i] += ENC_SIGN[i] * step;                   // apply encoder polarity
}
void isrA0(){ onAEdge(0); }
void isrA1(){ onAEdge(1); }
void isrA2(){ onAEdge(2); }
void isrA3(){ onAEdge(3); }

inline float rpmFromCounts(long dcounts, float dt_s) {
  const float counts_per_rev = PPR;                   // x1 on A
  float rps = (dcounts / counts_per_rev) / dt_s;      // rev/s
  return rps * 60.0f;                                 // RPM
}

// Wheel H-bridge control: signed PWM
void motorSetPWM_signed(uint8_t w, int pwmSigned) {
  int cmd = pwmSigned * MOTOR_SIGN[w];                // apply motor polarity
  bool fwd = (cmd >= 0);                              // direction
  uint8_t mag = (uint8_t)clampf(abs(cmd), 0, 255);    // magnitude 0..255
  digitalWrite(IN1_PINS[w], fwd ? HIGH : LOW);        // IN1/IN2 set direction
  digitalWrite(IN2_PINS[w], fwd ? LOW  : HIGH);
  analogWrite(PWM_PINS[w], mag);                      // PWM magnitude
}
void motorStop(uint8_t w){ digitalWrite(IN1_PINS[w],LOW); digitalWrite(IN2_PINS[w],LOW); analogWrite(PWM_PINS[w],0); }
void stopAllMotors(){ for(uint8_t w=0; w<4; ++w) motorStop(w); }

void enableAllDrivers(){
  for(uint8_t w=0; w<4; ++w){
    if(STBY_PINS[w] != 255){ pinMode(STBY_PINS[w], OUTPUT); digitalWrite(STBY_PINS[w], HIGH); }
  }
}

// Blade control (IBT-2)
void bladeEnable(bool en){
  if (BLADE_STBY_PIN != 255){ pinMode(BLADE_STBY_PIN, OUTPUT); digitalWrite(BLADE_STBY_PIN, en?HIGH:LOW); }
}
void bladeSetPWM(int signedPWM){
  int p = signedPWM;                                  // allow future reverse if desired
  bool fwd = (p >= 0);
  uint8_t mag = (uint8_t)clampf(abs(p),0,255);
  if (fwd){ analogWrite(BLADE_PWM_FWD_PIN, mag); analogWrite(BLADE_PWM_REV_PIN, 0); }
  else    { analogWrite(BLADE_PWM_FWD_PIN, 0);   analogWrite(BLADE_PWM_REV_PIN, mag); }
}

//=========================== PID on |RPM| ===============================
int pidStepPWM_abs(uint8_t w, float sp_abs, float meas_abs)
{
  const PIDG g = PID_GAIN[w];

  if (sp_abs < SP_STOP_RPM){                          // near-zero setpoint
    integ_abs[w]=0; d_abs[w]=0; meas_abs_prev[w]=meas_abs;
    return 0;                                         // no drive
  }

  float e = sp_abs - meas_abs;                        // error on magnitudes
  float raw_d = (meas_abs - meas_abs_prev[w]) / Ts;   // derivative of measurement
  meas_abs_prev[w] = meas_abs;

  float alpha = Ts / (Td_over_N_abs + Ts);            // 1st-order D filter coefficient
  d_abs[w] += alpha * (raw_d - d_abs[w]);             // filtered D(meas)

  integ_abs[w] = clampf(integ_abs[w] + e*Ts, -Imax_abs, Imax_abs);  // anti-windup

  float u = g.Kp*e + g.Ki*integ_abs[w] - g.Kd*d_abs[w];             // parallel PID
  if (u < 0) u = 0; if (u > 255) u = 255;                           // clamp to 0..255
  if (MIN_CMD_PWM > 0 && u > 0 && u < MIN_CMD_PWM) u = (float)MIN_CMD_PWM; // stiction kick

  return (int)u;
}

//=========================== FEED-FORWARD MAP ===========================
int ffPWM_from_RPM(uint8_t w, float sp_abs_rpm){
#if USE_FEEDFORWARD
  float a = FEEDFORWARD[w].a, b = FEEDFORWARD[w].b;
  if (a != 0.0f || b != 0.0f){
    float u = a*sp_abs_rpm + b;                       // linear map → PWM
    if (u < 0) u = 0; if (u > 255) u = 255;
    return (int)u;
  }
#endif
  // fallback: simple proportional map to 0..255
  float u = (sp_abs_rpm / MAX_CMD_RPM) * 255.0f;      // scale RPM to PWM
  if (u < 0) u = 0; if (u > 255) u = 255;
  if (MIN_CMD_PWM > 0 && u > 0 && u < MIN_CMD_PWM) u = (float)MIN_CMD_PWM;
  return (int)u;
}

//=========================== RC READ / SCALE ============================
void readChannelsPWM(){
  for (uint8_t i=0; i<6; ++i){
    uint16_t us = pulseInSmart(CH_PINS[i], 30000UL);  // up to 30ms guard
    if (us){
      if (us < 900) us = 900; if (us > 2100) us = 2100;
      rcRaw[i] = us;                                  // keep last good value
    }
  }
}
static inline float map01(uint16_t us){ return clampf((us - 1000) / 1000.0f, 0.0f, 1.0f); } // 1000..2000 -> 0..1
static inline float mapSym(uint16_t us){ return clampf((us - 1500) / 500.0f, -1.0f, +1.0f);} // 1000..2000 -> -1..+1

void scaleAndSmoothInputs(){
  // Raw → scaled
  float s_steer = mapSym(rcRaw[0]);                   // CH1: steer [-1..+1]
  float s_throt = map01 (rcRaw[1]);                   // CH2: throttle [0..1]
  float s_ladd  = mapSym(rcRaw[2]);                   // CH3: left additive [-1..+1]
  float s_spin  = mapSym(rcRaw[3]);                   // CH4: spin in place [-1..+1]
  ch5_blade     = (rcRaw[4] >= 1500) ? 1 : 0;         // CH5: blade switch
  ch6_blade_pwm = (uint8_t)clampf(map01(rcRaw[5])*255.0f, 0, 255); // CH6: blade speed 0..255

  // Deadband
  s_steer = deadbandf(s_steer, DB_STEER);
  s_throt = deadbandf(s_throt, DB_THROT);
  s_ladd  = deadbandf(s_ladd , DB_LADD );
  s_spin  = deadbandf(s_spin , DB_SPIN );

  // Smooth (low-pass)
  sm_steer = sm_steer + RC_ALPHA*(s_steer - sm_steer); // exponential LPF
  sm_throt = sm_throt + RC_ALPHA*(s_throt - sm_throt);
  sm_ladd  = sm_ladd  + RC_ALPHA*(s_ladd  - sm_ladd );
  sm_spin  = sm_spin  + RC_ALPHA*(s_spin  - sm_spin );

  // Expose scaled (post-smooth) for printing/use
  ch1_lr = sm_steer; ch2_th = sm_throt; ch3_ladd = sm_ladd; ch4_spin = sm_spin;
}

//=========================== MIXING → RPM SETPOINTS =====================
void computeSetpoints(){
  // Base forward speed from throttle (0..1) → RPM
  float base = ch2_th * MAX_CMD_RPM;                  // forward speed (RPM)

  // Steering mixes differential (+ left-add bias)
  float steer = ch1_lr * MAX_CMD_RPM;                 // steering as RPM differential
  float ladd  = ch3_ladd * (0.5f * MAX_CMD_RPM);      // left additive (half-scale)

  // In-place spin only if base ≈ 0
  float spin = 0.0f;
  if (base < 0.08f*MAX_CMD_RPM){                      // near stop? allow spin
    spin = ch4_spin * MAX_CMD_RPM;                    // spin rate in RPM
  }

  // Compose signed RPM setpoints per side
  sp_left_rpm  = (base - steer) + ladd + spin;        // left side target RPM
  sp_right_rpm = (base + steer) - ladd - spin;        // right side target RPM
}

//=========================== SETUP ======================================
void setup(){
  Serial.begin(115200);
  pinMode(ENABLE_PIN, INPUT_PULLUP);

  // RC pins
  for (uint8_t i=0;i<6;i++) pinMode(CH_PINS[i], INPUT_PULLUP);

  // Blade pins
  pinMode(BLADE_PWM_FWD_PIN, OUTPUT);
  pinMode(BLADE_PWM_REV_PIN, OUTPUT);
  bladeEnable(true);
  bladeSetPWM(0);

  // Wheel pins
  for (uint8_t w=0; w<4; ++w){
    pinMode(ENC_A_PINS[w], INPUT_PULLUP);
    pinMode(ENC_B_PINS[w], INPUT_PULLUP);
    pinMode(IN1_PINS[w], OUTPUT);
    pinMode(IN2_PINS[w], OUTPUT);
    pinMode(PWM_PINS[w], OUTPUT);
  }
  enableAllDrivers();

  // Encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[0]), isrA0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[1]), isrA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[2]), isrA2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[3]), isrA3, RISING);

  Serial.println(F("MEGA 4WD Driver — PID(|RPM|) + RC + Blade (IBT-2)"));
  Serial.print  (F("Modes: USE_PID=")); Serial.print(USE_PID);
  Serial.print  (F(", USE_FEEDFORWARD=")); Serial.println(USE_FEEDFORWARD);
}

//=========================== LOOP =======================================
void loop(){
  // Kill switch guard
  if (killActive()){
    stopAllMotors();
    bladeSetPWM(0);
    static unsigned long t0=0;
    if (millis()-t0 > 500){ t0=millis(); Serial.println(F("[KILL] D52 LOW — holding.")); }
    delay(10);
    return;
  }

  // 1) RC read + scaling + smoothing
  readChannelsPWM();
  scaleAndSmoothInputs();

  // 2) Mix to per-side RPM setpoints
  computeSetpoints();

  // 3) Update measured RPMs (non-blocking, once per loop using deltas)
  unsigned long t0 = micros();
  for (uint8_t w=0; w<4; ++w){
    long c; noInterrupts(); c = ENC_CNT[w]; interrupts();
    long dp = c - encCntPrev[w];
    encCntPrev[w] = c;
    rpm_meas[w] = rpmFromCounts(dp, Ts);             // signed RPM (ENC_SIGN already applied in ISR)
  }

  // 4) Command wheels (fronts + rears share side setpoint)
  float spL = sp_left_rpm;
  float spR = sp_right_rpm;

#if USE_PID
  // PID on |RPM|, apply direction afterward
  int uFL = pidStepPWM_abs(0, fabs(spL), fabs(rpm_meas[0])); motorSetPWM_signed(0, (spL>=0?+1:-1) * uFL);
  int uFR = pidStepPWM_abs(1, fabs(spR), fabs(rpm_meas[1])); motorSetPWM_signed(1, (spR>=0?+1:-1) * uFR);
  int uBL = pidStepPWM_abs(2, fabs(spL), fabs(rpm_meas[2])); motorSetPWM_signed(2, (spL>=0?+1:-1) * uBL);
  int uBR = pidStepPWM_abs(3, fabs(spR), fabs(rpm_meas[3])); motorSetPWM_signed(3, (spR>=0?+1:-1) * uBR);
#else
  // OPEN-LOOP: map RPM setpoints directly to PWM (optionally FF)
  int uL = ffPWM_from_RPM(0, fabs(spL));  // use FL params for left side
  int uR = ffPWM_from_RPM(1, fabs(spR));  // use FR params for right side
  motorSetPWM_signed(0, (spL>=0?+1:-1) * uL);
  motorSetPWM_signed(1, (spR>=0?+1:-1) * uR);
  motorSetPWM_signed(2, (spL>=0?+1:-1) * uL);
  motorSetPWM_signed(3, (spR>=0?+1:-1) * uR);
#endif

  // 5) Blade control (CH5 switch + CH6 speed)
  if (ch5_blade){ bladeSetPWM((int)ch6_blade_pwm); } // forward only
  else           { bladeSetPWM(0); }

  // 6) Status print
  static unsigned long lastP=0;
  if (millis()-lastP >= PRINT_MS){
    lastP = millis();
    Serial.print(F("SP[L/R]=(")); Serial.print(spL,1); Serial.print('/'); Serial.print(spR,1); Serial.print(F(")  "));
    Serial.print(F("RPM[L/R]=("));
      Serial.print(0.5f*(rpm_meas[0]+rpm_meas[2]),1); Serial.print('/'); // avg left
      Serial.print(0.5f*(rpm_meas[1]+rpm_meas[3]),1); Serial.print(F(")  "));
    Serial.print(F("Blade=")); Serial.print(ch5_blade ? (int)ch6_blade_pwm : 0); Serial.print(F("  "));
    Serial.print(F("RC: s=")); Serial.print(ch1_lr,2);
    Serial.print(F(" t="));     Serial.print(ch2_th,2);
    Serial.print(F(" la="));    Serial.print(ch3_ladd,2);
    Serial.print(F(" sp="));    Serial.print(ch4_spin,2);
    Serial.println();
  }

  // 7) Enforce loop rate
  unsigned long used_us = micros() - t0;
  long left_us = (long)(Ts*1000000.0f) - (long)used_us;
  if (left_us > 0) delayMicroseconds((unsigned long)left_us);
}
