/*
  IBT-2 / BTS7960 ramp up, ramp down, then off (UNO)
  RPWM -> D10 (PWM)
  LPWM -> D11 (PWM)
  EN   -> D12  (enable, HIGH = on)

  Behavior:
  - Ramp UP (0..255) over 10 s, printing PWM each step
  - Ramp DOWN (255..0) over 10 s, printing PWM each step
  - OFF for 10 s (EN=LOW)
  - Repeat
*/

const uint8_t PIN_RPWM = 11;
const uint8_t PIN_LPWM = 12;
const uint8_t PIN_EN   = 13;

const unsigned long RAMP_UP_MS   = 10000UL;  // 10 s up-ramp
const unsigned long RAMP_DOWN_MS = 10000UL;  // 10 s down-ramp
const unsigned long OFF_MS       = 10000UL;  // 10 s off

void setup() {
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_EN,   OUTPUT);

  Serial.begin(115200);
  while (!Serial) { /* wait on native USB boards; safe on Uno */ }

  digitalWrite(PIN_EN, HIGH);   // enable driver
  analogWrite(PIN_RPWM, 0);
  analogWrite(PIN_LPWM, 0);

  Serial.println(F("IBT-2 ramp test starting..."));
}

// Helper to do a timed ramp with printouts and timing compensation
void rampPWM(int start, int end, unsigned long total_ms) {
  // Keep LPWM low for forward; RPWM carries PWM
  analogWrite(PIN_RPWM, 0);

  // 256 steps would repeat 0 twice; use 255 distinct steps (0..255)
  const int steps = 255;
  const unsigned long step_ms = total_ms / steps;

  // Direction: +1 (up) or -1 (down)
  int dir = (end >= start) ? 1 : -1;

  int duty = start;
  for (int i = 0; i <= steps; i++) {
    unsigned long t0 = millis();

    // Clamp duty to 0..255 and write
    if (duty < 0) duty = 0;
    if (duty > 255) duty = 255;
    analogWrite(PIN_RPWM, duty);

    // Print current PWM
    Serial.print(F("PWM: "));
    Serial.println(duty);

    // Next duty
    duty += dir;

    // Timing compensation so each step period ~ step_ms even with print time
    unsigned long elapsed = millis() - t0;
    if (elapsed < step_ms) {
      delay(step_ms - elapsed);
    } else {
      // If prints take longer, we just skip delay to keep moving
    }
  }
}

void loop() {
  // RAMP UP 0 -> 255
  rampPWM(0, 255, RAMP_UP_MS);

  // RAMP DOWN 255 -> 0
  rampPWM(255, 0, RAMP_DOWN_MS);

  // OFF phase
  analogWrite(PIN_RPWM, 0);
  analogWrite(PIN_LPWM, 0);
  digitalWrite(PIN_EN, LOW);
  Serial.println(F("PWM: 0 (OFF)"));
  delay(OFF_MS);
  digitalWrite(PIN_EN, HIGH);
  delay(10); // small guard time
}

/*
Notes:
- If the motor spins the wrong way, swap which channel you drive:
  keep RPWM LOW and ramp LPWM instead.
- Printing every step adds overhead; the code compensates so each ramp
  stays close to 10 s.
*/
