# === Wheel Direction Checker (robot-forward) ===
# Spins each wheel in the ROBOT-FORWARD direction you define in WHEEL_DIR.
# Lets you confirm each wheel is correct visually.

from machine import Pin, PWM
from rp2 import PIO, StateMachine, asm_pio
from time import sleep

# ---------------- WHEEL DIR SETTINGS ----------------
# +1 means +PWM spins the wheel in robot-forward direction
# -1 means -PWM spins the wheel in robot-forward direction
# Set these values from your best guess or previous test
WHEEL_DIR = {
    "FL": -1,
    "FR": +1,
    "BL": -1,
    "BR": +1,
}

# ---------------- CONFIG ----------------
PIO_CHUNK   = 64
TEST_PWM    = 120    # Safe visible speed (0..255)
SPIN_TIME_S = 2.0    # seconds per wheel

# ---------------- MOTOR PINS ----------------
pins = {
    "BR": (PWM(Pin(2)),  Pin(4, Pin.OUT),  Pin(3, Pin.OUT)),
    "BL": (PWM(Pin(7)),  Pin(5, Pin.OUT),  Pin(6, Pin.OUT)),
    "FR": (PWM(Pin(10)), Pin(12, Pin.OUT), Pin(11, Pin.OUT)),
    "FL": (PWM(Pin(15)), Pin(13, Pin.OUT), Pin(14, Pin.OUT)),
}
for pwm, _, _ in pins.values():
    pwm.freq(20000)

stby  = Pin(1, Pin.OUT)
stby.high()

def motor_hw(wheel, pwm_cmd_hw):
    pwm, in1, in2 = pins[wheel]
    d = 1 if pwm_cmd_hw >= 0 else 0
    a = abs(int(pwm_cmd_hw))
    if a > 255: a = 255
    in1.value(d); in2.value(not d)
    pwm.duty_u16(a * 257)

def stop_all():
    for w in pins:
        motor_hw(w, 0)

# ---------------- MAIN ----------------
def main():
    print("\n=== ROBOT-FORWARD WHEEL CHECK ===")
    print("Spinning each wheel in ROBOT-FORWARD direction as set in WHEEL_DIR.")
    print("Confirm visually that all wheels move in the correct forward direction.\n")

    try:
        for w in ["FL", "FR", "BL", "BR"]:
            print(f"Wheel {w} → direction {WHEEL_DIR[w]} ({'FWD' if WHEEL_DIR[w] == 1 else 'REV'})")
            motor_hw(w, TEST_PWM * WHEEL_DIR[w])
            sleep(SPIN_TIME_S)
            motor_hw(w, 0)
            sleep(0.5)
        print("\nCheck complete. Adjust WHEEL_DIR at top if needed.")
    finally:
        stop_all()

try:
    main()
except KeyboardInterrupt:
    stop_all()
    print("\nStopped.")
