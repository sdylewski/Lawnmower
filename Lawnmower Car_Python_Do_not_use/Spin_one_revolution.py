# === One-revolution PPR calibrator ===
from machine import Pin, PWM
from rp2 import PIO, StateMachine, asm_pio
from time import sleep, ticks_ms, ticks_diff

# ---- Pick the wheel to measure ----
WHEEL   = "FR"       # change to "FL","FR","BL","BR"
PWM_CMD = 70         # gentle spin so you can catch a single revolution

# Use CHUNK=1 so we count every edge and see the truth
PIO_CHUNK = 1

# Pins
pins = {
    "BR": (PWM(Pin(2)),  Pin(4, Pin.OUT),  Pin(3, Pin.OUT)),
    "BL": (PWM(Pin(7)),  Pin(5, Pin.OUT),  Pin(6, Pin.OUT)),
    "FR": (PWM(Pin(10)), Pin(12, Pin.OUT), Pin(11, Pin.OUT)),
    "FL": (PWM(Pin(15)), Pin(13, Pin.OUT), Pin(14, Pin.OUT)),
}
for pwm, _, _ in pins.values():
    pwm.freq(20000)

encoder_pins = {"FL":17, "FR":28, "BL":8, "BR":0}
counts = {w: 0 for w in encoder_pins}

# stby pin for motor driver
stby  = Pin(1, Pin.OUT);
stby.high()

@asm_pio()
def chunk_counter():
    pull()
    mov(y, osr)
    mov(x, y)
    label("loop")
    wait(1, pin, 0)
    wait(0, pin, 0)
    jmp(y_dec, "loop")
    irq(rel(0))
    mov(y, x)
    jmp("loop")

def make_irq_handler(w):
    def handler(sm):
        counts[w] += PIO_CHUNK
    return handler

def start_sms():
    sms = {}
    for i, (w, pin_num) in enumerate(encoder_pins.items()):
        pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        sm = StateMachine(i, chunk_counter, in_base=pin)
        sm.irq(make_irq_handler(w))
        sm.put(PIO_CHUNK)
        sm.active(1)
        sms[w] = sm
    return sms

def motor_hw(wheel, pwm_cmd_hw):
    pwm, in1, in2 = pins[wheel]
    d = 1 if pwm_cmd_hw >= 0 else 0
    a = abs(int(pwm_cmd_hw))
    if a > 255: a = 255
    in1.value(d); in2.value(not d)
    pwm.duty_u16(a * 257)

def stop_all():
    for w in pins: motor_hw(w, 0)

try:
    start_sms()
    for w in counts: counts[w] = 0

    print("\nPPR ONE-REV CALIBRATION")
    print(f"Wheel: {WHEEL}, PWM: {PWM_CMD}")
    print("1) Put a tape mark on the tire.")
    print("2) Spin will start. When the mark is at your reference point, press Enter.")
    print("3) Let it go exactly ONE full revolution. When the mark returns, press Enter again.\n")

    motor_hw(WHEEL, PWM_CMD)
    input("Press Enter at START mark...")
    start = counts[WHEEL]
    input("Press Enter at NEXT pass of the mark...")
    end = counts[WHEEL]
    motor_hw(WHEEL, 0)

    ppr_eff = end - start
    print(f"\nMeasured pulses for one wheel revolution: {ppr_eff}  (effective PULSES_PER_REV for {WHEEL})")
    print("Write this down. Repeat for each wheel.\n")

except KeyboardInterrupt:
    stop_all()
    print("\nStopped.")
