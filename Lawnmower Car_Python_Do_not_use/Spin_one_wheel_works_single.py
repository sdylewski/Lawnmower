# === SPIN ONE WHEEL — rising-edge debounced PIO (no chunking) ===
from machine import Pin, PWM
from rp2 import PIO, StateMachine, asm_pio
from time import sleep, ticks_ms, ticks_diff

# ---------------- USER CONFIG ----------------
WHEEL_TO_TEST = "BR"     # "FL", "FR", "BL", or "BR"
TARGET_RPM    = 60.0
TEST_SECONDS  = 10
PRINT_MS      = 500

# Counter mode (works best vs your scope):
DEBOUNCE_US   = 50      # post-edge holdoff (try 75..150 µs)
SM_FREQ_HZ    = 1_000_000  # PIO clock so 1 instr ≈ 1 µs

# Drive mode:
#   "fixed_pwm" -> open-loop constant PWM (best for frequency/PPR checks)
#   "ff_hold"   -> feed-forward + small P trim to hold TARGET_RPM
CONTROL_MODE  = "fixed_pwm"
CAL_PWM       = 140

# Encoder/motor parameters
CPR_MOTOR      = 16
GEAR_RATIO     = 150
PULSES_PER_REV = CPR_MOTOR * GEAR_RATIO  # 2400 pulses/wheel rev

# ---------------- Hardware pins ----------------
# Motor pins: (PWM, IN1, IN2)
pins = {
    "BR": (PWM(Pin(2)),  Pin(4, Pin.OUT),  Pin(3, Pin.OUT)),
    "BL": (PWM(Pin(7)),  Pin(5, Pin.OUT),  Pin(6, Pin.OUT)),
    "FR": (PWM(Pin(10)), Pin(12, Pin.OUT), Pin(11, Pin.OUT)),
    "FL": (PWM(Pin(15)), Pin(13, Pin.OUT), Pin(14, Pin.OUT)),
}
for pwm, _, _ in pins.values():
    pwm.freq(20000)

# Optional motor driver STBY
try:
    Pin(1, Pin.OUT).high()
except:
    pass

# Encoder GPIOs (adjust to your wiring)
encoder_pins = {"FL":17, "FR":28, "BL":8, "BR":0}

# ---------------- PIO: rising-edge with debounce ----------------
@asm_pio()
def edge_counter_rising_db():
    pull()                  # OSR = debounce_cycles
    mov(y, osr)             # Y = debounce_cycles (constant)
    mov(x, y)               # X = working copy
    wrap_target()
    wait(0, pin, 0)         # be sure we're low before arming
    wait(1, pin, 0)         # rising edge -> count
    irq(rel(0))
    label("hold")
    jmp(x_dec, "hold")      # post-edge holdoff
    mov(x, y)               # reload holdoff
    wrap()

# ---------------- Globals (single wheel) ----------------
encoder_count = 0
irq_count     = 0

def make_irq_handler():
    def handler(sm):
        # one interrupt per debounced rising edge
        global irq_count, encoder_count
        irq_count     += 1
        encoder_count += 1
    return handler

def init_sm_for(wheel):
    gpio = encoder_pins[wheel]
    pin  = Pin(gpio, Pin.IN, Pin.PULL_UP)
    sm   = StateMachine(0, edge_counter_rising_db,
                        freq=SM_FREQ_HZ, in_base=pin, jmp_pin=pin)
    sm.irq(make_irq_handler())
    debounce_cycles = max(1, int(DEBOUNCE_US * (SM_FREQ_HZ // 1_000_000)))
    sm.put(debounce_cycles)     # preload debounce parameter
    sm.active(1)
    return sm

# ---------------- Motor & (optional) FF ----------------
def motor_hw(wheel, pwm_cmd_hw):
    pwm, in1, in2 = pins[wheel]
    d = 1 if pwm_cmd_hw >= 0 else 0
    a = abs(int(pwm_cmd_hw))
    if a > 255: a = 255
    in1.value(d); in2.value(not d)
    pwm.duty_u16(a * 257)

# Minimal FF (used only if CONTROL_MODE == "ff_hold")
FF_PARAMS = {
    "FL": {"FWD": {"k": 2.881, "c": 462.711}, "REV": {"k": 1.600, "c": 157.380}},
    "FR": {"FWD": {"k": 3.460, "c": 118.527}, "REV": {"k": 3.246, "c": 63.760}},
    "BL": {"FWD": {"k": 3.241, "c": 413.426}, "REV": {"k": 1.250, "c": 37.500}},
    "BR": {"FWD": {"k": 6.011, "c": 529.771}, "REV": {"k": 3.133, "c": 391.936}},
}
FF_GAIN = 0.8
P_TRIM  = 0.2

def ff_pwm(wheel, sp_rpm):
    sgn = 1 if sp_rpm >= 0 else -1
    p   = FF_PARAMS[wheel]["FWD" if sgn >= 0 else "REV"]
    mag = max(0.0, p["c"] + p["k"] * abs(sp_rpm))
    if abs(sp_rpm) <= 18.0:
        mag = max(mag, 30)
    u = int(sgn * FF_GAIN * mag)
    return 255 if u > 255 else -255 if u < -255 else u

# ---------------- Test loop ----------------
def run_test():
    global encoder_count, irq_count
    sm = init_sm_for(WHEEL_TO_TEST)

    encoder_count = 0
    irq_count     = 0
    prev_count    = 0
    prev_t        = ticks_ms()
    last_rpm      = 0.0

    # print-state (no function attributes)
    prev_count_print = 0
    prev_irqs_print  = 0
    prev_t_print     = ticks_ms()

    print(f"\nTesting {WHEEL_TO_TEST} (open-loop @ PWM={CAL_PWM}) for {TEST_SECONDS}s\n")
    print("Time(s) | Counts | MeasRPM | PWM")
    print("------------------------------------")

    t0 = ticks_ms()
    next_print = t0

    while ticks_diff(ticks_ms(), t0) < TEST_SECONDS * 1000:
        now  = ticks_ms()
        dt_s = (ticks_diff(now, prev_t)) / 1000.0
        if dt_s <= 0: dt_s = 1e-6

        # RPM from counted pulses
        dp   = encoder_count - prev_count
        rpm  = (dp / PULSES_PER_REV) / dt_s * 60.0
        last_rpm = 0.75 * last_rpm + 0.25 * rpm
        prev_count = encoder_count
        prev_t     = now

        # Command
        if CONTROL_MODE == "fixed_pwm":
            pwm_cmd = CAL_PWM
        else:
            err = TARGET_RPM - last_rpm
            pwm_cmd = ff_pwm(WHEEL_TO_TEST, TARGET_RPM) + P_TRIM * err
            if pwm_cmd > 255: pwm_cmd = 255
            if pwm_cmd < -255: pwm_cmd = -255

        motor_hw(WHEEL_TO_TEST, pwm_cmd)

        # Print interval
        if now >= next_print:
            elapsed_s = (now - t0) / 1000.0
            print(f"{elapsed_s:7.2f} | {encoder_count:6d} | {last_rpm:7.1f} | {int(pwm_cmd):3d}")

            dc   = encoder_count - prev_count_print
            di   = irq_count     - prev_irqs_print
            dts  = (now - prev_t_print) / 1000.0
            if dts <= 0: dts = 1e-6
            cps  = dc / dts
            irqs = di / dts
            ratio = (cps / irqs) if irqs > 0 else 0.0
            print("freq≈%.1f Hz  |  irqs≈%.1f/s  |  cps≈%.0f  |  cps/irq≈%.2f  (rising, debounce=%dus)"
                  % (cps, irqs, cps, ratio, DEBOUNCE_US))

            prev_count_print = encoder_count
            prev_irqs_print  = irq_count
            prev_t_print     = now
            next_print      += PRINT_MS

        sleep(0.01)

    motor_hw(WHEEL_TO_TEST, 0)
    sm.active(0)
    print("\nTest finished.\n")

# ---------------- RUN ----------------
try:
    run_test()
except KeyboardInterrupt:
    motor_hw(WHEEL_TO_TEST, 0)
    print("\nStopped.")
