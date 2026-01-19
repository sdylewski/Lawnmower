# === SPIN ONE WHEEL — PIO X-register edge counter + debounce (no IRQs/chunking) ===
from machine import Pin, PWM
from rp2 import PIO, StateMachine, asm_pio
from time import sleep, ticks_ms, ticks_diff

# ---------------- USER CONFIG ----------------
WHEEL_TO_TEST = "BR"     # "FL", "FR", "BL", or "BR"
TARGET_RPM    = 60.0
TEST_SECONDS  = 10
PRINT_MS      = 500

# PIO counter / debounce
PIO_SM_FREQ   = 125_000_000   # PIO clock (1 instr / cycle)
DEBOUNCE_US   = 100            # post-edge holdoff in microseconds

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

# Encoder GPIOs (adjust to your wiring) — single channel per wheel
encoder_pins = {"FL":17, "FR":28, "BL":8, "BR":0}

# ---------------- PIO: rising-edge counter into X + post-edge debounce ----------------
@asm_pio(autopush=True, push_thresh=32)   # IN X,32 auto-pushes into RX FIFO
def edge_count_x1_single_pin_db():
    # pull deb_cycles (in PIO cycles) once; keep it in OSR; Y is working countdown
    pull()                      # OSR = deb_cycles
    mov(y, osr)                 # Y   = deb_cycles (init; we'll reload from OSR each edge)

    wrap_target()
    wait(0, pin, 0)             # ensure LOW first (arm for a true rising edge)
    label("loop")
    wait(1, pin, 0)             # TRIGGER: rising edge on the monitored pin

    # X++ (standard idiom: ~X; dec; ~X)
    mov(x, invert(x))
    jmp(x_dec, "inc_done")
    label("inc_done")
    mov(x, invert(x))

    # Post-edge holdoff: burn 'deb_cycles' PIO cycles (ignore chatter/extra edges)
    mov(y, osr)                 # reload Y = deb_cycles
    label("deb")
    jmp(y_dec, "deb")

    # Re-arm: require LOW again before the next rising edge
    wait(0, pin, 0)
    jmp("loop")
    wrap()

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

# ---------------- PIO helpers (single wheel) ----------------
def i32(n):
    """Convert 32-bit unsigned to signed (wrap-around safe)."""
    return (n + 0x80000000) % 0x100000000 - 0x80000000

def init_sm_for(wheel, debounce_us):
    """Create one SM on the wheel’s encoder pin, preload deb_cycles, and start it."""
    pin_num = encoder_pins[wheel]
    pin     = Pin(pin_num, Pin.IN, Pin.PULL_UP)
    deb_cycles = max(1, int(debounce_us * (PIO_SM_FREQ // 1_000_000)))  # µs -> PIO cycles
    sm = StateMachine(0, edge_count_x1_single_pin_db, freq=PIO_SM_FREQ, in_base=pin)
    sm.put(deb_cycles)           # satisfies the 'pull()' at program start
    sm.active(1)
    return sm

def read_x(sm):
    """Sample X via IN X,32 -> RX FIFO; returns 32-bit unsigned X."""
    sm.exec("in_(x, 32)")
    return sm.get()

# ---------------- Test loop ----------------
def run_test():
    sm = init_sm_for(WHEEL_TO_TEST, DEBOUNCE_US)

    # baseline X and software counters
    x_last          = read_x(sm)
    encoder_count   = 0     # software total edges
    prev_count      = 0
    prev_t          = ticks_ms()
    last_rpm        = 0.0

    # print-state (no function attributes)
    prev_count_print = 0
    prev_t_print     = ticks_ms()

    print(f"\nTesting {WHEEL_TO_TEST} (open-loop @ PWM={CAL_PWM}) for {TEST_SECONDS}s\n")
    print("Time(s) | Counts | MeasRPM | PWM")
    print("------------------------------------")

    t0 = ticks_ms()
    next_print = t0

    while ticks_diff(ticks_ms(), t0) < TEST_SECONDS * 1000:
        now  = ticks_ms()

        # Pull fresh X and update software counter
        x_cur = read_x(sm)
        dx    = i32(x_cur - x_last)
        if dx < 0:
            dx &= 0xFFFFFFFF
        encoder_count += dx
        x_last = x_cur

        # RPM from counted pulses over loop Δt
        dt_s = (ticks_diff(now, prev_t)) / 1000.0
        if dt_s <= 0: dt_s = 1e-6
        dp   = encoder_count - prev_count
        rpm  = (dp / PULSES_PER_REV) / dt_s * 60.0
        last_rpm   = 0.75 * last_rpm + 0.25 * rpm
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

            # frequency estimate (edges/s) over the print window
            dc   = encoder_count - prev_count_print
            dts  = (now - prev_t_print) / 1000.0
            cps  = dc / dts if dts > 0 else 0.0
            print("freq≈%.1f Hz  (rising, post-edge debounce=%dus)" % (cps, DEBOUNCE_US))

            prev_count_print = encoder_count
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
