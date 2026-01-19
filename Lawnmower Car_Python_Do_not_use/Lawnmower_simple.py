# === SIMPLE DRIVE (FF + small P) with chunk_counter PIO ===
from machine import Pin, PWM, time_pulse_us
from rp2 import PIO, StateMachine, asm_pio
from time import sleep, ticks_ms, ticks_diff

# ---------------- Config ----------------
# Choose the SAME chunk size you used when you ran the FF tests
PIO_CHUNK = 64

CPR_MOTOR = 16
GEAR_RATIO = 150
PULSES_PER_REV = CPR_MOTOR * GEAR_RATIO   # 2400
PULSES_PER_REV = 32000 # over-ride with measured values

RPM_SAMPLE_MS   = 150
PRINT_INTERVAL_MS = 500
ALPHA_RPM = 0.25            # RPM smoothing (raise to 0.35 for calmer low speed)
MIN_PULSES = 4              # minimum pulses before accepting a sample (improves low-speed accuracy)
MAX_RPM_ABS     = 300.0      # drop obvious glitches
MAX_RPM_STEP    = 200.0      # drop sudden jumps within a short dt
MAX_WHEEL_RPM_HINT = 220.0  # only for scaling or sanity checks

# Robot-forward direction per wheel (+1 means +PWM spins robot forward)
# Run the direction test below and flip signs that are wrong.
WHEEL_DIR = {
    "FL": -1,
    "FR": +1,
    "BL": -1,
    "BR": +1,
}

# ---- Feed-Forward params (REPLACE with your latest fitted values for PIO_CHUNK) ----
# PWM ≈ c + k·RPM  →  FF = sign(rpm) * (c + k*abs(rpm))
# Derived with PIO_CHUNK = 64
FF_PARAMS = {
    "FL": {
        "FWD": {"k": 44.521, "c": 73.973},
        "REV": {"k": 49.296, "c": 197.711},
    },
    "FR": {
        "FWD": {"k": 57.491, "c": 151.394},
        "REV": {"k": 55.051, "c": 150.142},
    },
    "BL": {
        "FWD": {"k": 52.036, "c": 255.373},
        "REV": {"k": 48.069, "c": 128.081},
    },
    "BR": {
        "FWD": {"k": 48.731, "c": 129.300},
        "REV": {"k": 53.592, "c": 252.385},
    },
}

# Small proportional trim on top of feed-forward (keeps it steady without jerks)
P_TRIM = {"FL": 0.2, "FR": 0.2, "BL": 0.2, "BR": 0.2}


# Target RPM for the simple hold test
TARGET_RPM = 40.0  # change this and re-run

# ---------------- Motor pins ----------------
pins = {
    "BR": (PWM(Pin(2)),  Pin(4, Pin.OUT),  Pin(3, Pin.OUT)),
    "BL": (PWM(Pin(7)),  Pin(5, Pin.OUT),  Pin(6, Pin.OUT)),
    "FR": (PWM(Pin(10)), Pin(12, Pin.OUT), Pin(11, Pin.OUT)),
    "FL": (PWM(Pin(15)), Pin(13, Pin.OUT), Pin(14, Pin.OUT)),
}
for pwm, _, _ in pins.values():
    pwm.freq(20000)  # quiet PWM

encoder_pins = { "FL":17, "FR":28, "BL":8, "BR":0}
encoder_counts = {w: 0 for w in encoder_pins}

# track the last *hardware* sign we actually drove (+1/-1)
last_hw_sign = {w: 1 for w in ["FL","FR","BL","BR"]}

def motor_hw(wheel, pwm_cmd_hw):
    pwm, in1, in2 = pins[wheel]
    d = 1 if pwm_cmd_hw >= 0 else 0
    a = abs(int(pwm_cmd_hw))
    if a > 255: a = 255
    in1.value(d)
    in2.value(not d)
    pwm.duty_u16(a * 257)
    if a > 0:
        last_hw_sign[wheel] = 1 if pwm_cmd_hw >= 0 else -1

# globals
last_hw_sign = {w: +1 for w in encoder_pins}  # +1/-1 last hardware direction

def motor_rf(wheel, pwm_cmd_rf):
    # Robot-forward -> hardware
    pwm_cmd_hw = int(pwm_cmd_rf * WHEEL_DIR[wheel])
    pwm, in1, in2 = pins[wheel]
    d = 1 if pwm_cmd_hw >= 0 else 0
    a = abs(pwm_cmd_hw)
    if a > 255: a = 255
    in1.value(d); in2.value(not d)
    pwm.duty_u16(a * 257)
    if a > 0:
        # remember last hardware spin sign for this wheel
        last_hw_sign[wheel] = +1 if pwm_cmd_hw >= 0 else -1


def stop_all():
    for w in pins:
        motor_rf(w, 0)

# ---------------- PIO: chunked edge counter ----------------
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


sms = {}

def make_irq_handler(wheel):
    def handler(sm):
        encoder_counts[wheel] += PIO_CHUNK
    return handler

def init_sms():
    for sm in sms.values():
        try: sm.active(0)
        except: pass
    sms.clear()
    for w in encoder_counts: encoder_counts[w] = 0
    for i, (wheel, pin_num) in enumerate(encoder_pins.items()):
        pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        sm = StateMachine(i, chunk_counter, in_base=pin)
        sm.irq(make_irq_handler(wheel))
        sm.put(PIO_CHUNK)
        sm.active(1)
        sms[wheel] = sm

# ---------------- RPM measurement ----------------
last_rpm = {w: 0.0 for w in encoder_pins}
prev_counts = {w: 0 for w in encoder_pins}
prev_t = ticks_ms()
next_print = prev_t + PRINT_INTERVAL_MS

def update_rpm():
    global prev_t
    now = ticks_ms()
    el = ticks_diff(now, prev_t)
    if el < RPM_SAMPLE_MS:
        return False
    dt = el / 1000.0
    for w in encoder_pins:
        dp = encoder_counts[w] - prev_counts[w]
        if dp < MIN_PULSES and el < 300:
            continue  # wait more time for accuracy at low speed
        rps = (dp / PULSES_PER_REV) / dt
        rpm = rps * 60.0
        # RPM in ROBOT-FORWARD sign = hardware spin sign (we command sign) * WHEEL_DIR[w]
        # We don't have quadrature, so infer sign from commanded PWM we are sending.
        # We'll keep a small history of what we sent:
    prev_t = now
    return True

# Track last commanded ROBOT-FORWARD PWM to infer RPM sign
last_cmd_rf = {w: 0 for w in encoder_pins}


def read_rpm_signed(now, dt_s):
    out = {}
    for w in encoder_pins:
        dp = encoder_counts[w] - prev_counts[w]
        # at very low speeds accept longer window; otherwise require some pulses
        if dp < MIN_PULSES and dt_s < 0.30:
            out[w] = last_rpm[w]
            continue

        rps = (dp / PULSES_PER_REV) / dt_s
        rpm_uns = rps * 60.0

        # sign into ROBOT-FORWARD frame:
        sgn_rf = last_hw_sign[w] * WHEEL_DIR[w]
        rpm = rpm_uns * sgn_rf

        # reject obvious glitches (e.g., your FR 566rpm spike)
        if abs(rpm) > MAX_RPM_ABS:
            # discard this interval so dp won't accumulate
            prev_counts[w] = encoder_counts[w]
            out[w] = last_rpm[w]
            continue

        # reject sudden jumps within short windows
        if abs(rpm - last_rpm[w]) > MAX_RPM_STEP and dt_s < 0.35:
            # discard this interval so dp won't accumulate
            prev_counts[w] = encoder_counts[w]
            out[w] = last_rpm[w]
            continue

        # accept & smooth
        last_rpm[w] = (1 - ALPHA_RPM) * last_rpm[w] + ALPHA_RPM * rpm
        prev_counts[w] = encoder_counts[w]
        out[w] = last_rpm[w]
    return out


# ---------------- Feed-forward helpers ----------------
def start_pwm_for(wheel, sp_rpm):
    """Minimum PWM to break static friction, from |c| with bounds."""
    dir_key = "FWD" if sp_rpm >= 0 else "REV"
    c = abs(FF_PARAMS[wheel][dir_key].get("c", 0))
    floor = int(0.8 * c)  # try 0.7..0.9
    if floor < 30:  floor = 30
    if floor > 160: floor = 160
    return floor

FF_GAIN = 0.8  # try 0.7..0.9

FF_GAIN = 0.8  # 0.7..0.9

def ff_pwm(wheel, sp_rpm):
    sgn = 1 if sp_rpm >= 0 else -1
    dir_key = "FWD" if sgn >= 0 else "REV"
    p = FF_PARAMS.get(wheel, {}).get(dir_key)
    if not p:
        u = (abs(sp_rpm)/MAX_WHEEL_RPM_HINT)*255.0
        return int(sgn * max(0, min(255, u)))
    k = p["k"]; c = p["c"]
    mag = max(0.0, c + k * abs(sp_rpm))
    if abs(sp_rpm) <= 18.0:
        mag = max(mag, 30)  # tiny floor
    u = int(sgn * FF_GAIN * mag)
    if u > 255: u = 255
    if u < -255: u = -255
    return u


# ---------------- Direction check ----------------
def direction_check():
    print("\n=== Direction Check (visual) ===")
    print("Each wheel will spin ROBOT-FORWARD for ~2s.")
    print("If a wheel spins backward, flip its sign in WHEEL_DIR[...] and rerun.\n")

    test_rpm  = 40.0
    dwell_ms  = 5000

    for w in ["FL","FR","BL","BR"]:
        # hard reset counts just for clarity
        for ww in encoder_counts: encoder_counts[ww] = 0
        for ww in prev_counts:    prev_counts[ww]    = 0
        last_rpm[w] = 0.0

        u = ff_pwm(w, +test_rpm)   # FF forward
        motor_rf(w, u)
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < dwell_ms:
            pass
        motor_rf(w, 0)

        dp = encoder_counts[w]
        rpm = (dp / PULSES_PER_REV) / (dwell_ms/1000.0) * 60.0
        print(f"{w}: FF={u:+4d}  measured ≈ {rpm:5.1f} RPM  (VISUAL: should move forward!)\n")
        sleep(0.5)


# ---------------- Simple hold loop (FF + small P) ----------------
def hold_constant_rpm(target_rpm=TARGET_RPM, seconds=12):
    print(f"\n=== Holding all wheels at {target_rpm:.1f} RPM (FF + small P only) ===")
    global prev_t, next_print
    prev_t = ticks_ms()
    next_print = prev_t + PRINT_INTERVAL_MS
    for w in last_rpm: last_rpm[w] = 0.0
    for w in prev_counts: prev_counts[w] = encoder_counts[w]

    t0 = ticks_ms()
    while ticks_diff(ticks_ms(), t0) < int(seconds*1000):
        now = ticks_ms()
        el = ticks_diff(now, prev_t)
        if el >= RPM_SAMPLE_MS:
            dt = el / 1000.0
            # compute RPM (signed by last command)
            rpms = read_rpm_signed(now, dt)
            if now >= next_print:
                dp_debug = {w: encoder_counts[w] - prev_counts[w] for w in encoder_pins}
                print("DP since last accepted:", dp_debug)

            prev_t = now

            # command each wheel
            cmds = {}
            for w in ["FL","FR","BL","BR"]:
                sp = target_rpm
                # FF component
                u_ff = ff_pwm(w, sp)
                # small P trim
                err = sp - rpms.get(w, 0.0)
                u = u_ff + P_TRIM[w]*err
                # clamp & send
                if u > 255: u = 255
                if u < -255: u = -255
                last_cmd_rf[w] = u
                motor_rf(w, u)
                cmds[w] = int(u)

            if now >= next_print:
                print(f"SP={int(target_rpm)}  RPM:", {w: round(last_rpm[w],1) for w in last_rpm})
                print("PWM:", cmds)
                print("----")
                next_print = now + PRINT_INTERVAL_MS

        sleep(0.01)

    stop_all()
    print("Hold test finished.\n")

# ---------------- Run ----------------
try:
    init_sms()

    # Optional: enable TB6612 standbys if you have them
    try:
        Pin(9, Pin.OUT).high()
        Pin(1, Pin.OUT).high()
    except:
        pass

    # 1) Direction check (visual)
    direction_check()

    # 2) Simple constant-RPM hold using FF + small P
    hold_constant_rpm(TARGET_RPM, seconds=12)

except KeyboardInterrupt:
    stop_all()
    print("Stopped.")
