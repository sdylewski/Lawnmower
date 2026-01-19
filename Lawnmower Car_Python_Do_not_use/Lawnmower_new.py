# === MAIN DRIVE with Feed-Forward + chunk_counter PIO ===
from machine import Pin, PWM, time_pulse_us
from rp2 import PIO, StateMachine, asm_pio
from time import sleep, ticks_ms, ticks_diff, sleep_us

# ---------------- Config ----------------
debug = True
debug_encoders_only = False

CPR_MOTOR = 16
GEAR_RATIO = 150
PULSES_PER_REV = CPR_MOTOR * GEAR_RATIO   # 2400
PULSES_PER_REV = 32000 # over-ride with measured values

# !! Use the SAME chunk size you used during feed-forward testing !!
PIO_CHUNK = 64   # TODO: set to 32/64/96/128 per your best run

RPM_MIN_INTERVAL_MS = 100   # faster RPM cadence feels snappier
PRINT_INTERVAL_MS    = 500
ALPHA = 0.2                 # smoothing for RPM + RC display

MAX_WHEEL_RPM = 220.0       # your “expected” curve (for scaling setpoints / FF)
PID_DT_MIN_MS = 20
INTEGRAL_LIMIT = 255.0

# Robot-forward direction per wheel (+1 means +PWM spins robot-forward)
WHEEL_DIR = {
    "FL": -1,
    "FR": +1,
    "BL": -1,
    "BR": +1,
}
# feed-forward values help set target to measured RPMs.
# Feed-Forward parameters from your calibration (examples from your post)
# PWM ≈ c + k·RPM  (per wheel, per direction)   →  FF_pwm = sign(rpm)*| c + k·|rpm|| |
# IMPORTANT: fill in with YOUR latest CSV fit results for the SAME PIO_CHUNK.
#=== Feed-Forward Calibration Summary (Forward direction only) ===
#-- chunk = 64 --
#Wheel | k (PWM/RPM) |  c (PWM) |  R²  | n pts | fit window (PWM)
#   FL |       2.301 | -351.940 | 0.715 |     6 | 75..200
#   FR |       3.337 | -109.428 | 0.961 |     6 | 75..200
#   BL |       3.403 | -443.283 | 0.793 |     6 | 75..200
#   BR |       2.584 | -185.475 | 0.219 |     6 | 75..200

#=== Feed-Forward Calibration Summary (Reverse direction only) ===
#-- chunk = 64 --
#Wheel | k (PWM/RPM) |  c (PWM) |  R²  | n pts | fit window (PWM)
#   FL |      -1.763 | -189.329 | 0.943 |     6 | 75..200
#   FR |      -3.317 |  -69.261 | 0.993 |     6 | 75..200
#   BL |      -5.947 | -593.932 | 0.666 |     6 | 75..200
#   BR |      -2.623 | -313.616 | 0.564 |     6 | 75..200

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


def wheel_max_rpm_from_ff(wheel):
    # Compute FWD and REV max from your linear fits:  PWM = c + k*RPM
    # FWD: at PWM = +255 → RPM = (255 - c)/k
    # REV: at PWM = -255 → RPM = (-255 - c_rev)/k_rev
    fwd = FF_PARAMS[wheel]["FWD"]
    rev = FF_PARAMS[wheel]["REV"]
    # Guard against divide by zero / bad fit
    kf, kr = fwd.get("k", 0.0), rev.get("k", 0.0)
    cf, cr = fwd.get("c", 0.0), rev.get("c", 0.0)

    fwd_max = (255.0 - cf)/kf if abs(kf) > 1e-6 else 0.0
    rev_max = (-255.0 - cr)/kr if abs(kr) > 1e-6 else 0.0  # note kr is negative usually
    # Use a safety margin so PID still has some authority at the top end
    fwd_max *= 0.9
    rev_max *= 0.9
    # Return symmetric bound you can use to clip setpoints
    return max(0.0, min(abs(fwd_max), abs(rev_max)))

WHEEL_MAX_RPM = {w: wheel_max_rpm_from_ff(w) for w in FF_PARAMS}


# PID (use your latest tuned values if different)
PID_params = {
    "FL": {"Kp": 3.259, "Ki": 5.368, "Kd": 0.495},
    "FR": {"Kp": 8.674, "Ki": 13.430, "Kd": 1.400},
    "BL": {"Kp": 4.207, "Ki": 7.480, "Kd": 0.592},
    "BR": {"Kp": 4.248, "Ki": 5.826, "Kd": 0.774},
}

# ---------------- RC Inputs ----------------
CH1 = Pin(21, Pin.IN)  # steering
CH2 = Pin(20, Pin.IN)  # mix
CH3 = Pin(19, Pin.IN)  # throttle
CH4 = Pin(18, Pin.IN)  # yaw/spin
CH5 = Pin(9, Pin.IN)  # spare switch

carLED = Pin(25, Pin.OUT)

# TB6612 STBY (ignore if not present)
#Front_stby = Pin(9, Pin.OUT); # pin 9 moved to switch input, both stby pins are on pin 1 now.
#Front_stby.high()
stby  = Pin(1, Pin.OUT);
stby.high()

# ---------------- Motor Pins ----------------
pins = {
    "BR": (PWM(Pin(2)),  Pin(4, Pin.OUT),  Pin(3, Pin.OUT)),
    "BL": (PWM(Pin(7)),  Pin(5, Pin.OUT),  Pin(6, Pin.OUT)),
    "FL": (PWM(Pin(10)), Pin(12, Pin.OUT), Pin(11, Pin.OUT)),
    "FR": (PWM(Pin(15)), Pin(13, Pin.OUT), Pin(14, Pin.OUT)),
}
for pwm, _, _ in pins.values():
    pwm.freq(20000)  # 20kHz.  TB6612fng can handle 100kHz

def motor_control_robot_forward(wheel, pwm_cmd_rf):
    """pwm_cmd_rf is ROBOT-FORWARD signed PWM in [-255..255]."""
    pwm_cmd_hw = int(pwm_cmd_rf * WHEEL_DIR[wheel])
    pwm, in1, in2 = pins[wheel]
    direction = 1 if pwm_cmd_hw >= 0 else 0
    abs_speed = min(abs(pwm_cmd_hw), 255)
    in1.value(direction)
    in2.value(not direction)
    pwm.duty_u16(int(abs_speed * 65535 / 255))
    # update last HARDWARE command sign when we actually command motion
    if abs_speed > 0:
        last_hw_cmd_sign[wheel] = 1 if pwm_cmd_hw >= 0 else -1

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

encoder_pins = {"FL":17, "FR":28, "BL":8, "BR":0}
encoder_counts = {w: 0 for w in encoder_pins}
sms = {}

def make_irq_handler(wheel):
    def handler(sm):
        encoder_counts[wheel] += PIO_CHUNK
    return handler

def init_sms():
    # stop old
    for sm in sms.values():
        try: sm.active(0)
        except: pass
    sms.clear()
    # zero
    for w in encoder_counts:
        encoder_counts[w] = 0
    # start fresh
    for i, (wheel, pin_num) in enumerate(encoder_pins.items()):
        pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        sm = StateMachine(i, chunk_counter, in_base=pin)
        sm.irq(make_irq_handler(wheel))
        sm.put(PIO_CHUNK)
        sm.active(1)
        sms[wheel] = sm

# ---------------- State / Helpers ----------------
prev_time = ticks_ms()
next_print_ms = prev_time + PRINT_INTERVAL_MS

last_rpm = {w: 0.0 for w in encoder_pins}               # ROBOT-FORWARD signed
prev_counts = {w: 0 for w in encoder_pins}

# NEW: track last HARDWARE cmd sign; alias old name for safety
last_hw_cmd_sign = {w: 1 for w in encoder_pins}
last_cmd_sign = last_hw_cmd_sign  # backward-compat alias if referenced elsewhere

smoothed = {"CH1":0.0, "CH2":0.0, "CH3":0.0, "CH4":0.0}

PID_state = {w: {"integral": 0.0, "prev_err": 0.0, "last_t": prev_time, "out": 0.0} for w in encoder_pins}

def clamp(v, a, b): return a if v < a else b if v > b else v
def deadband(v, band=5): return 0 if abs(v) <= band else v
def pwm_deadband(pwm, band=3): return 0 if -band <= pwm <= band else pwm

def start_pwm_for(wheel, sp_rpm):
    """Minimum PWM to break static friction, per wheel & direction, from |c|."""
    dir_key = "FWD" if sp_rpm >= 0 else "REV"
    c = abs(FF_PARAMS[wheel][dir_key].get("c", 0))
    # Use a fraction of |c| with sane bounds
    # (Fits were 75..200 PWM; |c| is roughly the 'dead-zone' size)
    floor = int(0.8 * c)           # try 0.7..0.9
    if floor < 30:  floor = 30     # don't be too tiny
    if floor > 160: floor = 160    # don't be crazy big
    return floor

def feed_forward_pwm_with_floor(wheel, set_rpm_rf):
    """FF with low-speed floor so we don't dither below stiction."""
    sgn = 1 if set_rpm_rf >= 0 else -1
    dir_key = "FWD" if sgn >= 0 else "REV"
    p = FF_PARAMS.get(wheel, {}).get(dir_key)
    if not p:  # fallback
        return rpm_to_pwm_setpoint(set_rpm_rf)

    k = p.get("k", 0.0); c = p.get("c", 0.0)
    mag = c + k * abs(set_rpm_rf)     # linear FF magnitude
    # Apply a floor near zero: if SP is small, at least push with start PWM
    # Define a low-speed region in RPM (tune 0..20 RPM)
    LOW_RPM = 18.0
    if abs(set_rpm_rf) <= LOW_RPM:
        mag = max(abs(mag), start_pwm_for(wheel, set_rpm_rf))
    u = sgn * mag
    return clamp(int(u), -255, 255)

def read_channel(pin, mn=-100, mx=100):
    while pin.value() == 1:
        sleep_us(100)
    p = time_pulse_us(pin, 1, 30000)
    if p < 1000 or p > 2000:
        return (mn + mx)//2, p
    v = int((p - 1000) * (mx - mn) / 1000 + mn)
    return clamp(v, mn, mx), p

def read_switch(pin):
    p = time_pulse_us(pin, 1, 30000)
    return (p <= 500), p

def rpm_to_pwm_setpoint(rpm):  # rough, only used if FF missing
    s = 1 if rpm >= 0 else -1
    return s * min(int((abs(rpm) / MAX_WHEEL_RPM) * 255), 255)

def feed_forward_pwm(wheel, set_rpm_rf):
    """Use measured (k,c) per wheel per direction. Returns ROBOT-FORWARD signed PWM."""
    sgn = 1 if set_rpm_rf >= 0 else -1
    dir_key = "FWD" if sgn >= 0 else "REV"
    # FIXED: use FF_PARAMS (was FF_params)
    p = FF_PARAMS.get(wheel, {}).get(dir_key)
    if not p:
        # fallback if not measured
        return rpm_to_pwm_setpoint(set_rpm_rf)
    k = p.get("k", 0.0)
    c = p.get("c", 0.0)
    mag = c + k * abs(set_rpm_rf)    # magnitude in PWM units
    # If k≈0 or mag bogus, fall back
    if abs(k) < 1e-6:
        return rpm_to_pwm_setpoint(set_rpm_rf)
    u = sgn * mag
    return clamp(int(u), -255, 255)

def pid_update(wheel, set_rpm_rf, meas_rpm_rf, now_ms):
    st = PID_state[wheel]
    dt_ms = max(1, ticks_diff(now_ms, st["last_t"]))
    if dt_ms < PID_DT_MIN_MS:
        return st["out"]
    dt = dt_ms / 1000.0

    Kp = PID_params[wheel]["Kp"]
    Ki = PID_params[wheel]["Ki"]
    Kd = PID_params[wheel]["Kd"]

    # --- error with deadband ---
    err = set_rpm_rf - meas_rpm_rf
    ERR_DB = 1.0  # RPM; try 1..2
    if -ERR_DB <= err <= ERR_DB:
        err = 0.0

    # --- derivative on measurement (reduces kick) ---
    meas_prev = st.get("meas_prev", meas_rpm_rf)
    dmeas = (meas_rpm_rf - meas_prev) / dt
    st["meas_prev"] = meas_rpm_rf

    # Zero-crossing hysteresis: don't flip sign for tiny commands
    SIGN_DB_RPM = 3.0  # small band
    if abs(set_rpm_rf) < SIGN_DB_RPM:
        # "Prefer" the current motion; if nearly stopped, keep command sign
        if abs(meas_rpm_rf) < SIGN_DB_RPM:
            set_rpm_rf = 0.0   # truly neutral → no chasing

    # When near zero or stuck, freeze I unless we're clearly moving
    moving = abs(meas_rpm_rf) > 3.0

    # --- feed-forward with correct direction block ---
    # Expect FF_PARAMS = {wheel: {"FWD":{k,c}, "REV":{k,c}}}
    sp = set_rpm_rf
    dir_key = "FWD" if sp >= 0 else "REV"
    k = FF_PARAMS[wheel][dir_key]["k"]
    c = FF_PARAMS[wheel][dir_key]["c"]
    # --- feed-forward with low-speed floor ---
    FF = feed_forward_pwm_with_floor(wheel, set_rpm_rf)
    #FF = (c + k*abs(sp)) * (1 if sp >= 0 else -1)

    # --- static friction kick near zero ---
    STICT_RPM = 5.0       # below this we consider "stuck"
    KICK_PWM  = 80        # try 60..100
    KICK_MS   = 120
    kick = 0
    if abs(sp) > 0 and abs(meas_rpm_rf) < STICT_RPM:
        # allow occasional push to break free
        if now_ms - st.get("last_kick_t", 0) > KICK_MS:
            kick = KICK_PWM * (1 if sp >= 0 else -1)
            st["last_kick_t"] = now_ms

    # --- PI with I-zone & conditional integration ---
    # only integrate if moving OR not saturating into the error
    moving = abs(meas_rpm_rf) > 3.0
    uP = Kp * err
    uI = Ki * st["integral"]
    # Reduce D when FF is near saturation (to avoid fighting)
    ff_mag = abs(c + k*abs(sp))
    sat_frac = min(ff_mag/255.0, 1.0)
    Kd_eff = Kd * (1.0 - 0.5*sat_frac)  # cut D up to 50% near top
    uD = -Kd_eff * dmeas

    u_pid = uP + uI + uD

    u_unsat = FF + u_pid + kick
    u = max(-255, min(255, u_unsat))

    pushing = (abs(u) >= 255) and ((u_unsat * err) > 0)
    if Ki > 0 and moving and not pushing and abs(set_rpm_rf) > 0.5:
        st["integral"] += err * dt
        st["integral"] *= 0.995
        lim = INTEGRAL_LIMIT / Ki
        if st["integral"] >  lim: st["integral"] =  lim
        if st["integral"] < -lim: st["integral"] = -lim
    else:
        # bleed off integral when near zero or pushing into saturation
        st["integral"] *= 0.98

    # --- slew limit (rate limit PWM) ---
    prev_u = st.get("out", 0.0)
    # Faster response: big steps when command is non-trivial
    base_slew = 80.0
    if abs(set_rpm_rf) > 0.5 * WHEEL_MAX_RPM.get(wheel, MAX_WHEEL_RPM):
        base_slew = 255.0  # essentially disable slew at high command
    SLEW = base_slew

    du = u - prev_u
    if du > SLEW:  u = prev_u + SLEW
    if du < -SLEW: u = prev_u - SLEW

    st["prev_err"] = err
    st["last_t"]   = now_ms
    st["out"]      = u
    return u


# ---------------- Main ----------------
tires = ["FL", "FR", "BL", "BR"]
init_sms()

Front_stby = Pin(9, Pin.OUT); Front_stby.high()
Back_stby  = Pin(1, Pin.OUT); Back_stby.high()

try:
    while True:
        carLED.toggle(); sleep(0.05)

        # RC raw
        rc1_raw, r1 = read_channel(CH1, -100, 100)
        rc2_raw, r2 = read_channel(CH2, -100, 100)
        rc3_raw, r3 = read_channel(CH3,    0, 155)
        rc4_raw, r4 = read_channel(CH4, -100, 100)
        sw,      r5 = read_switch(CH5)

        # Smooth
        smoothed["CH1"] = (1-ALPHA)*smoothed["CH1"] + ALPHA*rc1_raw
        smoothed["CH2"] = (1-ALPHA)*smoothed["CH2"] + ALPHA*rc2_raw
        smoothed["CH3"] = (1-ALPHA)*smoothed["CH3"] + ALPHA*rc3_raw
        smoothed["CH4"] = (1-ALPHA)*smoothed["CH4"] + ALPHA*rc4_raw

        # Deadband after smoothing (controller inputs)
        rc1 = deadband(int(smoothed["CH1"]), 5)
        rc2 = deadband(int(smoothed["CH2"]), 5)
        rc3 = deadband(int(smoothed["CH3"]), 5)
        rc4 = deadband(int(smoothed["CH4"]), 5)

        now = ticks_ms()
        elapsed = ticks_diff(now, prev_time)
        print_due = now >= next_print_ms

        # --- RPM update (ROBOT-FORWARD signed) ---
        if elapsed >= RPM_MIN_INTERVAL_MS:
            dt = elapsed / 1000.0
            dp_debug = {}
            for w in tires:
                dp = encoder_counts[w] - prev_counts[w]    # unsigned pulses from PIO
                dp_debug[w] = dp

                # Require a minimum number of pulses at low speed for better resolution
                # Relax if time window grows too large.
                min_pulses = 8   # try 8..16 with PIO_CHUNK=16
                if dp < min_pulses and elapsed < 300:  # allow up to 300 ms window
                    continue  # wait for more pulses or next loop

                # sign to ROBOT-FORWARD: last HW command sign x WHEEL_DIR
                dp_signed = dp * last_hw_cmd_sign[w] * WHEEL_DIR[w]
                rps = (dp_signed / PULSES_PER_REV) / dt
                # heavier smoothing at low speed, lighter at high
                alpha = 0.35 if abs(rps*60.0) < 20 else 0.2
                last_rpm[w] = (1-alpha)*last_rpm[w] + alpha*(rps*60.0)
                prev_counts[w] = encoder_counts[w]

            if debug and print_due:
                print("Δpulses:", dp_debug, "dt:", round(dt,3), "PPR:", PULSES_PER_REV)
            prev_time = now


        if debug and debug_encoders_only and print_due:
            print("Measured RPM (RF):", {w: round(last_rpm[w],1) for w in tires})
            next_print_ms = now + PRINT_INTERVAL_MS
            continue

        # ----- Build setpoints in ROBOT-FORWARD coordinates -----
        thr  = rc3 + rc2
        spin = (rc3 < 10 and abs(rc4) > 5)

        if spin:
            spin_pwm = clamp(abs(rc4) * 2, 0, 255)
            left_pwm  =  spin_pwm if rc4 > 0 else -spin_pwm
            right_pwm = -spin_pwm if rc4 > 0 else  spin_pwm
            left_pwm  = pwm_deadband(left_pwm, 3)
            right_pwm = pwm_deadband(right_pwm, 3)
            set_rpm = {
                "FL":  ( left_pwm/255.0)*MAX_WHEEL_RPM,
                "BL":  ( left_pwm/255.0)*MAX_WHEEL_RPM,
                "FR":  (right_pwm/255.0)*MAX_WHEEL_RPM,
                "BR":  (right_pwm/255.0)*MAX_WHEEL_RPM,
            }
            if print_due: print(f"SPIN {'CW' if rc4>0 else 'CCW'} @ {spin_pwm}")
        else:
            left_pwm  = clamp(thr + rc1, -255, 255)
            right_pwm = clamp(thr - rc1, -255, 255)
            left_pwm  = pwm_deadband(left_pwm, 3)
            right_pwm = pwm_deadband(right_pwm, 3)
            set_rpm = {
                "FL":  ( left_pwm/255.0)*MAX_WHEEL_RPM,
                "BL":  ( left_pwm/255.0)*MAX_WHEEL_RPM,
                "FR":  (right_pwm/255.0)*MAX_WHEEL_RPM,
                "BR":  (right_pwm/255.0)*MAX_WHEEL_RPM,
            }
            if print_due: print(f"DRIVE L={left_pwm:.1f} R={right_pwm:.1f}")

        # Clamp setpoints per wheel to measured capability (prevents unreachable targets)
        for w in set_rpm:
            max_rpm = WHEEL_MAX_RPM.get(w, MAX_WHEEL_RPM)
            if set_rpm[w] >  max_rpm: set_rpm[w] =  max_rpm
            if set_rpm[w] < -max_rpm: set_rpm[w] = -max_rpm

        # ----- Neutral / soft brake to prevent creep -----
        ZERO_RPM_SP   = 2.0
        ZERO_RPM_MEAS = 3.0
        SOFT_BRAKE_PWM = 80

        cmds = {}
        for w in tires:
            if abs(set_rpm[w]) < ZERO_RPM_SP:
                st = PID_state[w]
                st["integral"] = 0.0
                st["prev_err"] = 0.0
                if abs(last_rpm[w]) < ZERO_RPM_MEAS:
                    motor_control_robot_forward(w, 0)   # coast
                    cmds[w] = 0
                else:
                    brake = -SOFT_BRAKE_PWM if last_rpm[w] > 0 else SOFT_BRAKE_PWM
                    motor_control_robot_forward(w, brake)
                    cmds[w] = brake
                continue

            # PID+FF in ROBOT-FORWARD coords
            u = pid_update(w, set_rpm[w], last_rpm[w], now)
            motor_control_robot_forward(w, u)
            cmds[w] = u

        # ----- periodic prints -----
        if print_due:
            print("Encoders:", {w: encoder_counts[w] for w in tires})
            print("Set RPM (RF):", {w: int(set_rpm[w]) for w in tires})
            print("Meas RPM (RF):", {w: round(last_rpm[w],1) for w in tires})
            print("PWM Out (RF): ", {w: int(cmds[w]) for w in tires})
            print("----------")
            next_print_ms = now + PRINT_INTERVAL_MS

except KeyboardInterrupt:
    # stop motors on Ctrl-C
    for w in pins:
        motor_control_robot_forward(w, 0)
    print("Stopped.")
