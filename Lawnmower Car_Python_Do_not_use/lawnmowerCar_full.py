from machine import Pin, PWM, time_pulse_us
from rp2 import PIO, StateMachine, asm_pio
from time import sleep, ticks_ms, ticks_diff, sleep_us
from math import pi

"""
RC CAR PID CONTROL — FULL PROGRAM (with robust calibration)
- RC input reading & smoothing
- PIO encoder counting (IRQ per pulse)
- Closed-loop PID speed control per wheel
- Relay-based ZN autotune per wheel (memory-safe, robust start, pause between wheels)
- Left wheels inverted in drive block (as requested)
- Hearty inline comments
"""

# === USER/BUILD SETTINGS ===
debug = True                               # master debug flag
debug_encoders_only = False                # only encoder prints when True
calibration_mode = True                   # set True to run PID autotune per wheel

CPR_MOTOR = 16                             # encoder counts per motor rev (1 ch, 1 edge)
GEAR_RATIO = 150                           # gearbox ratio
PULSES_PER_REV = CPR_MOTOR * GEAR_RATIO    # pulses per output shaft revolution

RPM_MIN_INTERVAL_MS = 250                  # cadence for RPM math (sampling window)
PRINT_INTERVAL_MS = 500                    # cadence for debug prints
ALPHA = 0.2                                # smoothing for inputs/RPM display
PID_DT_MIN_MS = 20                         # minimum dt for PID updates (limit jitter)

MAX_WHEEL_RPM = 120.0                      # estimated max wheel RPM at full PWM

# === ZN CALIBRATION SETTINGS ===
CALIBRATION_TARGET_RPM = 30.0              # default RPM target during calibration
RELAY_PWM = 100                             # on/off PWM magnitude (0..255)
DURATION_MS = 7000                          # per-wheel calibration window
NO_MOTION_TIMEOUT_MS = 1500                 # abort if no new pulses for this long
SWITCH_PAUSE_S = 3                          # pause between wheels during calibration
SAMPLE_MS = 250                              # cadence for RPM calc (calibration helper)
WARMUP_MS = 1500                             # ignore early part when computing min/max
ALPHA_CAL = 0.25                             # EMA smoothing for calibration RPM

# === Initial PID parameters (can be overwritten by autotune) ===
PID_params = {
    "FL": {"Kp": 9.674, "Ki": 21.83, "Kd": 1.072},
    "FR": {"Kp": 7.639, "Ki": 30.25, "Kd": 0.48},
    "BL": {"Kp": 5.602, "Ki": 17.78, "Kd": 0.44},
    "BR": {"Kp": 4.443, "Ki": 17.63, "Kd": 0.28},
}
PID_params = {
    "FL": {"Kp": 1, "Ki": 0, "Kd": 0},
    "FR": {"Kp": 1, "Ki": 0, "Kd": 0},
    "BL": {"Kp": 1, "Ki": 0, "Kd": 0},
    "BR": {"Kp": 1, "Ki": 0, "Kd": 0},
}

# === RC Inputs ===
CH1 = Pin(21, Pin.IN)
CH2 = Pin(20, Pin.IN)
CH3 = Pin(19, Pin.IN)
CH4 = Pin(18, Pin.IN)
CH5 = Pin(16, Pin.IN)

carLED = Pin(25, Pin.OUT)

# === Motor outputs mapping (TB6612-style) ===
pins = {
    "BR": (PWM(Pin(2)),  Pin(4, Pin.OUT),  Pin(3, Pin.OUT)),
    "BL": (PWM(Pin(7)),  Pin(5, Pin.OUT),  Pin(6, Pin.OUT)),
    "FL": (PWM(Pin(10)), Pin(12, Pin.OUT), Pin(11, Pin.OUT)),
    "FR": (PWM(Pin(15)), Pin(13, Pin.OUT), Pin(14, Pin.OUT)),
}
for pwm, in1, in2 in pins.values():
    pwm.freq(1000)

def motor_control(wheel, pwm_cmd):
    """Drive one wheel with signed PWM in [-255..255]."""
    pwm, in1, in2 = pins[wheel]
    direction = 1 if pwm_cmd >= 0 else 0
    abs_speed = min(abs(int(pwm_cmd)), 255)
    in1.value(direction)
    in2.value(not direction)
    pwm.duty_u16(int(abs_speed * 65535 / 255))

# Optional STBY pins
try:
    Front_stby = Pin(9, Pin.OUT); Front_stby.high()
    Back_stby  = Pin(1, Pin.OUT); Back_stby.high()
except Exception:
    pass

# === PIO PROGRAM: rising-edge counter (IRQ per pulse) ===
@asm_pio()
def edge_counter():
    label("edge_counter")
    wait(1, pin, 0)
    wait(0, pin, 0)
    irq(rel(0))
    jmp("edge_counter")
    wrap()

# === ENCODER INPUT PINS ===
encoder_pins = {
    "FR": 17,
    "FL": 28,
    "BL": 8,
    "BR": 0
}

encoder_counts = {wheel: 0 for wheel in encoder_pins}

# === STATE MACHINES SETUP ===
sms = {}

def make_irq_handler(wheel):
    def handler(sm):
        encoder_counts[wheel] += 1  # increment by 1 per pulse
    return handler

for i, (wheel, pin_num) in enumerate(encoder_pins.items()):
    pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)  # internal pull-up; use external if needed
    sm = StateMachine(i, edge_counter, in_base=pin, jmp_pin=pin)
    sm.irq(make_irq_handler(wheel))
    sm.active(1)
    sms[wheel] = sm

# === RPM calculation and input smoothing (drive loop) ===
prev_counts = encoder_counts.copy()        # snapshot of counts at last RPM update
prev_time = ticks_ms()                     # timestamp of last RPM update
last_rpm = {w: 0.0 for w in encoder_pins}  # exponentially-smoothed RPM per wheel
smoothed = {"CH1":0, "CH2":0, "CH3":0, "CH4":0}  # smoothed RC inputs

# Print cadence timer (separate from RPM cadence)
next_print_ms = prev_time + PRINT_INTERVAL_MS

# === PID state ===
PID_state = {w: {"integral": 0.0, "prev_err": 0.0, "last_t": prev_time, "out": 0.0} for w in encoder_pins}
INTEGRAL_LIMIT = 255.0  # bound for integral anti-windup in PWM units

# === RC helpers ===
def read_channel(pin, mn=-100, mx=100):
    """Return integer in [mn..mx] mapped from 1000..2000 µs pulse width.
    Also return the raw pulse width for debugging.
    """
    while pin.value() == 1:
        sleep_us(100)
    p = time_pulse_us(pin, 1, 30000)
    if p < 1000 or p > 2000:
        return (mn + mx)//2, p
    v = int((p - 1000) * (mx - mn) / 1000 + mn)
    return max(min(v, mx), mn), p

def read_switch(pin):
    p = time_pulse_us(pin, 1, 30000)
    return (p >= 1500), p

# Map desired signed RPM to signed PWM target (open-loop helper)
def rpm_to_pwm_setpoint(rpm):
    if rpm >= 0:
        return min(int((rpm / MAX_WHEEL_RPM) * 255), 255)
    else:
        return -min(int(((-rpm) / MAX_WHEEL_RPM) * 255), 255)

# === PID update ===
def pid_update(wheel, set_rpm, meas_rpm, now_ms):
    """Velocity-form PID: returns PWM command in [-255..255].
    Runs at most every PID_DT_MIN_MS to avoid jitter and CPU load.
    """
    st = PID_state[wheel]
    dt_ms = max(1, ticks_diff(now_ms, st["last_t"]))
    if dt_ms < PID_DT_MIN_MS:
        return st["out"]  # skip ultra-fast updates
    dt = dt_ms / 1000.0
    err = set_rpm - meas_rpm
    Kp = PID_params[wheel]["Kp"]
    Ki = PID_params[wheel]["Ki"]
    Kd = PID_params[wheel]["Kd"]
    st["integral"] += err * dt
    # Anti-windup clamp (avoid division by zero if Ki==0)
    if Ki != 0:
        max_int = INTEGRAL_LIMIT / abs(Ki)
        if st["integral"] > max_int: st["integral"] = max_int
        if st["integral"] < -max_int: st["integral"] = -max_int
    deriv = (err - st["prev_err"]) / dt
    out = Kp*err + Ki*st["integral"] + Kd*deriv
    # clamp output to PWM range
    if out > 255: out = 255
    if out < -255: out = -255
    st["prev_err"] = err
    st["last_t"] = now_ms
    st["out"] = out
    return out

# === Ziegler–Nichols (relay autotune describing function) ===
def zn_from_relay(relay_amp_pwm, time_series):
    """Compute Ku, Tu from sustained oscillation data and return suggested PID gains.
    time_series: list of (t_ms, rpm) over a window where relay control induced oscillations.
    """
    if len(time_series) < 10:
        return None
    # Find local maxima (simple peak pick)
    peaks = []
    for i in range(1, len(time_series)-1):
        if time_series[i-1][1] < time_series[i][1] > time_series[i+1][1]:
            peaks.append(time_series[i])
    if len(peaks) < 2:
        return None
    periods = [peaks[i+1][0] - peaks[i][0] for i in range(len(peaks)-1)]
    Tu_ms = sum(periods) / len(periods)
    rpms = [r for _, r in time_series]
    a = (max(rpms) - min(rpms)) / 2.0
    if a <= 0:
        return None
    d = float(abs(relay_amp_pwm))
    Ku = (4.0 * d) / (pi * a)
    Tu = Tu_ms / 1000.0
    return {"Ku": Ku, "Tu": Tu,
            "Kp": 0.6 * Ku,
            "Ki": 1.2 * Ku / Tu,
            "Kd": 0.075 * Ku * Tu}

# === Calibration helpers (memory-safe) ===
rpm_filt_cal = {w: 0.0 for w in encoder_pins}

def cal_update_rpm(wheel, last_counts, last_ms):
    """Compute RPM at SAMPLE_MS cadence with EMA smoothing for calibration.
    Returns (rpm_filtered, new_last_counts, new_last_ms, dp).
    """
    now = ticks_ms()
    if ticks_diff(now, last_ms) < SAMPLE_MS:
        return None, last_counts, last_ms, 0
    dt_s = ticks_diff(now, last_ms) / 1000.0
    dp = encoder_counts[wheel] - last_counts
    inst_rpm = (dp / PULSES_PER_REV) / dt_s * 60.0
    rf = rpm_filt_cal[wheel]
    rf = (1-ALPHA_CAL)*rf + ALPHA_CAL*inst_rpm
    rpm_filt_cal[wheel] = rf
    return rf, encoder_counts[wheel], now, dp

# === Helper to pause between wheels ===
def pause_between_wheels():
    print(f"Pausing {SWITCH_PAUSE_S} seconds before next wheel...")
    sleep(SWITCH_PAUSE_S)

# === Main loop ===
tires = ["FL", "FR", "BL", "BR"]

try:
    while True:
        carLED.toggle(); sleep(0.05)  # Heartbeat LED

        # --- Read RC and smooth ---
        rc1, r1 = read_channel(CH1, -100, 100)  # steering
        rc2, r2 = read_channel(CH2, -100, 100)  # extra mixing
        rc3, r3 = read_channel(CH3, 0, 155)     # throttle
        rc4, r4 = read_channel(CH4, -100, 100)  # spin
        sw,  r5 = read_switch(CH5)              # mode switch
        smoothed["CH1"] = smoothed["CH1"]*(1-ALPHA) + rc1*ALPHA
        smoothed["CH2"] = smoothed["CH2"]*(1-ALPHA) + rc2*ALPHA
        smoothed["CH3"] = smoothed["CH3"]*(1-ALPHA) + rc3*ALPHA
        smoothed["CH4"] = smoothed["CH4"]*(1-ALPHA) + rc4*ALPHA

        now = ticks_ms()
        elapsed = ticks_diff(now, prev_time)
        print_due = now >= next_print_ms

        # --- RPM update at cadence (for drive loop) ---
        if elapsed >= RPM_MIN_INTERVAL_MS:
            dt = elapsed / 1000.0
            dp_debug = {}
            for w in tires:
                dp = encoder_counts[w] - prev_counts[w]
                dp_debug[w] = dp
                rps = dp / PULSES_PER_REV / dt
                last_rpm[w] = (1-ALPHA)*last_rpm[w] + ALPHA*(rps*60.0)
                prev_counts[w] = encoder_counts[w]
            if debug and print_due:
                print("Δpulses:", dp_debug, "dt:", round(dt,3), "PPR:", PULSES_PER_REV)
            prev_time = now

        # --- Calibration (Relay Autotune) ---
        if calibration_mode:
            # One wheel at a time; robust start and timeout handling
            if not hasattr(calibration_mode, "state"):
                # attach simple state to the function object to keep globals minimal
                calibration_mode.state = {
                    "active": False,
                    "wheel_index": 0,
                    "start_ms": 0,
                    "last_counts": 0,
                    "last_ms": 0,
                    "last_motion_ms": 0,
                    "have_first_sample": False,
                    "prev2": None, "prev1": None, "cur": None,
                    "last_peak_ms": None, "sum_period_ms": 0, "peak_count": 0,
                    "rpm_min": 1e9, "rpm_max": -1e9,
                    "log": {w: [] for w in tires}  # small logs; can be disabled
                }

            st = calibration_mode.state

            if not st["active"]:
                st["active"] = True
                st["wheel_index"] = 0
                st["start_ms"] = now
                st["have_first_sample"] = False
                w = tires[st["wheel_index"]]
                st["last_counts"] = encoder_counts[w]
                st["last_ms"] = now
                st["last_motion_ms"] = now
                st["prev2"] = st["prev1"] = st["cur"] = None
                st["last_peak_ms"] = None; st["sum_period_ms"] = 0; st["peak_count"] = 0
                st["rpm_min"] = 1e9; st["rpm_max"] = -1e9
                # kick the wheel immediately
                motor_control(w, RELAY_PWM)
                if debug:
                    print("PID calibration START")

            # current wheel
            w = tires[st["wheel_index"]]

            # Update RPM at cadence for calibration
            rpm, st["last_counts"], st["last_ms"], dp = cal_update_rpm(w, st["last_counts"], st["last_ms"])

            if rpm is not None:
                # first sample: reset timeout clock
                if not st["have_first_sample"]:
                    st["have_first_sample"] = True
                    st["last_motion_ms"] = now
                # relay control
                cmd = RELAY_PWM if rpm < CALIBRATION_TARGET_RPM else -RELAY_PWM
                motor_control(w, cmd)
                # motion safety
                if dp > 0:
                    st["last_motion_ms"] = now
                elif ticks_diff(now, st["last_motion_ms"]) > NO_MOTION_TIMEOUT_MS:
                    if debug:
                        print("Abort:", w, "— no motion detected")
                    # advance wheel
                    st["wheel_index"] += 1
                    if st["wheel_index"] >= len(tires):
                        calibration_mode = False
                        st["active"] = False
                        if debug: print("PID calibration DONE (some aborted). params:", PID_params)
                    else:
                        motor_control(w, 0)
                        pause_between_wheels()
                        # init next wheel
                        w = tires[st["wheel_index"]]
                        st["start_ms"] = now
                        st["have_first_sample"] = False
                        st["last_counts"] = encoder_counts[w]
                        st["last_ms"] = now
                        st["last_motion_ms"] = now
                        st["prev2"] = st["prev1"] = st["cur"] = None
                        st["last_peak_ms"] = None; st["sum_period_ms"] = 0; st["peak_count"] = 0
                        st["rpm_min"] = 1e9; st["rpm_max"] = -1e9
                        motor_control(w, RELAY_PWM)
                # peak detection
                st["prev2"], st["prev1"], st["cur"] = st["prev1"], st["cur"], rpm
                p2, p1, c = st["prev2"], st["prev1"], st["cur"]
                if (p2 is not None) and (p1 is not None) and (p2 < p1 > c):
                    if st["last_peak_ms"] is not None:
                        st["sum_period_ms"] += ticks_diff(now, st["last_peak_ms"])
                        st["peak_count"] += 1
                    st["last_peak_ms"] = now
                # amplitude tracking after warmup
                if ticks_diff(now, st["start_ms"]) > WARMUP_MS:
                    if rpm < st["rpm_min"]: st["rpm_min"] = rpm
                    if rpm > st["rpm_max"]: st["rpm_max"] = rpm
                # optional: tiny log
                if len(st["log"][w]) < 200:  # cap to avoid memory blow-up
                    st["log"][w].append((ticks_diff(now, st["start_ms"]), rpm))

            # end-of-window?
            if ticks_diff(now, st["start_ms"]) >= DURATION_MS:
                # compute ZN for this wheel
                a = (st["rpm_max"] - st["rpm_min"]) / 2.0
                if st["peak_count"] >= 1 and a > 0:
                    Tu = (st["sum_period_ms"] / st["peak_count"]) / 1000.0
                    d = float(abs(RELAY_PWM))
                    Ku = (4.0 * d) / (pi * a)
                    Kp = 0.6 * Ku
                    Ki = 1.2 * Ku / Tu
                    Kd = 0.075 * Ku * Tu
                    PID_params[w]["Kp"] = Kp
                    PID_params[w]["Ki"] = Ki
                    PID_params[w]["Kd"] = Kd
                    if debug:
                        print("ZN", w, "-> Kp=", round(Kp,3), "Ki=", round(Ki,3), "Kd=", round(Kd,3))
                else:
                    if debug:
                        print("ZN failed for", w, "(insufficient oscillation)")
                # move to next wheel or finish
                motor_control(w, 0)
                st["wheel_index"] += 1
                if st["wheel_index"] >= len(tires):
                    calibration_mode = False
                    st["active"] = False
                    if debug:
                        print("PID calibration DONE. New PID params:", PID_params)
                else:
                    pause_between_wheels()
                    # init next wheel
                    w = tires[st["wheel_index"]]
                    st["start_ms"] = now
                    st["have_first_sample"] = False
                    st["last_counts"] = encoder_counts[w]
                    st["last_ms"] = now
                    st["last_motion_ms"] = now
                    st["prev2"] = st["prev1"] = st["cur"] = None
                    st["last_peak_ms"] = None; st["sum_period_ms"] = 0; st["peak_count"] = 0
                    st["rpm_min"] = 1e9; st["rpm_max"] = -1e9
                    motor_control(w, RELAY_PWM)

            # during calibration, skip normal drive below
            if print_due and debug:
                wdbg = tires[calibration_mode.state["wheel_index"]]
                print("CAL wheel=", wdbg, "rpm=", round(rpm_filt_cal[wdbg],1))
            if print_due:
                next_print_ms = now + PRINT_INTERVAL_MS
            continue

        # --- Normal drive / spin control ---
        if debug and debug_encoders_only and print_due:
            print("Measured RPM:", {w: round(last_rpm[w], 1) for w in tires})
            next_print_ms = now + PRINT_INTERVAL_MS
            continue

        # Mix: throttle + steering; spin if throttle near zero and spin command present
        thr  = smoothed["CH3"] + smoothed["CH2"]
        spin = (smoothed["CH3"] < 10 and abs(smoothed["CH4"]) > 5)
        if spin:
            speed = min(255, abs(smoothed["CH4"]) * 2)
            dirn  = "CW" if smoothed["CH4"] > 0 else "CCW"
            if print_due:
                print(f"SPIN {dirn} @ {speed}")
            # In spin, convert PWM intent to RPM setpoints (signed)
            left_cmd  = -speed if smoothed["CH4"] > 0 else  speed
            right_cmd =  speed if smoothed["CH4"] > 0 else -speed
            # NOTE: invert LEFT wheels as requested
            set_rpm = {
                "FL":  -(left_cmd/255.0)*MAX_WHEEL_RPM,
                "BL":  -(left_cmd/255.0)*MAX_WHEEL_RPM,
                "FR":   (right_cmd/255.0)*MAX_WHEEL_RPM,
                "BR":   (right_cmd/255.0)*MAX_WHEEL_RPM,
            }
        else:
            left  = max(-255, min(255, thr - smoothed["CH1"]))
            right = max(-255, min(255, thr + smoothed["CH1"]))
            if print_due:
                print(f"DRIVE L={left:.1f} R={right:.1f}")
            # NOTE: invert LEFT wheels as requested
            set_rpm = {
                "FL":  -(left/255.0)*MAX_WHEEL_RPM,
                "BL":  -(left/255.0)*MAX_WHEEL_RPM,
                "FR":   (right/255.0)*MAX_WHEEL_RPM,
                "BR":   (right/255.0)*MAX_WHEEL_RPM,
            }

        # PID control outputs per wheel
        cmds = {}
        for w in tires:
            cmds[w] = pid_update(w, set_rpm[w], last_rpm[w], now)
            motor_control(w, cmds[w])

        if print_due:
            print("Requested Speeds:", {w: int(rpm_to_pwm_setpoint(set_rpm[w])) for w in tires})
            print("Measured RPM:", {w: round(last_rpm[w], 1) for w in tires})
            print("PID PWM Out:", {w: int(cmds[w]) for w in tires})
            print("----------")
            next_print_ms = now + PRINT_INTERVAL_MS

except KeyboardInterrupt:
    print("Stopped")
    for w in tires:
        motor_control(w, 0)

