from machine import Pin, PWM, time_pulse_us
from rp2 import PIO, StateMachine, asm_pio
from time import sleep, ticks_ms, ticks_diff, sleep_us

"""
RC CAR PID SPEED CONTROL — NO CALIBRATION
- RC input reading & smoothing
- PIO encoder counting (IRQ per pulse)
- Closed-loop speed control per wheel using encoder RPM feedback
- Left wheels inverted in drive & spin modes (as requested)
- No autotune code; gains are static and editable
"""

# === USER/BUILD SETTINGS ===
debug = True                               # master debug flag
debug_encoders_only = False                # only encoder prints when True

CPR_MOTOR = 16                             # encoder counts per motor rev (1 ch, 1 edge)
GEAR_RATIO = 150                           # gearbox ratio
PULSES_PER_REV = CPR_MOTOR * GEAR_RATIO    # pulses per output shaft revolution

RPM_MIN_INTERVAL_MS = 100                  # cadence for RPM math (sampling window)
PRINT_INTERVAL_MS = 500                    # cadence for debug prints
ALPHA = 0.2                                # smoothing for inputs/RPM display
PID_DT_MIN_MS = 20                         # minimum dt for PID updates (limit jitter)

MAX_WHEEL_RPM = 120.0                      # estimated max wheel RPM at full PWM (adjust per hardware)

# === TEST MODE SETTINGS (CH5 switch) ===
TEST_TARGET_RPM = 20.0          # target rpm during test mode
TEST_DWELL_MS   = 5000          # how long to run each wheel before switching (ms)

# === PID GAINS (static; tune manually) ===
PID_params = {
    "FL": {"Kp": 1.0, "Ki": 0.0, "Kd": 0.0},
    "FR": {"Kp": 1.0, "Ki": 0.0, "Kd": 0.0},
    "BL": {"Kp": 1.0, "Ki": 0.0, "Kd": 0.0},
    "BR": {"Kp": 1.0, "Ki": 0.0, "Kd": 0.0},
}
INTEGRAL_LIMIT = 255.0                     # bound for integral anti-windup in PWM units

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

# === RC helpers ===
from time import sleep_us

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

# === PID update (velocity form) ===
PID_state = {w: {"integral": 0.0, "prev_err": 0.0, "last_t": prev_time, "out": 0.0} for w in encoder_pins}

def pid_update(wheel, set_rpm, meas_rpm, now_ms):
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

# === Main loop ===
tires = ["FL", "FR", "BL", "BR"]

# Test mode state (advanced when CH5 is ON)
test_state = {"active": False, "wheel_index": 0, "start_ms": 0}

try:
    while True:
        carLED.toggle(); sleep(0.05)  # Heartbeat LED

        # --- Read RC and smooth ---
        rc1, r1 = read_channel(CH1, -100, 100)  # steering
        rc2, r2 = read_channel(CH2, -100, 100)  # extra mixing
        rc3, r3 = read_channel(CH3, 0, 155)     # throttle
        rc4, r4 = read_channel(CH4, -100, 100)  # spin
        sw,  r5 = read_switch(CH5)              # spare switch
        smoothed["CH1"] = smoothed["CH1"]*(1-ALPHA) + rc1*ALPHA
        smoothed["CH2"] = smoothed["CH2"]*(1-ALPHA) + rc2*ALPHA
        smoothed["CH3"] = smoothed["CH3"]*(1-ALPHA) + rc3*ALPHA
        smoothed["CH4"] = smoothed["CH4"]*(1-ALPHA) + rc4*ALPHA
        
        now = ticks_ms()
        elapsed = ticks_diff(now, prev_time)
        print_due = now >= next_print_ms

        # --- RPM update at cadence ---
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

            # --- Normal drive / spin control (set target RPM from RC) ---
            if debug and debug_encoders_only and print_due:
                print("Measured RPM:", {w: round(last_rpm[w], 1) for w in tires})
                next_print_ms = now + PRINT_INTERVAL_MS
                continue

            thr  = smoothed["CH3"] + smoothed["CH2"]
            spin = (smoothed["CH3"] < 10 and abs(smoothed["CH4"]) > 5)
            if spin:
                speed = min(255, abs(smoothed["CH4"]) * 2)
                dirn  = "CW" if smoothed["CH4"] > 0 else "CCW"
                if print_due:
                    print(f"SPIN {dirn} @ {speed}")
                left_cmd  = -speed if smoothed["CH4"] > 0 else  speed
                right_cmd =  speed if smoothed["CH4"] > 0 else -speed
                set_rpm = {
                    "FL":  -(left_cmd/255.0)*MAX_WHEEL_RPM,  # invert left side
                    "BL":  -(left_cmd/255.0)*MAX_WHEEL_RPM,
                    "FR":   (right_cmd/255.0)*MAX_WHEEL_RPM,
                    "BR":   (right_cmd/255.0)*MAX_WHEEL_RPM,
                }
            else:
                left  = max(-255, min(255, thr - smoothed["CH1"]))
                right = max(-255, min(255, thr + smoothed["CH1"]))
                if print_due:
                    print(f"DRIVE L={left:.1f} R={right:.1f}")
                set_rpm = {
                    "FL":  -(left/255.0)*MAX_WHEEL_RPM,   # invert left side
                    "BL":  -(left/255.0)*MAX_WHEEL_RPM,
                    "FR":   (right/255.0)*MAX_WHEEL_RPM,
                    "BR":   (right/255.0)*MAX_WHEEL_RPM,
                }

            # Closed-loop speed control: compute PWM from PID on RPM error
            cmds = {}
            for w in tires:
                cmds[w] = pid_update(w, set_rpm[w], last_rpm[w], now)
                motor_control(w, cmds[w])

            if print_due:
                print("Raw1:", r1, " Raw2:", r2, " Raw3:", r3, " Raw4:", r4, " Raw5:", r5)
                print("RC1:", rc1, " RC2:", rc2, " RC3:", rc3, " RC4:", rc4, " SW:", sw)   
                print("Encoders:", {w: encoder_counts[w] for w in tires})
                print("Target RPM:", {w: int(set_rpm[w]) for w in tires})
                print("Measured RPM:", {w: round(last_rpm[w], 1) for w in tires})
                print("PID PWM Out:", {w: int(cmds[w]) for w in tires})
                print("----------")
                next_print_ms = now + PRINT_INTERVAL_MS


except KeyboardInterrupt:
    print("Stopped")
    for w in tires:
        motor_control(w, 0)

