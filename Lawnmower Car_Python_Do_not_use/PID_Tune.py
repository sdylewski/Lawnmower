from machine import Pin, PWM
from rp2 import PIO, StateMachine, asm_pio
from time import sleep, ticks_ms, ticks_diff
from math import pi

"""
PID RELAY AUTOTUNE with chunk_counter PIO, signed RPM (robot-forward),
deadband + hysteresis, optional per-wheel auto-target, and end-of-run summary.
"""

# === USER SETTINGS ===
CPR_MOTOR = 16
GEAR_RATIO = 150
PULSES_PER_REV = CPR_MOTOR * GEAR_RATIO  # 2400
PULSES_PER_REV = 32000 # over-ride with measured values

# PIO chunk size: use the same you validated in your FF tests (e.g., 32/64/etc.)
PIO_CHUNK = 64

# Relay test knobs
DEFAULT_TARGET_RPM = 50.0         # fallback if AUTO_TARGET is False
RELAY_PWM = 180                   # bang-bang magnitude (0..255)
DURATION_MS = 10000               # total run time per wheel
WARMUP_MS = 1500                  # ignore peaks until after warmup
NO_MOTION_TIMEOUT_MS = 1500       # safety
SAMPLE_MS = 250                   # RPM update cadence
ALPHA = 0.15                      # RPM low-pass

# Chatter controls
BAND_RPM = 2.0                    # deadband around target
SWITCH_MIN_MS = 400               # min time between flips
SWITCH_PAUSE_S = 1                # pause between wheels

# Auto-target (measure steady forward RPM at RELAY_PWM and target a fraction)
AUTO_TARGET = True
AUTO_TARGET_FRAC = 0.6            # e.g., 60% of steady speed

# Max wheel RPM (for sanity / prints only)
MAX_WHEEL_RPM = 220.0

# Robot-forward direction per wheel (+1: +PWM spins robot-forward, -1: reversed)
WHEEL_DIR = {
    "FL": -1,
    "FR": +1,
    "BL": -1,
    "BR": +1,
}

# === MOTOR DRIVER PINS ===
pins = {
    "BR": (PWM(Pin(2)),  Pin(4, Pin.OUT),  Pin(3, Pin.OUT)),
    "BL": (PWM(Pin(7)),  Pin(5, Pin.OUT),  Pin(6, Pin.OUT)),
    "FR": (PWM(Pin(10)), Pin(12, Pin.OUT), Pin(11, Pin.OUT)),
    "FL": (PWM(Pin(15)), Pin(13, Pin.OUT), Pin(14, Pin.OUT)),
}
for pwm, _, _ in pins.values():
    pwm.freq(20000)

def motor_control_rf(wheel, pwm_cmd_rf):
    """Drive using ROBOT-FORWARD sign; map to hardware using WHEEL_DIR."""
    pwm_cmd_hw = int(pwm_cmd_rf * WHEEL_DIR[wheel])
    pwm, in1, in2 = pins[wheel]
    direction = 1 if pwm_cmd_hw >= 0 else 0
    abs_speed = min(abs(pwm_cmd_hw), 255)
    in1.value(direction)
    in2.value(not direction)
    pwm.duty_u16(int(abs_speed * 65535 / 255))
    last_hw_cmd_sign[wheel] = 1 if pwm_cmd_hw >= 0 else -1

def stop_all():
    for w in pins:
        motor_control_rf(w, 0)

# Standby (ignore if not present)
#Front_stby = Pin(9, Pin.OUT); Front_stby.high()
stby  = Pin(1, Pin.OUT);
stby.high()

# === PIO CHUNKED ENCODER COUNTER ===
@asm_pio()
def chunk_counter():
    pull()            # OSR = N
    mov(y, osr)       # Y = N
    mov(x, y)         # X = N (backup)
    label("loop")
    wait(1, pin, 0)
    wait(0, pin, 0)
    jmp(y_dec, "loop")
    irq(rel(0))       # N pulses seen
    mov(y, x)         # reload Y
    jmp("loop")

encoder_pins = {"FR":17, "FL":28, "BL":8, "BR":0}
encoder_counts = {w: 0 for w in encoder_pins}
sms = {}

def make_irq_handler(wheel, chunk):
    def handler(sm):
        encoder_counts[wheel] += chunk
    return handler

def init_sms(chunk):
    # stop old
    for sm in sms.values():
        try: sm.active(0)
        except: pass
    sms.clear()
    # zero counts
    for w in encoder_counts:
        encoder_counts[w] = 0
    # start
    for i, (wheel, pin_num) in enumerate(encoder_pins.items()):
        pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        sm = StateMachine(i, chunk_counter, in_base=pin)
        sm.irq(make_irq_handler(wheel, chunk))
        sm.put(chunk)
        sm.active(1)
        sms[wheel] = sm

# === STATE ===
rpm_filt = {w: 0.0 for w in encoder_pins}
last_hw_cmd_sign = {w: 1 for w in encoder_pins}  # last HARDWARE sign for RPM sign inference

def update_running_rpm(wheel, last_counts, last_ms):
    """Return (rpm_filtered, new_counts, new_ms, dp_unsigned) at cadence SAMPLE_MS.
       RPM is ROBOT-FORWARD signed based on last HW command sign and WHEEL_DIR.
    """
    now = ticks_ms()
    if ticks_diff(now, last_ms) < SAMPLE_MS:
        return None, last_counts, last_ms, 0
    dt_s = ticks_diff(now, last_ms) / 1000.0
    dp = encoder_counts[wheel] - last_counts  # unsigned pulses from PIO
    dp_signed = dp * last_hw_cmd_sign[wheel] * WHEEL_DIR[wheel]  # map to ROBOT-FORWARD sign
    inst_rpm = (dp_signed / PULSES_PER_REV) / dt_s * 60.0
    rf = rpm_filt[wheel]
    rf = (1-ALPHA)*rf + ALPHA*inst_rpm
    rpm_filt[wheel] = rf
    return rf, encoder_counts[wheel], now, dp

def detect_peak(prev2, prev1, cur):
    return (prev2 is not None) and (prev1 is not None) and (prev2 < prev1 > cur)

# === OPTIONAL: quick steady-speed measure to auto-target ===
def measure_forward_ss_rpm(wheel, dwell_ms=1500):
    """Run forward at +RELAY_PWM (robot-forward) and return filtered RPM."""
    rpm_filt[wheel] = 0.0
    start = ticks_ms()
    motor_control_rf(wheel, +RELAY_PWM)
    last_counts = encoder_counts[wheel]
    last_ms = start
    # allow spin-up and sampling
    while ticks_diff(ticks_ms(), start) < dwell_ms:
        rpm, last_counts, last_ms, _ = update_running_rpm(wheel, last_counts, last_ms)
        sleep(0.01)
    motor_control_rf(wheel, 0)
    return rpm_filt[wheel]

# === MAIN AUTOTUNE ===
print("Starting PID Calibration (target=%.1f RPM; band=±%.1f RPM; chunk=%d)...)"
      % (DEFAULT_TARGET_RPM, BAND_RPM, PIO_CHUNK))

init_sms(PIO_CHUNK)

summary = {}  # wheel -> dict of results

for wheel in ("FL","FR","BL","BR"):
    print("\n=== Calibrating:", wheel, "===")

    # pick per-wheel target if requested
    if AUTO_TARGET:
        fwd_ss = measure_forward_ss_rpm(wheel, dwell_ms=1500)
        target = max(5.0, abs(fwd_ss) * AUTO_TARGET_FRAC)
        print("Auto-target: %s forward_ss=%.1f RPM -> target=%.1f RPM" % (wheel, fwd_ss, target))
    else:
        target = DEFAULT_TARGET_RPM

    # set up
    start_ms = ticks_ms()
    last_counts = encoder_counts[wheel]
    last_ms = start_ms
    last_motion_ms = start_ms

    have_first_sample = False
    prev2 = None; prev1 = None; cur = None
    last_peak_ms = None
    sum_period_ms = 0
    peak_count = 0
    rpm_min =  1e9
    rpm_max = -1e9

    # start forward
    cmd = +RELAY_PWM
    motor_control_rf(wheel, cmd)
    last_cmd = cmd
    last_switch_ms = ticks_ms()

    while ticks_diff(ticks_ms(), start_ms) < DURATION_MS:
        rpm, last_counts, last_ms, dp = update_running_rpm(wheel, last_counts, last_ms)
        now = ticks_ms()

        if rpm is None:
            sleep(0.01)
            continue

        if not have_first_sample:
            have_first_sample = True
            last_motion_ms = now

        # safety: no motion?
        if dp > 0:
            last_motion_ms = now
        elif ticks_diff(now, last_motion_ms) > NO_MOTION_TIMEOUT_MS:
            print("Abort:", wheel, "— no motion detected")
            break

        # bang-bang with deadband + min switch dwell (hysteresis)
        desired_cmd = last_cmd
        if rpm < (target - BAND_RPM):
            desired_cmd = +RELAY_PWM
        elif rpm > (target + BAND_RPM):
            desired_cmd = -RELAY_PWM
        # only switch if we've respected min dwell
        if desired_cmd != last_cmd and ticks_diff(now, last_switch_ms) >= SWITCH_MIN_MS:
            last_cmd = desired_cmd
            last_switch_ms = now
            motor_control_rf(wheel, last_cmd)

        # peak detection (on filtered rpm)
        prev2, prev1, cur = prev1, cur, rpm
        if detect_peak(prev2, prev1, cur):
            if last_peak_ms is not None:
                sum_period_ms += ticks_diff(now, last_peak_ms)
                peak_count += 1
            last_peak_ms = now

        # record extrema after warmup
        if ticks_diff(now, start_ms) > WARMUP_MS:
            rpm_min = min(rpm_min, rpm)
            rpm_max = max(rpm_max, rpm)

        sleep(0.01)

    motor_control_rf(wheel, 0)

    # compute ZN from oscillation
    if peak_count >= 1 and rpm_max > rpm_min:
        Tu = (sum_period_ms / peak_count) / 1000.0    # ultimate period [s]
        a = (rpm_max - rpm_min) / 2.0                 # oscillation amplitude [RPM]
        d = float(abs(RELAY_PWM))                     # relay magnitude [PWM]
        Ku = (4.0 * d) / (pi * a)                     # ultimate gain
        Kp = 0.6 * Ku
        Ki = 1.2 * Ku / Tu
        Kd = 0.075 * Ku * Tu
        print("Result:", wheel,
              "target=%.1fRPM" % target,
              "Tu=%.3fs" % Tu,
              "a=%.2fRPM" % a,
              "=> Kp=%.3f" % Kp,
              "Ki=%.3f" % Ki,
              "Kd=%.3f" % Kd)
        summary[wheel] = {
            "target": target, "Tu": Tu, "a": a,
            "Kp": Kp, "Ki": Ki, "Kd": Kd,
            "ok": True
        }
    else:
        print("ZN failed (not enough oscillation):", wheel,
              "(peaks=%d, rpm_min=%.2f, rpm_max=%.2f)" % (peak_count, rpm_min, rpm_max))
        summary[wheel] = {"target": target, "ok": False}

    print("Switching in", SWITCH_PAUSE_S, "s...")
    sleep(SWITCH_PAUSE_S)

stop_all()
print("\nCalibration complete.\n")

# === SUMMARY TABLE ===
print("=== PID Autotune Summary ===")
print("Wheel  Target  Tu(s)     Amp     Kp       Ki       Kd   Note")
for w in ("FL","FR","BL","BR"):
    s = summary.get(w, None)
    if not s:
        print(f"{w:>4}  {'-':>6}  {'-':>6}  {'-':>7}  {'-':>6}  {'-':>7}  {'-':>6}  ---")
        continue
    if s.get("ok"):
        print(f"{w:>4}  {s['target']:>6.1f}  {s['Tu']:>6.3f}  {s['a']:>7.2f}  "
              f"{s['Kp']:>6.3f}  {s['Ki']:>7.3f}  {s['Kd']:>6.3f}  OK")
    else:
        print(f"{w:>4}  {s['target']:>6.1f}  {'-':>6}  {'-':>7}  {'-':>6}  {'-':>7}  {'-':>6}  FAIL")

# === Paste into main code: PID_GAINS ===
print("\n# === Paste into main code: PID_GAINS ===")
print("PID_GAINS = {")
for wheel in ["FL", "FR", "BL", "BR"]:
    s = summary.get(wheel)
    if s and s.get("ok"):
        print(f'    "{wheel}": {{"Kp": {s["Kp"]:.3f}, "Ki": {s["Ki"]:.3f}, "Kd": {s["Kd"]:.3f}}},')
    else:
        print(f'    "{wheel}": {{"Kp": 0.0, "Ki": 0.0, "Kd": 0.0}},  # TODO: no valid tune')
print("}")

