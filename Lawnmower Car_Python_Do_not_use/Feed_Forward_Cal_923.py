from machine import Pin, PWM
from rp2 import PIO, StateMachine, asm_pio
from time import ticks_ms, ticks_diff, sleep, localtime

# ================== CONFIG ==================
CPR_MOTOR = 16
GEAR_RATIO = 150
PULSES_PER_REV = CPR_MOTOR * GEAR_RATIO  # 2400

TEST_INTERVAL_MS = 3000      # dwell per PWM step
PWM_STEP = 25               # step size 0..255
MAX_PWM = 250
CHUNK_LIST = [32]           # try 1, 16, 32, 64, 128

# Debounced PIO settings
DEBOUNCE_US = 25            # post-edge holdoff (try 50..150 µs)
SM_FREQ_HZ  = 1_000_000     # PIO clock so 1 instr ≈ 1 µs

# "Expected" column only (for eyeballing); not used in fit
MAX_WHEEL_RPM = 220.0

WHEELS = ["FL", "FR", "BL", "BR"]

# Robot-forward direction per wheel: +1 => +PWM spins robot-forward; -1 => reversed
WHEEL_DIR = {
    "FL": -1,
    "FR": +1,
    "BL": -1,
    "BR": +1,
}

# Linear-fit window (avoid deadzone and saturation)
FIT_PWM_MIN = 75
FIT_PWM_MAX = 200

# ================== MOTOR PINS ==================
pins = {
    "BR": (PWM(Pin(2)),  Pin(4, Pin.OUT),  Pin(3, Pin.OUT)),
    "BL": (PWM(Pin(7)),  Pin(5, Pin.OUT),  Pin(6, Pin.OUT)),
    "FR": (PWM(Pin(10)), Pin(12, Pin.OUT), Pin(11, Pin.OUT)),
    "FL": (PWM(Pin(15)), Pin(13, Pin.OUT), Pin(14, Pin.OUT)),
}
for pwm, _, _ in pins.values():
    pwm.freq(20000)

def motor_control_rf(wheel, pwm_cmd_rf):
    """Drive in ROBOT-FORWARD sign; map to hardware using WHEEL_DIR."""
    pwm_cmd_hw = int(pwm_cmd_rf * WHEEL_DIR[wheel])
    pwm, in1, in2 = pins[wheel]
    direction = 1 if pwm_cmd_hw >= 0 else 0
    abs_speed = min(abs(pwm_cmd_hw), 255)
    in1.value(direction)
    in2.value(not direction)
    pwm.duty_u16(int(abs_speed * 65535 / 255))

def stop_all():
    for w in pins:
        motor_control_rf(w, 0)

# TB6612 standby (no-op if not present)
try:
    stby = Pin(1, Pin.OUT)
    stby.high()
except Exception:
    pass

# ================== DEBOUNCED, CHUNKED PIO COUNTER ==================
# Pull order at init: 1) CHUNK_N  2) DEBOUNCE_CYCLES
@asm_pio()
def edge_counter_chunk_db():
    pull()              # OSR = chunk_target (N)
    mov(isr, osr)       # ISR holds chunk_target constant
    mov(x, osr)         # X = chunk countdown (starts at N)
    pull()              # OSR = debounce_cycles (constant for reload)
    mov(y, osr)         # Y = debounce countdown working copy
    wrap_target()
    wait(0, pin, 0)     # arm on low
    wait(1, pin, 0)     # rising edge detected -> count it
    jmp(x_dec, "deb")   # X-- ; if not zero skip IRQ, else fall-through
    irq(rel(0))         # chunk of N edges completed
    mov(x, isr)         # reload X = N
    label("deb")        # post-edge holdoff
    jmp(y_dec, "deb")   # burn debounce cycles
    mov(y, osr)         # reload Y = debounce_cycles
    wrap()

encoder_pins = {"FL":17, "FR":28, "BL":8, "BR":0}
encoder_counts = {w: 0 for w in encoder_pins}
sms = {}

def make_irq_handler(wheel, PIO_CHUNK):
    def handler(sm):
        # one IRQ per CHUNK_N debounced rising edges
        encoder_counts[wheel] += PIO_CHUNK
    return handler

def init_sms(PIO_CHUNK):
    # deactivate any existing SMs
    for sm in sms.values():
        try:
            sm.active(0)
        except:
            pass
    sms.clear()
    # zero counts
    for w in encoder_counts:
        encoder_counts[w] = 0

    deb_cycles = max(1, int(DEBOUNCE_US * (SM_FREQ_HZ // 1_000_000)))

    # start fresh with this chunk
    for i, (wheel, pin_num) in enumerate(encoder_pins.items()):
        pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        sm = StateMachine(i, edge_counter_chunk_db, freq=SM_FREQ_HZ, in_base=pin)
        sm.irq(make_irq_handler(wheel, PIO_CHUNK))
        sm.put(PIO_CHUNK)   # chunk target (N)
        sm.put(deb_cycles)  # debounce cycles
        sm.active(1)
        sms[wheel] = sm

# ================== FIT / CSV HELPERS ==================
def fit_pwm_vs_rpm(rows, fit_min_pwm, fit_max_pwm):
    """
    rows: list of dicts with {'pwm','rpm'} (rpm signed, robot-forward).
    Fit is always done on absolute RPM to ensure k and c are positive magnitudes.
    """
    xs, ys = [], []
    for r in rows:
        p = r["pwm"]
        rpm_abs = abs(r["rpm"])
        if fit_min_pwm <= p <= fit_max_pwm:
            xs.append(rpm_abs)
            ys.append(p)
    n = len(xs)
    if n < 2:
        return None, None, None, n
    mx = sum(xs) / n
    my = sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    den = sum((x - mx) ** 2 for x in xs) or 1e-9
    k = abs(num / den)
    c = abs(my - k * mx)
    ss_tot = sum((y - my) ** 2 for y in ys) or 1e-9
    ss_res = sum((y - (c + k * x)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - (ss_res / ss_tot)
    return k, c, r2, n

def now_stamp():
    try:
        y,m,d,hh,mm,ss,_,_ = localtime()
    except:
        y,m,d,hh,mm,ss = (2000,1,1,0,0,0)
    return "%04d%02d%02d_%02d%02d%02d" % (y,m,d,hh,mm,ss)

CSV_FILENAME = "%s_Feed_foward.csv" % now_stamp()

def csv_header_if_empty():
    try:
        with open(CSV_FILENAME, "r") as f:
            return
    except:
        pass
    with open(CSV_FILENAME, "w") as f:
        f.write("record_type,wheel,chunk,direction,pwm,dpulses,rpm_meas,rpm_exp,k,c,r2,n,fit_min,fit_max\n")

def csv_write_sample(wheel, chunk, direction, pwm, dp, rpm_meas, rpm_exp):
    with open(CSV_FILENAME, "a") as f:
        f.write("sample,%s,%d,%s,%d,%d,%.6f,%.6f,,,,,,\n" %
                (wheel, chunk, direction, pwm, dp, rpm_meas, rpm_exp))

def csv_write_fit(wheel, chunk, direction, k, c, r2, n):
    with open(CSV_FILENAME, "a") as f:
        f.write("fit,%s,%d,%s,,,,,,%.6f,%.6f,%.6f,%d,%d,%d\n" %
                (wheel, chunk, direction, k, c, r2, n, FIT_PWM_MIN, FIT_PWM_MAX))

# also retain samples in memory (optional; handy for later tooling)
ALL_SAMPLES = []  # dicts: wheel, chunk, direction, pwm, rpm_meas, rpm_exp
FITS = {}         # key: (wheel, chunk, direction) -> {k,c,r2,n,fit_min,fit_max}

# ================== SWEEP / TEST ==================
def sweep_one_direction(wheel, PIO_CHUNK, dir_sign):
    """
    dir_sign = +1 for FWD (robot-forward), -1 for REV (robot-reverse).
    Returns the collected rows for fitting.
    """
    label = "FWD" if dir_sign > 0 else "REV"
    print(f"\n=== Testing {wheel} (chunk={PIO_CHUNK} pulses/IRQ, {label}) ===")
    print(f"{'PWM':>4} | {'Δpulses':>8} | {'RPM_meas':>9} | {'RPM_exp':>8}")
    print("-" * 46)

    rows = []
    for pwm_val in range(0, MAX_PWM+1, PWM_STEP):
        pwm_rf = dir_sign * pwm_val  # robot-forward sign

        # start step
        start = ticks_ms()
        start_count = encoder_counts[wheel]
        motor_control_rf(wheel, pwm_rf)

        # dwell
        while ticks_diff(ticks_ms(), start) < TEST_INTERVAL_MS:
            pass

        # measure
        dp = encoder_counts[wheel] - start_count
        dt_s = TEST_INTERVAL_MS / 1000.0
        rpm_meas = (dp / PULSES_PER_REV) / dt_s * 60.0
        # sign rpm by commanded robot-forward direction
        rpm_meas *= (1 if pwm_rf >= 0 else -1)
        rpm_exp  = (abs(pwm_val) / MAX_PWM) * MAX_WHEEL_RPM

        print(f"{pwm_val:>4} | {dp:>8} | {rpm_meas:>9.2f} | {rpm_exp:>8.2f}")
        rows.append({"pwm": pwm_val, "rpm": rpm_meas})
        csv_write_sample(wheel, PIO_CHUNK, label, pwm_val, dp, rpm_meas, rpm_exp)
        ALL_SAMPLES.append({
            "wheel": wheel, "chunk": PIO_CHUNK, "direction": label,
            "pwm": pwm_val, "rpm_meas": rpm_meas, "rpm_exp": rpm_exp
        })

    motor_control_rf(wheel, 0)
    return rows

def test_wheel_both_dirs(wheel, PIO_CHUNK):
    # Forward sweep (robot-forward positive)
    rows_fwd = sweep_one_direction(wheel, PIO_CHUNK, +1)
    k, c, r2, n = fit_pwm_vs_rpm(rows_fwd, FIT_PWM_MIN, FIT_PWM_MAX)
    if k is not None:
        print(f"\n[{wheel} | chunk={PIO_CHUNK} | FWD]  PWM ≈ c + k·RPM  →  k={k:.3f}  c={c:.3f}  R²={r2:.4f}  (n={n})")
        print("FF (FWD): FF_pwm = c + k*RPM")
        csv_write_fit(wheel, PIO_CHUNK, "FWD", k, c, r2, n)
        FITS[(wheel, PIO_CHUNK, "FWD")] = {
            "k": k, "c": c, "r2": r2, "n": n,
            "fit_min": FIT_PWM_MIN, "fit_max": FIT_PWM_MAX
        }
    else:
        print(f"\n[{wheel} | chunk={PIO_CHUNK} | FWD]  Not enough points in fit window.")

#     # Reverse sweep (robot-reverse negative)
#     rows_rev = sweep_one_direction(wheel, PIO_CHUNK, -1)
#     k, c, r2, n = fit_pwm_vs_rpm(rows_rev, FIT_PWM_MIN, FIT_PWM_MAX)
#     if k is not None:
#         print(f"\n[{wheel} | chunk={PIO_CHUNK} | REV]  PWM ≈ c + k·RPM  →  k={k:.3f}  c={c:.3f}  R²={r2:.4f}  (n={n})")
#         print("FF (REV): FF_pwm = c + k*RPM   (RPM will be negative in reverse)")
#         csv_write_fit(wheel, PIO_CHUNK, "REV", k, c, r2, n)
#         FITS[(wheel, PIO_CHUNK, "REV")] = {
#             "k": k, "c": c, "r2": r2, "n": n,
#             "fit_min": FIT_PWM_MIN, "fit_max": FIT_PWM_MAX
#         }
#     else:
#         print(f"\n[{wheel} | chunk={PIO_CHUNK} | REV]  Not enough points in fit window.")

# ================== RUN ==================
def main():
    # CSV header
    csv_header_if_empty()
    stop_all()

    for PIO_CHUNK in CHUNK_LIST:
        print("\n" + "="*70)
        print(f"Running tests with PIO_CHUNK = {PIO_CHUNK}")
        # Estimate IRQ rate at MAX_WHEEL_RPM using your PPR and chunk size
        est_irqs = int(((MAX_WHEEL_RPM/60.0) * PULSES_PER_REV) / PIO_CHUNK)
        print(f"Estimated IRQ rate per wheel at {int(MAX_WHEEL_RPM)} RPM: ~{est_irqs} IRQ/s")
        init_sms(PIO_CHUNK)
        for wheel in ["FL", "FR", "BL", "BR"]:
            encoder_counts[wheel] = 0
            test_wheel_both_dirs(wheel, PIO_CHUNK)
            sleep(0.5)

    stop_all()
    print("\nAll tests complete.")
    print(f"CSV saved to: {CSV_FILENAME}")
    
    print("\n=== Feed-Forward Calibration Summary (Forward direction only) ===")
    for chunk in CHUNK_LIST:
        print(f"\n-- chunk = {chunk} --")
        print("Wheel | k (PWM/RPM) |  c (PWM) |  R²  | n pts | fit window (PWM)")
        print("---------------------------------------------------------------")
        for wheel in WHEELS:
            fit_fwd = FITS.get((wheel, chunk, "FWD"))
            if not fit_fwd:
                print(f"{wheel:>5} |     ---     |    ---   |  --- |   -   |   ---")
                continue
            k  = fit_fwd["k"]; c = fit_fwd["c"]; r2 = fit_fwd["r2"]; n = fit_fwd["n"]
            fmin = fit_fwd.get("fit_min", FIT_PWM_MIN)
            fmax = fit_fwd.get("fit_max", FIT_PWM_MAX)
            print(f"{wheel:>5} | {k:11.3f} | {c:8.3f} | {r2:4.3f} | {n:5d} | {fmin}..{fmax}")
    
    print("\n=== Feed-Forward Calibration Summary (Reverse direction only) ===")
    for chunk in CHUNK_LIST:
        print(f"\n-- chunk = {chunk} --")
        print("Wheel | k (PWM/RPM) |  c (PWM) |  R²  | n pts | fit window (PWM)")
        print("---------------------------------------------------------------")
        for wheel in WHEELS:
            fit_rev = FITS.get((wheel, chunk, "REV"))
            if not fit_rev:
                print(f"{wheel:>5} |     ---     |    ---   |  --- |   -   |   ---")
                continue
            k  = fit_rev["k"]; c = fit_rev["c"]; r2 = fit_rev["r2"]; n = fit_rev["n"]
            fmin = fit_rev.get("fit_min", FIT_PWM_MIN)
            fmax = fit_rev.get("fit_max", FIT_PWM_MAX)
            print(f"{wheel:>5} | {k:11.3f} | {c:8.3f} | {r2:4.3f} | {n:5d} | {fmin}..{fmax}")
    print("\nUse in FF:  FF_pwm = sign(rpm) * (c + k * abs(rpm))")
    
    # === Paste into main code: FF_PARAMS (per direction) ===
    chunk_for_export = CHUNK_LIST[-1] if CHUNK_LIST else None
    print("\n# === Paste into main code: FF_PARAMS ===")
    if chunk_for_export is None:
        print("# No chunk selected; CHUNK_LIST is empty.")
    else:
        print("# Derived with PIO_CHUNK = {}".format(chunk_for_export))
        print("FF_PARAMS = {")
        for wheel in ["FL", "FR", "BL", "BR"]:
            fwd = FITS.get((wheel, chunk_for_export, "FWD"))
            rev = FITS.get((wheel, chunk_for_export, "REV"))
            k_fwd = fwd["k"] if fwd else 0.0
            c_fwd = fwd["c"] if fwd else 0.0
            k_rev = rev["k"] if rev else 0.0
            c_rev = rev["c"] if rev else 0.0
            print('    "{}": {{'.format(wheel))
            print('        "FWD": {{"k": {:.3f}, "c": {:.3f}}},'.format(k_fwd, c_fwd))
            print('        "REV": {{"k": {:.3f}, "c": {:.3f}}},'.format(k_rev, c_rev))
            print("    },")
        print("}")

main()
