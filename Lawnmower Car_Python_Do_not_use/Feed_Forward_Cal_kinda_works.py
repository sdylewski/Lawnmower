# === FEED-FORWARD CALIBRATION (FWD-only) ===
# Older working PIO: rising-edge counter with PIO-side chunking + debounce.
# CSV logging; no plotting.

from machine import Pin, PWM
from rp2 import PIO, StateMachine, asm_pio
from time import ticks_ms, ticks_diff, sleep, localtime

# ================== CONFIG ==================
CPR_MOTOR       = 16
GEAR_RATIO      = 150
PULSES_PER_REV  = CPR_MOTOR * GEAR_RATIO   # 2400 by spec

TEST_INTERVAL_MS = 2000   # total dwell per PWM step
SETTLE_MS        = 200    # ignore initial transient in each step
PWM_STEP         = 25
MAX_PWM          = 255

# Debounce + chunking (this is the version that worked for you)
CHUNK_LIST  = [1, 2, 4]       # try 1,2,4 (or 16/32 later)
DEBOUNCE_US = 25              # post-edge holdoff (us)
SM_FREQ_HZ  = 1_000_000       # 1 MHz PIO clock (1 instr ≈ 1 µs)

# Wheels to test (FWD only)
WHEELS = ["FL", "FR", "BL", "BR"]
WHEELS = ["BR"]              # uncomment to test a single wheel quickly

# Robot-forward direction per wheel: +1 => +PWM spins robot-forward; -1 => reversed
WHEEL_DIR = {
    "FL": -1,
    "FR": +1,
    "BL": -1,
    "BR": +1,
}

# Fit window (avoid deadzone/saturation)
FIT_PWM_MIN = 75
FIT_PWM_MAX = 200

# "Expected" (display only)
MAX_WHEEL_RPM = 220.0

# ================== MOTOR PINS ==================
# Make sure these match your wiring
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
    d = 1 if pwm_cmd_hw >= 0 else 0
    a = abs(pwm_cmd_hw)
    if a > 255: a = 255
    in1.value(d); in2.value(not d)
    pwm.duty_u16(a * 257)

def stop_all():
    for w in pins:
        motor_control_rf(w, 0)

# Optional standby
try:
    Pin(1, Pin.OUT).high()
except:
    pass

# ================== PIO: rising-edge with chunking + debounce (OLDER WORKING VERSION) ==================
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

# ================== ENCODERS / COUNTERS ==================
# Make sure these match your wiring
encoder_pins  = {"FL":17, "FR":28, "BL":8, "BR":0}

encoder_counts = {w: 0 for w in encoder_pins}  # total edges (we add CHUNK_N per IRQ)
irq_chunks     = {w: 0 for w in encoder_pins}  # number of PIO IRQs seen
sms            = {}

def make_irq_handler(wheel, CHUNK_N):
    def handler(sm):
        irq_chunks[wheel]     += 1
        encoder_counts[wheel] += CHUNK_N
    return handler

def init_sms(CHUNK_N):
    # Stop any old SMs
    for sm in sms.values():
        try: sm.active(0)
        except: pass
    sms.clear()

    # Clear counters
    for w in encoder_counts: encoder_counts[w] = 0
    for w in irq_chunks:     irq_chunks[w]     = 0

    # Convert debounce µs -> PIO cycles at SM_FREQ_HZ
    deb_cycles = max(1, int(DEBOUNCE_US * (SM_FREQ_HZ // 1_000_000)))

    # Create SMs
    for i, (wheel, pin_num) in enumerate(encoder_pins.items()):
        pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        sm  = StateMachine(
            i,
            edge_counter_chunk_db,
            freq=SM_FREQ_HZ,     # makes deb_cycles truly in microseconds
            in_base=pin          # used by wait(pin, …)
        )
        sm.irq(make_irq_handler(wheel, CHUNK_N))
        sm.put(CHUNK_N)     # chunk target N  -> ISR/X
        sm.put(deb_cycles)  # debounce cycles -> OSR/Y
        sm.active(1)
        sms[wheel] = sm

# ================== FIT / CSV HELPERS ==================
def fit_pwm_vs_rpm(rows, fit_min_pwm, fit_max_pwm):
    # fit |RPM| so k,c are positive magnitudes
    xs, ys = [], []
    for r in rows:
        p = r["pwm"]; rpm_abs = abs(r["rpm"])
        if fit_min_pwm <= p <= fit_max_pwm:
            xs.append(rpm_abs); ys.append(p)
    n = len(xs)
    if n < 2: return None, None, None, n
    mx = sum(xs)/n; my = sum(ys)/n
    num = sum((x-mx)*(y-my) for x,y in zip(xs,ys))
    den = sum((x-mx)*(x-mx) for x in xs) or 1e-9
    k = abs(num / den)
    c = abs(my - k*mx)
    ss_tot = sum((y-my)**2 for y in ys) or 1e-9
    ss_res = sum((y-(c + k*x))**2 for x,y in zip(xs,ys))
    r2 = 1.0 - (ss_res / ss_tot)
    return k, c, r2, n

def now_stamp():
    try:
        y,m,d,hh,mm,ss,_,_ = localtime()
    except:
        y,m,d,hh,mm,ss = (2000,1,1,0,0,0)
    return "%04d%02d%02d_%02d%02d%02d" % (y,m,d,hh,mm,ss)

CSV_FILENAME = "%s_Feed_forward_FWD.csv" % now_stamp()

def csv_header_if_empty():
    try:
        with open(CSV_FILENAME, "r") as f: return
    except: pass
    with open(CSV_FILENAME, "w") as f:
        f.write("record_type,wheel,chunk,direction,pwm,dpulses,rpm_meas,rpm_exp,k,c,r2,n,fit_min,fit_max,irq_chunks\n")

def csv_write_sample(wheel, chunk, direction, pwm, dp, rpm_meas, rpm_exp, irqs):
    with open(CSV_FILENAME, "a") as f:
        f.write("sample,%s,%d,%s,%d,%d,%.6f,%.6f,,,,,,%d\n" %
                (wheel, chunk, direction, pwm, dp, rpm_meas, rpm_exp, irqs))

def csv_write_fit(wheel, chunk, direction, k, c, r2, n):
    with open(CSV_FILENAME, "a") as f:
        f.write("fit,%s,%d,%s,,,,,,%.6f,%.6f,%.6f,%d,%d,%d,\n" %
                (wheel, chunk, direction, k, c, r2, n, FIT_PWM_MIN, FIT_PWM_MAX))

ALL_SAMPLES = []
FITS = {}

# ================== SWEEP / TEST ==================
def sweep_fwd(wheel, CHUNK_N):
    print("\n=== Testing {} (chunk={} pulses/IRQ, FWD) ===".format(wheel, CHUNK_N))
    print(" PWM | Δpulses |  RPM_meas |  RPM_exp")
    print("----------------------------------------------")

    # settle/reset
    motor_control_rf(wheel, 0)
    t_pre = ticks_ms()
    while ticks_diff(ticks_ms(), t_pre) < SETTLE_MS: pass
    encoder_counts[wheel] = 0
    irq_chunks[wheel]     = 0

    rows = []
    for pwm_val in range(0, MAX_PWM+1, PWM_STEP):
        pwm_rf = +pwm_val

        # command
        motor_control_rf(wheel, pwm_rf)

        # settle portion
        t0 = ticks_ms()
        while ticks_diff(ticks_ms(), t0) < SETTLE_MS: pass

        # measure portion
        t1 = ticks_ms()
        start_count = encoder_counts[wheel]
        start_irqs  = irq_chunks[wheel]
        MEAS_MS = TEST_INTERVAL_MS - SETTLE_MS
        while ticks_diff(ticks_ms(), t1) < MEAS_MS: pass

        dp    = encoder_counts[wheel] - start_count
        di    = irq_chunks[wheel]     - start_irqs
        dt_s  = MEAS_MS / 1000.0
        rpm   = (dp / PULSES_PER_REV) / dt_s * 60.0
        rpm_exp = (pwm_val / MAX_PWM) * MAX_WHEEL_RPM

        print("{:4d} | {:8d} | {:10.2f} | {:8.2f}".format(pwm_val, dp, rpm, rpm_exp))
        rows.append({"pwm": pwm_val, "rpm": rpm})
        csv_write_sample(wheel, CHUNK_N, "FWD", pwm_val, dp, rpm, rpm_exp, di)
        ALL_SAMPLES.append({
            "wheel": wheel, "chunk": CHUNK_N, "direction": "FWD",
            "pwm": pwm_val, "rpm_meas": rpm, "rpm_exp": rpm_exp
        })

    motor_control_rf(wheel, 0)
    return rows

def test_wheel_forward_only(wheel, CHUNK_N):
    rows = sweep_fwd(wheel, CHUNK_N)
    k, c, r2, n = fit_pwm_vs_rpm(rows, FIT_PWM_MIN, FIT_PWM_MAX)
    if k is not None:
        print("\n[{} | chunk={} | FWD]  PWM ≈ c + k·RPM  →  k={:.3f}  c={:.3f}  R²={:.4f}  (n={})"
              .format(wheel, CHUNK_N, k, c, r2, n))
        print("FF (FWD): FF_pwm = c + k*RPM")
        FITS[(wheel, CHUNK_N, "FWD")] = {"k": k, "c": c, "r2": r2, "n": n,
                                         "fit_min": FIT_PWM_MIN, "fit_max": FIT_PWM_MAX}
        csv_write_fit(wheel, CHUNK_N, "FWD", k, c, r2, n)
    else:
        print("\n[{} | chunk={} | FWD]  Not enough points in fit window.".format(wheel, CHUNK_N))

# ================== RUN ==================
def main():
    csv_header_if_empty()
    stop_all()

    for CHUNK_N in CHUNK_LIST:
        print("\n" + "="*70)
        print("Running tests with CHUNK_N = {}".format(CHUNK_N))
        print("Debounce = {}us  |  PPR = {}  |  Settle/Measure = {}/{} ms"
              .format(DEBOUNCE_US, PULSES_PER_REV, SETTLE_MS, TEST_INTERVAL_MS - SETTLE_MS))

        init_sms(CHUNK_N)

        for wheel in WHEELS:
            encoder_counts[wheel] = 0
            irq_chunks[wheel]     = 0
            test_wheel_forward_only(wheel, CHUNK_N)
            sleep(0.5)

    stop_all()
    print("\nAll tests complete.")
    print("CSV saved to: {}".format(CSV_FILENAME))

    print("\n=== Feed-Forward Calibration Summary (Forward only) ===")
    for chunk in CHUNK_LIST:
        print("\n-- chunk = {} --".format(chunk))
        print("Wheel | k (PWM/RPM) |  c (PWM) |  R²  | n pts | fit window (PWM)")
        print("---------------------------------------------------------------")
        for wheel in WHEELS:
            fwd = FITS.get((wheel, chunk, "FWD"))
            if not fwd:
                print("{:>5} |     ---     |    ---   |  --- |   -   |   ---".format(wheel))
                continue
            print("{:>5} | {:11.3f} | {:8.3f} | {:4.3f} | {:5d} | {}..{}"
                  .format(wheel, fwd["k"], fwd["c"], fwd["r2"], fwd["n"], fwd["fit_min"], fwd["fit_max"]))

    # === Paste into main code: FF_PARAMS (FWD-only) ===
    chunk_for_export = CHUNK_LIST[-1] if CHUNK_LIST else None
    print("\n# === Paste into main code: FF_PARAMS (FWD-only) ===")
    if chunk_for_export is None:
        print("# No chunk selected; CHUNK_LIST is empty.")
    else:
        print("# Derived with CHUNK_N = {}".format(chunk_for_export))
        print("FF_PARAMS = {")
        for wheel in WHEELS:
            fwd = FITS.get((wheel, chunk_for_export, "FWD"))
            k_fwd = fwd["k"] if fwd else 0.0
            c_fwd = fwd["c"] if fwd else 0.0
            print('    "{}": {{ "FWD": {{"k": {:.3f}, "c": {:.3f}}} }},'.format(wheel, k_fwd, c_fwd))
        print("}")

main()
