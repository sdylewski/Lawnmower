# === FEED-FORWARD CALIBRATION (FWD-only) ===
# Debounced, qualified edges in PIO with PIO-side chunking (1 IRQ per CHUNK_N edges).
# Rising/falling selectable; counts IRQs only and derives true edge count via remainder.
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

# Debounce + chunking
CHUNK_LIST  = [1, 32]     # edges ~constant; IRQs ~1/n
DEBOUNCE_US = 15          # post-edge qualification time (us)
SM_FREQ_HZ  = 1_000_000   # 1 MHz PIO clock (1 instr ≈ 1 µs)
EDGE_MODE   = "rising"    # "rising" or "falling"

# Wheels to test (FWD only) — instantiate PIO only for these
encoder_pins  = {"FL":17, "FR":28, "BL":8, "BR":0}
WHEELS = ["FL"]           # set to the single connected wheel for bench tests

# Robot-forward direction per wheel: +1 => +PWM spins robot-forward; -1 => reversed
WHEEL_DIR = {"FL": -1, "FR": +1, "BL": -1, "BR": +1}

# Fit window (avoid deadzone/saturation)
FIT_PWM_MIN = 75
FIT_PWM_MAX = 200

# "Expected" (display only)
MAX_WHEEL_RPM = 220.0

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
    d = 1 if pwm_cmd_hw >= 0 else 0
    a = abs(pwm_cmd_hw);  a = 255 if a > 255 else a
    in1.value(d); in2.value(not d)
    pwm.duty_u16(a * 257)

def stop_all():
    for w in pins:
        motor_control_rf(w, 0)

# Optional standby (TB6612 etc.)
try:
    Pin(1, Pin.OUT).high()
except:
    pass

# ================== PIO PROGRAMS (bistable: pre-qual + post-qual) ==================
# Y = chunk countdown; ISR holds N (chunk size constant); OSR holds deb_cycles constant; X is the working countdown.

@asm_pio()
def edge_counter_chunk_db_rise():
    # Pull #1: chunk N
    pull()
    mov(isr, osr)          # ISR = N (constant)
    mov(y,   osr)          # Y   = N (countdown to IRQ)

    # Pull #2: debounce cycles
    pull()
    mov(x,   osr)          # X = deb_cycles (working)
    # OSR now keeps deb_cycles constant for reloads

    wrap_target()

    # Pre-qual: LOW must be stable for deb_cycles
    label("arm_low_reload")
    mov(x, osr)            # reload X = deb_cycles
    label("qual_low")
    jmp(pin, "arm_low_reload")  # if it went HIGH early, restart LOW qualification
    jmp(x_dec, "qual_low")      # burn X while LOW is held

    # Rising edge
    wait(1, pin, 0)

    # Post-qual: HIGH must be stable for deb_cycles
    mov(x, osr)            # reload X
    label("qual_high")
    jmp(pin, "dec_high")   # stay here while HIGH
    jmp("arm_low_reload")  # if it dropped early, discard and start over
    label("dec_high")
    jmp(x_dec, "qual_high")

    # Count accepted edge; raise IRQ each N edges
    jmp(y_dec, "done_edge")  # Y-- ; if not zero, no IRQ this time
    irq(rel(0))              # N qualified edges -> IRQ0
    mov(y, isr)              # reload Y = N
    label("done_edge")
    jmp("arm_low_reload")
    wrap()

@asm_pio()
def edge_counter_chunk_db_fall():
    # Pull #1: chunk N
    pull()
    mov(isr, osr)
    mov(y,   osr)
    # Pull #2: debounce cycles
    pull()
    mov(x,   osr)

    wrap_target()

    # Pre-qual: HIGH must be stable for deb_cycles
    label("arm_high_reload")
    mov(x, osr)              # reload X = deb_cycles
    label("qual_high")
    jmp(pin, "dec_high")     # if HIGH, go decrement X
    jmp("arm_high_reload")   # if LOW early, restart HIGH qualification
    label("dec_high")
    jmp(x_dec, "qual_high")  # burn X while HIGH is held

    # Falling edge
    wait(0, pin, 0)

    # Post-qual: LOW must be stable for deb_cycles
    mov(x, osr)
    label("qual_low")
    jmp(pin, "arm_high_reload")  # if it rose early, discard and re-qual HIGH
    jmp(x_dec, "qual_low")

    # Count accepted edge; raise IRQ each N edges
    jmp(y_dec, "done_edge")
    irq(rel(0))
    mov(y, isr)
    label("done_edge")
    jmp("arm_high_reload")
    wrap()

# ================== ENCODERS / COUNTERS ==================
irq_chunks = {w: 0 for w in encoder_pins}   # count IRQs; derive edges = IRQs*N + remainder
sms        = {}

def make_irq_handler(wheel):
    def handler(sm):
        irq_chunks[wheel] += 1
    return handler

def _read_y(sm):
    # Snapshot Y via ISR->FIFO
    sm.exec("mov(isr, y)")
    sm.exec("push()")
    return sm.get() & 0xFFFFFFFF

def _align_chunk_phase(sm):
    # Reset Y := N (value stored in ISR) so the next accepted edge starts a fresh chunk
    sm.exec("mov(y, isr)")

def _drain_rx(sm, max_reads=8):
    """Best-effort: empty RX FIFO if firmware exposes a length; otherwise no-op."""
    n = 0
    try:
        n = sm.rx_fifo()   # some firmwares expose a readable count
    except:
        n = 0
    for _ in range(min(int(n or 0), max_reads)):
        try:
            _ = sm.get()
        except:
            break

def init_sms(CHUNK_N):
    # stop old
    for sm in sms.values():
        try: sm.active(0)
        except: pass
    sms.clear()
    for w in irq_chunks: irq_chunks[w] = 0

    deb_cycles = max(1, int(DEBOUNCE_US * (SM_FREQ_HZ // 1_000_000)))
    prog = edge_counter_chunk_db_rise if EDGE_MODE == "rising" else edge_counter_chunk_db_fall

    # Instantiate PIO only for the wheels under test
    for i, wheel in enumerate(WHEELS):
        pin_num = encoder_pins[wheel]
        pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        sm  = StateMachine(
            i, prog,
            freq=SM_FREQ_HZ,
            in_base=pin,       # for wait(..., pin, 0)
            jmp_pin=pin        # for jmp(pin, ...)
        )
        sm.irq(make_irq_handler(wheel))
        sm.put(CHUNK_N)     # N -> ISR & Y
        sm.put(deb_cycles)  # deb_cycles -> OSR & X
        sm.active(1)
        sms[wheel] = sm

# ================== FIT / CSV HELPERS ==================
def fit_pwm_vs_rpm(rows, fit_min_pwm, fit_max_pwm):
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
        f.write("record_type,wheel,chunk,edge,direction,pwm,dpulses,rpm_meas,rpm_exp,k,c,r2,n,fit_min,fit_max,irq_chunks\n")

def csv_write_sample(wheel, chunk, edge, direction, pwm, dp, rpm_meas, rpm_exp, irqs):
    with open(CSV_FILENAME, "a") as f:
        f.write("sample,%s,%d,%s,%s,%d,%d,%.6f,%.6f,,,,,,%d\n" %
                (wheel, chunk, edge, direction, pwm, dp, rpm_meas, rpm_exp, irqs))

def csv_write_fit(wheel, chunk, edge, direction, k, c, r2, n):
    with open(CSV_FILENAME, "a") as f:
        f.write("fit,%s,%d,%s,%s,,,,,,%.6f,%.6f,%.6f,%d,%d,%d,\n" %
                (wheel, chunk, edge, direction, k, c, r2, n, FIT_PWM_MIN, FIT_PWM_MAX))

ALL_SAMPLES = []
FITS = {}

# ================== SWEEP / TEST ==================
def sweep_fwd(wheel, CHUNK_N):
    print("\n=== Testing {} (chunk={} pulses/IRQ, FWD) ===".format(wheel, CHUNK_N))
    print("Edge = {}  |  Debounce = {}us  |  PPR = {}  |  Settle/Measure = {}/{} ms"
          .format(EDGE_MODE, DEBOUNCE_US, PULSES_PER_REV, SETTLE_MS, TEST_INTERVAL_MS - SETTLE_MS))
    print("\n PWM | Δpulses |  RPM_meas |  RPM_exp")
    print("----------------------------------------------")

    # settle/reset before the whole sweep
    motor_control_rf(wheel, 0)
    t_pre = ticks_ms()
    while ticks_diff(ticks_ms(), t_pre) < SETTLE_MS: pass
    irq_chunks[wheel] = 0

    rows = []
    for pwm_val in range(0, MAX_PWM+1, PWM_STEP):
        pwm_rf = +pwm_val

        # command step
        motor_control_rf(wheel, pwm_rf)

        # settle portion
        t0 = ticks_ms()
        while ticks_diff(ticks_ms(), t0) < SETTLE_MS: pass

        # Align chunk phase to start-of-window (Y := N), clear any RX residue, then start
        sm = sms[wheel]
        sm.active(0)
        _align_chunk_phase(sm)
        _drain_rx(sm)
        start_irqs = irq_chunks[wheel]
        sm.active(1)

        # measure portion
        t1 = ticks_ms()
        MEAS_MS = TEST_INTERVAL_MS - SETTLE_MS
        while ticks_diff(ticks_ms(), t1) < MEAS_MS: pass

        # Snapshot remainder at end (how deep into the current chunk we are)
        sm.active(0)
        y_left = _read_y(sm)               # Y = edges remaining in current chunk
        sm.active(1)

        di        = irq_chunks[wheel] - start_irqs
        remainder = (CHUNK_N - (y_left % CHUNK_N)) % CHUNK_N
        dp_edges  = di * CHUNK_N + remainder

        dt_s  = MEAS_MS / 1000.0
        rpm   = (dp_edges / PULSES_PER_REV) / dt_s * 60.0
        rpm_exp = (pwm_val / MAX_PWM) * MAX_WHEEL_RPM

        print("{:4d} | {:8d} | {:10.2f} | {:8.2f}".format(pwm_val, dp_edges, rpm, rpm_exp))

        rows.append({"pwm": pwm_val, "rpm": rpm})
        csv_write_sample(wheel, CHUNK_N, EDGE_MODE, "FWD", pwm_val, dp_edges, rpm, rpm_exp, di)
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
        print("\n[{} | chunk={} | {} | FWD]  PWM ≈ c + k·RPM  →  k={:.3f}  c={:.3f}  R²={:.4f}  (n={})"
              .format(wheel, CHUNK_N, EDGE_MODE, k, c, r2, n))
        print("FF (FWD): FF_pwm = c + k*RPM")
        FITS[(wheel, CHUNK_N, "FWD", EDGE_MODE)] = {
            "k": k, "c": c, "r2": r2, "n": n,
            "fit_min": FIT_PWM_MIN, "fit_max": FIT_PWM_MAX
        }
        csv_write_fit(wheel, CHUNK_N, EDGE_MODE, "FWD", k, c, r2, n)
    else:
        print("\n[{} | chunk={} | {} | FWD]  Not enough points in fit window."
              .format(wheel, CHUNK_N, EDGE_MODE))

# ================== RUN ==================
def main():
    csv_header_if_empty()
    stop_all()

    for CHUNK_N in CHUNK_LIST:
        print("\n" + "="*70)
        print("Running tests with CHUNK_N = {}".format(CHUNK_N))
        print("Edge = {}  |  Debounce = {}us  |  PPR = {}  |  Settle/Measure = {}/{} ms"
              .format(EDGE_MODE, DEBOUNCE_US, PULSES_PER_REV, SETTLE_MS, TEST_INTERVAL_MS - SETTLE_MS))

        init_sms(CHUNK_N)

        for wheel in WHEELS:
            irq_chunks[wheel] = 0
            test_wheel_forward_only(wheel, CHUNK_N)
            sleep(0.5)

    stop_all()
    print("\nAll tests complete.")
    print("CSV saved to: {}".format(CSV_FILENAME))

    print("\n=== Feed-Forward Calibration Summary (Forward only) ===")
    for chunk in CHUNK_LIST:
        print(f"\n-- chunk = {chunk} --")
        print("Wheel | k (PWM/RPM) |  c (PWM) |  R²  | n pts | fit window (PWM)")
        print("---------------------------------------------------------------")
        for wheel in WHEELS:
            fwd = FITS.get((wheel, chunk, "FWD", EDGE_MODE))
            if not fwd:
                print("{:>5} |     ---     |    ---   |  --- |   -   |   ---".format(wheel))
                continue
            print("{:>5} | {:11.3f} | {:8.3f} | {:4.3f} | {:5d} | {}..{}"
                  .format(wheel, fwd["k"], fwd["c"], fwd["r2"], fwd["n"],
                          fwd["fit_min"], fwd["fit_max"]))

    # === Paste into main code: FF_PARAMS (FWD-only) ===
    chunk_for_export = CHUNK_LIST[-1] if CHUNK_LIST else None
    print("\n# === Paste into main code: FF_PARAMS (FWD-only) ===")
    if chunk_for_export is None:
        print("# No chunk selected; CHUNK_LIST is empty.")
    else:
        print("# Derived with CHUNK_N = {}  (edge: {})".format(chunk_for_export, EDGE_MODE))
        print("FF_PARAMS = {")
        for wheel in WHEELS:
            fwd = FITS.get((wheel, chunk_for_export, "FWD", EDGE_MODE))
            k_fwd = fwd["k"] if fwd else 0.0
            c_fwd = fwd["c"] if fwd else 0.0
            print('    "{}": {{ "FWD": {{"k": {:.3f}, "c": {:.3f}}} }},'.format(wheel, k_fwd, c_fwd))
        print("}")

main()
