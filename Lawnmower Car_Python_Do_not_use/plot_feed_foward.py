#!/usr/bin/env python3
# Standalone plotting for feed-forward sweeps (no csv/defaultdict required)
# Auto-finds the newest *_Feed_foward.csv in the current directory if --csv is omitted.

import argparse
import os
print("PYTHONPATH:", os.environ.get('PYTHONPATH'))
print("PATH:", os.environ.get('PATH'))
from collections_extended import bag
import matplotlib.pyplot as plt

WHEELS = ["FL", "FR", "BL", "BR"]
DEFAULT_MAX_PWM = 255
DEFAULT_PWM_STEP = 25
DEFAULT_MAX_WHEEL_RPM = 220.0

def find_latest_csv():
    """Return path to newest *_Feed_foward.csv in CWD, or None."""
    candidates = [f for f in os.listdir(".")
                  if f.endswith("_Feed_foward.csv") and os.path.isfile(f)]
    if not candidates:
        return None
    # pick by modification time
    candidates.sort(key=lambda f: os.path.getmtime(f), reverse=True)
    return candidates[0]

def load_feedforward_csv(csv_path):
    """
    Loads CSV into:
      samples[(wheel, chunk, direction)] = list of dicts
      fits[(wheel, chunk, direction)]    = dict
      meta = {"chunks_available": [...]}
    """
    samples = {}
    fits = {}
    chunks_available = bag()

    with open(csv_path, 'r') as f:
        header_line = f.readline()
        if not header_line:
            return samples, fits, {"chunks_available": []}
        headers = [h.strip() for h in header_line.strip().split(',')]

        for line in f:
            parts = [p.strip() for p in line.strip().split(',')]
            if len(parts) != len(headers):
                continue
            row = dict(zip(headers, parts))

            rtype = row.get("record_type", "")
            wheel = row.get("wheel", "")
            direction = row.get("direction", "")
            chunk_str = row.get("chunk", "")
            if not (rtype and wheel and direction and chunk_str):
                continue
            try:
                chunk = int(float(chunk_str))
            except:
                continue

            chunks_available.add(chunk)
            key = (wheel, chunk, direction)

            if rtype == "sample":
                try:
                    pwm = int(float(row["pwm"]))
                    dpulses = int(float(row["dpulses"]))
                    rpm_meas = float(row["rpm_meas"])
                    rpm_exp  = float(row["rpm_exp"])
                except:
                    continue
                samples.setdefault(key, []).append({
                    "pwm": pwm,
                    "dpulses": dpulses,
                    "rpm_meas": rpm_meas,
                    "rpm_exp": rpm_exp
                })

            elif rtype == "fit":
                try:
                    k = float(row["k"]); c = float(row["c"]); r2 = float(row["r2"])
                    n = int(float(row["n"]))
                    fit_min = int(float(row.get("fit_min", 0)))
                    fit_max = int(float(row.get("fit_max", 0)))
                except:
                    continue
                fits[key] = {
                    "k": k, "c": c, "r2": r2, "n": n,
                    "fit_min": fit_min, "fit_max": fit_max
                }

    # Sort each wheel's samples by PWM
    for key in samples:
        samples[key].sort(key=lambda d: d["pwm"])

    meta = {"chunks_available": sorted(set(chunks_available))}
    return samples, fits, meta

def plot_wheel(ax, wheel, chunk, samples, fits,
               max_pwm=DEFAULT_MAX_PWM, pwm_step=DEFAULT_PWM_STEP, max_rpm=DEFAULT_MAX_WHEEL_RPM):
    fwd = samples.get((wheel, chunk, "FWD"), [])
    rev = samples.get((wheel, chunk, "REV"), [])

    xp_fwd = [d["pwm"] for d in fwd]
    xp_rev = [d["pwm"] for d in rev]
    y_fwd = [d["rpm_meas"] for d in fwd]
    y_rev = [d["rpm_meas"] for d in rev]

    # Expected curves
    xexp = list(range(0, max_pwm + 1, pwm_step))
    yexp_f = [(p / max_pwm) * max_rpm for p in xexp]
    yexp_r = [-(p / max_pwm) * max_rpm for p in xexp]

    # Plot measured
    if xp_fwd:
        ax.plot(xp_fwd, y_fwd, label="Measured FWD")
    if xp_rev:
        ax.plot(xp_rev, y_rev, label="Measured REV")

    # Plot expected (dashed)
    ax.plot(xexp, yexp_f, linestyle="--", label="Expected FWD")
    ax.plot(xexp, yexp_r, linestyle="--", label="Expected REV")

    # Fit lines: PWM = c + k*RPM  => RPM = (PWM - c)/k
    def plot_fit(fit, label):
        if not fit:
            return
        k, c = fit["k"], fit["c"]
        if abs(k) < 1e-9:
            return
        fmin = fit.get("fit_min", 0); fmax = fit.get("fit_max", max_pwm)
        xs = list(range(int(fmin), int(fmax) + 1, pwm_step))
        ys = [(x - c) / k for x in xs]
        ax.plot(xs, ys, label=f"{label} fit (R²={fit['r2']:.2f})")

    plot_fit(fits.get((wheel, chunk, "FWD")), "FWD")
    plot_fit(fits.get((wheel, chunk, "REV")), "REV")

    ax.set_title(f"{wheel}  (chunk={chunk})")
    ax.set_xlabel("PWM")
    ax.set_ylabel("RPM")
    ax.grid(True)
    ax.legend(loc="best", fontsize=8)

def pick_chunk(meta, user_chunk):
    if user_chunk is not None:
        return user_chunk
    chunks = meta.get("chunks_available", [])
    if not chunks:
        return None
    return chunks[-1]  # highest/last

def main():
    ap = argparse.ArgumentParser(description="Plot feed-forward sweep CSV from Pico.")
    ap.add_argument("--csv", help="Path to *_Feed_foward.csv (default: newest in current folder)")
    ap.add_argument("--chunk", type=int, help="PIO chunk to plot (default: auto from CSV)")
    ap.add_argument("--pwm-step", type=int, default=DEFAULT_PWM_STEP, help="PWM step for expected curves")
    ap.add_argument("--max-rpm", type=float, default=DEFAULT_MAX_WHEEL_RPM, help="Expected max RPM for reference")
    ap.add_argument("--no-save", action="store_true", help="Do not save PNG")
    args = ap.parse_args()

    csv_path = args.csv or find_latest_csv()
    if not csv_path:
        print("No --csv provided and no *_Feed_foward.csv found in the current directory.")
        return
    if not os.path.isfile(csv_path):
        print("CSV not found:", csv_path)
        return

    print("Using CSV:", csv_path)
    samples, fits, meta = load_feedforward_csv(csv_path)
    if not samples:
        print("No sample data in CSV.")
        return

    chunk = pick_chunk(meta, args.chunk)
    if chunk is None:
        print("No chunk info found in CSV; please specify --chunk.")
        return

    # 2x2 plot
    fig, axs = plt.subplots(2, 2, figsize=(10, 7), sharex=True, sharey=True)
    axs = axs.ravel()
    for i, wheel in enumerate(WHEELS):
        plot_wheel(axs[i], wheel, chunk, samples, fits,
                   max_pwm=DEFAULT_MAX_PWM, pwm_step=args.pwm_step, max_rpm=args.max_rpm)

    fig.suptitle(f"Wheel RPM vs PWM — Measured (FWD/REV) vs Expected  |  chunk={chunk}")
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    if not args.no_save:
        out_png = os.path.splitext(csv_path)[0] + ".png"
        try:
            plt.savefig(out_png, dpi=150)
            print("Saved plot:", out_png)
        except Exception as e:
            print("Failed to save plot:", e)

    try:
        plt.show()
    except Exception:
        pass

if __name__ == "__main__":
    main()
