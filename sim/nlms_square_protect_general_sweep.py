"""General simulation for low-frequency square-reference NLMS protection.

This script models the firmware-side strategy added in
Core/Src/nlms_square_protect.c:

- The output remains the raw NLMS error: e = mix - y.
- When the reference is a low-frequency square wave, the NLMS weight update uses
  a notch-filtered error, e_update = notch(f_protect, e).
- f_protect is the desired/useful signal frequency estimate, not a hard-coded
  100 Hz special case.

The script sweeps desired frequency, square-reference frequency, and delay.
"""

import math
from pathlib import Path

import numpy as np
import pandas as pd

try:
    from numba import njit
except Exception as exc:  # pragma: no cover
    raise RuntimeError("This sweep expects numba for speed") from exc

FS = 50_000.0
DURATION_S = 4.0
N = int(FS * DURATION_S)
TAPS = 96
A_DESIRED = 0.5
A_INTERFERENCE = 1.0
BASE_MU = 0.0003
SQUARE_MU = 0.01
NOTCH_R = 0.995


@njit
def make_sine(freq, n, fs):
    out = np.empty(n, dtype=np.float64)
    for i in range(n):
        out[i] = math.sin(2.0 * math.pi * freq * i / fs)
    return out


@njit
def make_square(freq, n, fs):
    out = np.empty(n, dtype=np.float64)
    for i in range(n):
        out[i] = 1.0 if math.sin(2.0 * math.pi * freq * i / fs) >= 0.0 else -1.0
    return out


@njit
def delay_smooth(x, delay_samples, tau_samples):
    n = len(x)
    delayed = np.zeros(n, dtype=np.float64)
    for i in range(n):
        j = i - delay_samples
        if j >= 0:
            delayed[i] = x[j]

    out = np.empty(n, dtype=np.float64)
    if tau_samples > 0.0:
        alpha = 1.0 / (tau_samples + 1.0)
        y = 0.0
        for i in range(n):
            y += alpha * (delayed[i] - y)
            out[i] = y
    else:
        for i in range(n):
            out[i] = delayed[i]
    return out


@njit
def run_nlms_square_protect(mixed, ref, f_protect, square_mode):
    w = np.zeros(TAPS, dtype=np.float64)
    xhist = np.zeros(TAPS, dtype=np.float64)
    out = np.empty(len(mixed), dtype=np.float64)

    c = math.cos(2.0 * math.pi * f_protect / FS)
    b0 = 1.0
    b1 = -2.0 * c
    b2 = 1.0
    a1 = -2.0 * NOTCH_R * c
    a2 = NOTCH_R * NOTCH_R

    x1 = x2 = y1 = y2 = 0.0

    for i in range(len(mixed)):
        for k in range(TAPS - 1, 0, -1):
            xhist[k] = xhist[k - 1]
        xhist[0] = ref[i]

        y = 0.0
        norm = 1e-6
        for k in range(TAPS):
            y += w[k] * xhist[k]
            norm += xhist[k] * xhist[k]

        e = mixed[i] - y
        e_update = e
        mu = BASE_MU

        if square_mode:
            e_update = b0 * e + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
            x2 = x1
            x1 = e
            y2 = y1
            y1 = e_update
            mu = SQUARE_MU

        fac = mu * e_update / norm
        for k in range(TAPS):
            w[k] += fac * xhist[k]

        out[i] = e

    return out


@njit
def measure(output, desired, f_desired, start_idx):
    res_sum = 0.0
    des_sum = 0.0
    es = 0.0
    ec = 0.0
    cnt = 0

    for i in range(start_idx, len(output)):
        e = output[i]
        d = desired[i]
        r = e - d
        res_sum += r * r
        des_sum += d * d

        phase = 2.0 * math.pi * f_desired * i / FS
        es += e * math.sin(phase)
        ec += e * math.cos(phase)
        cnt += 1

    residual_rms = math.sqrt(res_sum / cnt)
    desired_rms = math.sqrt(des_sum / cnt)
    amp = math.sqrt((2.0 * es / cnt) ** 2 + (2.0 * ec / cnt) ** 2)
    snr_db = 20.0 * math.log10((desired_rms + 1e-15) / (residual_rms + 1e-15))
    return amp, residual_rms, snr_db


def simulate_case(f_desired, f_ref, delay_samples, square_mode):
    desired = A_DESIRED * make_sine(float(f_desired), N, FS)
    ref = make_square(float(f_ref), N, FS)
    interference = A_INTERFERENCE * delay_smooth(ref, int(delay_samples), 2.0)
    mixed = desired + interference
    out = run_nlms_square_protect(mixed, ref, float(f_desired), bool(square_mode))
    return measure(out, desired, float(f_desired), int(2.0 * FS))


def main():
    # Warm-up numba compilation.
    simulate_case(100, 110, 12, True)

    scenarios = []
    for f_desired in [50, 80, 100, 150, 200, 300, 500]:
        scenarios.append((f_desired, f_desired + 10))
    for f_desired, f_ref in [
        (100, 50),
        (100, 200),
        (100, 500),
        (200, 80),
        (500, 110),
        (500, 300),
        (1000, 300),
    ]:
        scenarios.append((f_desired, f_ref))
    scenarios = list(dict.fromkeys(scenarios))

    delays = [0, 12, 60, 90, 110, 160, 240]
    rows = []
    for f_desired, f_ref in scenarios:
        for delay in delays:
            for mode_name, square_mode in [
                ("baseline", False),
                ("square_protect", True),
            ]:
                amp, residual, snr = simulate_case(f_desired, f_ref, delay, square_mode)
                rows.append(
                    {
                        "f_desired": f_desired,
                        "f_ref_square": f_ref,
                        "delay_samples": delay,
                        "delay_ms": delay / FS * 1000.0,
                        "mode": mode_name,
                        "desired_amp": amp,
                        "residual_rms": residual,
                        "snr_db": snr,
                    }
                )

    df = pd.DataFrame(rows)
    base = df[df["mode"] == "baseline"].set_index(
        ["f_desired", "f_ref_square", "delay_samples"]
    )
    deltas = []
    for _, row in df[df["mode"] == "square_protect"].iterrows():
        key = (row["f_desired"], row["f_ref_square"], row["delay_samples"])
        base_row = base.loc[key]
        deltas.append(
            {
                **row.to_dict(),
                "delta_snr_vs_baseline": row["snr_db"] - base_row["snr_db"],
                "delta_amp_vs_baseline": row["desired_amp"] - base_row["desired_amp"],
            }
        )
    delta_df = pd.DataFrame(deltas)

    summary = {
        "total_cases": len(delta_df),
        "good_cases_snr_gt_20dB": int((delta_df["snr_db"] > 20.0).sum()),
        "improved_cases_gt_3dB": int((delta_df["delta_snr_vs_baseline"] > 3.0).sum()),
        "regression_cases": int(
            ((delta_df["delta_snr_vs_baseline"] < -3.0)
             | (delta_df["delta_amp_vs_baseline"].abs() > 0.05)).sum()
        ),
        "median_snr_db": float(delta_df["snr_db"].median()),
        "worst_snr_db": float(delta_df["snr_db"].min()),
        "best_snr_db": float(delta_df["snr_db"].max()),
    }

    print("square_protect summary:")
    for k, v in summary.items():
        print(f"{k},{v}")

    print("\nWorst square_protect cases:")
    print(
        delta_df.sort_values("snr_db")
        .head(10)
        .to_string(index=False, float_format=lambda x: f"{x:.4f}")
    )

    print("\nBest square_protect cases:")
    print(
        delta_df.sort_values("delta_snr_vs_baseline", ascending=False)
        .head(10)
        .to_string(index=False, float_format=lambda x: f"{x:.4f}")
    )

    out_dir = Path(__file__).resolve().parent
    df.to_csv(out_dir / "nlms_square_protect_full.csv", index=False)
    delta_df.to_csv(out_dir / "nlms_square_protect_delta.csv", index=False)


if __name__ == "__main__":
    main()
