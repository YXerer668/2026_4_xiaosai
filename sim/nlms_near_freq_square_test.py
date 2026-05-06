"""NLMS simulation for near-frequency sine + square-wave reference cancellation.

Scenario:
- desired signal: 100 Hz sine
- interference: 110 Hz square wave after a small delay/smoothing path
- reference input: 110 Hz square wave
- mixed input: desired + interference

The key comparison is ordinary sample-by-sample NLMS versus using a 100 Hz
notch-filtered error only for the NLMS weight update. The output remains the
raw NLMS error, so the desired 100 Hz sine is not filtered out of the output.
"""

import math
import numpy as np

FS = 50_000.0
DURATION_S = 5.0
TAPS = 96
F_DESIRED = 100.0
F_REF = 110.0
A_DESIRED = 0.5
A_INTERFERENCE = 1.0


def make_signals(delay_samples=12, tau_samples=2.0):
    n = np.arange(int(FS * DURATION_S), dtype=np.float64)
    t = n / FS
    desired = A_DESIRED * np.sin(2.0 * np.pi * F_DESIRED * t)
    ref = np.where(np.sin(2.0 * np.pi * F_REF * t) >= 0.0, 1.0, -1.0)

    if delay_samples > 0:
        delayed = np.concatenate([np.zeros(delay_samples), ref[:-delay_samples]])
    else:
        delayed = ref.copy()

    if tau_samples > 0.0:
        alpha = 1.0 / (tau_samples + 1.0)
        interference = np.empty_like(delayed)
        y = 0.0
        for i, x in enumerate(delayed):
            y += alpha * (x - y)
            interference[i] = y
    else:
        interference = delayed

    interference *= A_INTERFERENCE
    mixed = desired + interference
    return desired, ref, interference, mixed


def run_nlms(mixed, ref, mu=0.0003):
    w = np.zeros(TAPS, dtype=np.float64)
    xhist = np.zeros(TAPS, dtype=np.float64)
    out = np.empty_like(mixed)

    for i, x in enumerate(ref):
        xhist[1:] = xhist[:-1]
        xhist[0] = x
        y = float(np.dot(w, xhist))
        e = mixed[i] - y
        norm = float(np.dot(xhist, xhist)) + 1e-6
        w += (mu * e / norm) * xhist
        out[i] = e

    return out


def run_nlms_update_error_notch(mixed, ref, mu=0.03, f_notch=100.0, r=0.995):
    """Use notched error only for adaptation. Output remains raw e."""
    w = np.zeros(TAPS, dtype=np.float64)
    xhist = np.zeros(TAPS, dtype=np.float64)
    out = np.empty_like(mixed)

    c = math.cos(2.0 * math.pi * f_notch / FS)
    b0, b1, b2 = 1.0, -2.0 * c, 1.0
    a1, a2 = -2.0 * r * c, r * r
    x1 = x2 = y1 = y2 = 0.0

    for i, x in enumerate(ref):
        xhist[1:] = xhist[:-1]
        xhist[0] = x
        y = float(np.dot(w, xhist))
        e = mixed[i] - y

        # Notch-filtered error for weight update only.
        e_update = b0 * e + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
        x2, x1 = x1, e
        y2, y1 = y1, e_update

        norm = float(np.dot(xhist, xhist)) + 1e-6
        w += (mu * e_update / norm) * xhist
        out[i] = e

    return out


def measure(output, desired, start_s=2.5):
    start = int(start_s * FS)
    e = output[start:]
    d = desired[start:]
    residual = e - d

    n = np.arange(start, len(output), dtype=np.float64)
    t = n / FS
    s = np.sin(2.0 * np.pi * F_DESIRED * t)
    c = np.cos(2.0 * np.pi * F_DESIRED * t)
    amp = math.hypot(2.0 * np.mean(e * s), 2.0 * np.mean(e * c))

    desired_rms = math.sqrt(float(np.mean(d * d)))
    residual_rms = math.sqrt(float(np.mean(residual * residual)))
    snr_db = 20.0 * math.log10((desired_rms + 1e-15) / (residual_rms + 1e-15))
    return amp, residual_rms, snr_db


def main():
    print("delay,method,desired_amp_100Hz,residual_rms,SNR_dB")
    for delay in [0, 12, 60, 90, 110]:
        desired, ref, _interference, mixed = make_signals(delay_samples=delay)
        cases = [
            ("baseline_mu0.0003", run_nlms(mixed, ref, mu=0.0003)),
            ("notch_update_mu0.03", run_nlms_update_error_notch(mixed, ref, mu=0.03)),
            ("notch_update_mu0.1", run_nlms_update_error_notch(mixed, ref, mu=0.1)),
        ]
        for name, output in cases:
            amp, residual_rms, snr_db = measure(output, desired)
            print(f"{delay},{name},{amp:.4f},{residual_rms:.4f},{snr_db:.2f}")


if __name__ == "__main__":
    main()
