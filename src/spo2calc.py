# -*- coding: utf-8 -*-
"""
Lightweight SpO2 estimator with quality diagnostics for MAX30102.
Returns (spo2, diag), where diag contains:
  - quality_score (0..100)
  - ac_rms_ir
  - peaks
"""

import math

def _moving_average(x, w):
    out = []
    acc = 0.0
    for i, v in enumerate(x):
        acc += v
        if i >= w:
            acc -= x[i - w]
        out.append(acc / float(min(i + 1, w)))
    return out

def _ac(signal, ma):
    return [signal[i] - ma[i] for i in range(len(signal))]

def _rms(x):
    if not x:
        return 0.0
    return math.sqrt(sum(v*v for v in x)/len(x))

def _count_peaks(x, fs, thr_frac=0.15, refractory_s=0.30, min_height=500):
    rng = max(x) - min(x) if x else 0
    thr = max(min_height, int(thr_frac * rng))
    refr = int(refractory_s * fs)
    peaks, last = 0, -refr
    for i in range(1, len(x)-1):
        if x[i] > thr and x[i] > x[i-1] and x[i] >= x[i+1] and (i - last) > refr:
            peaks += 1
            last = i
    return peaks

def estimate_spo2(red, ir, fs=100):
    n = min(len(red), len(ir))
    if n < int(2*fs):
        return None, {"quality_score": 0, "ac_rms_ir": 0.0, "peaks": 0}

    # DC removal
    ma_win = max(1, int(0.8 * fs))
    ir_ma  = _moving_average(ir[:n], ma_win)
    red_ma = _moving_average(red[:n], ma_win)
    ir_ac  = _ac(ir[:n],  ir_ma)
    red_ac = _ac(red[:n], red_ma)

    ac_ir_rms  = _rms(ir_ac)
    ac_red_rms = _rms(red_ac)

    # R ratio with simple clipping
    if ac_ir_rms < 1e-6:
        return None, {"quality_score": 0, "ac_rms_ir": ac_ir_rms, "peaks": 0}
    R = (ac_red_rms / ac_ir_rms)

    # Empirical MAX30102 cal: SpO2 ≈ 110 - 25*R (clip 70..100)
    spo2 = max(70.0, min(100.0, 110.0 - 25.0 * R))

    # Peaks for plausibility
    peaks = _count_peaks(ir_ac, fs)

    # Quality: blend of AC strength and peaks count
    q_ac    = max(0.0, min(100.0, (ac_ir_rms / 200.0) * 100.0))  # scale 0..~200
    q_peaks = 100.0 if peaks >= 3 else (50.0 if peaks == 2 else 10.0 if peaks == 1 else 0.0)
    quality = int(0.6 * q_ac + 0.4 * q_peaks)
    quality = max(0, min(100, quality))

    return round(spo2, 1), {"quality_score": quality, "ac_rms_ir": round(ac_ir_rms, 1), "peaks": peaks}
