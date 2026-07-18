#!/usr/bin/env python3
"""Static telemetry log analysis: GPS noise cloud + IMU stability.

Usage: python tools/plot_static.py [logs/static_test.csv]

Log format: service lines (skipped), header line, then CSV rows
t_ms,ax,ay,az,gx,gy,gz,fix,east,north,spd at ~10 Hz.
GPS updates at 1 Hz, so east/north repeat between fixes; only rows where
(east, north) change are treated as real fix samples.
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

N_COLS = 11
OUTLIER_SIGMA = 3.0


def load_log(path):
    rows = []
    with open(path, newline="") as f:
        for rec in csv.reader(f):
            if len(rec) != N_COLS or not rec[0][:1].isdigit():
                continue  # service lines, header, torn lines
            try:
                rows.append([float(v) for v in rec])
            except ValueError:
                continue
    if not rows:
        sys.exit(f"no data rows found in {path}")
    return np.array(rows)


def dedup_fixes(east, north, t):
    """Indices of rows where (east, north) changed vs previous row."""
    changed = np.ones(len(east), dtype=bool)
    changed[1:] = (np.diff(east) != 0) | (np.diff(north) != 0)
    return np.flatnonzero(changed)


def stats_block(name, e, n):
    de, dn = e - e.mean(), n - n.mean()
    dist = np.hypot(de, dn)
    cep50 = np.median(dist)
    print(f"--- {name} ({len(e)} fixes) ---")
    print(f"  east : std={e.std(ddof=1):6.3f} m  min={e.min():8.3f}  max={e.max():8.3f}")
    print(f"  north: std={n.std(ddof=1):6.3f} m  min={n.min():8.3f}  max={n.max():8.3f}")
    print(f"  CEP50 (median dist from mean): {cep50:.3f} m")
    return cep50


def main():
    root = Path(__file__).resolve().parent.parent
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "logs" / "static_test.csv"
    data = load_log(path)

    t = (data[:, 0] - data[0, 0]) / 1000.0  # seconds from log start
    ax_, ay_, az_ = data[:, 1], data[:, 2], data[:, 3]
    gx_, gy_, gz_ = data[:, 4], data[:, 5], data[:, 6]
    east, north = data[:, 8], data[:, 9]

    fix_idx = dedup_fixes(east, north, t)
    fe, fn, ft = east[fix_idx], north[fix_idx], t[fix_idx]

    print(f"rows: {len(data)}, duration: {t[-1]:.1f} s, unique fixes: {len(fe)}")
    print()
    stats_block("all fixes", fe, fn)

    # 3-sigma outliers (the "board was moved" spike and similar)
    de, dn = fe - fe.mean(), fn - fn.mean()
    keep = (np.abs(de) <= OUTLIER_SIGMA * fe.std(ddof=1)) & (
        np.abs(dn) <= OUTLIER_SIGMA * fn.std(ddof=1))
    out = ~keep
    print()
    stats_block(f"filtered (>{OUTLIER_SIGMA:.0f} sigma removed: {out.sum()} pts)",
                fe[keep], fn[keep])
    if out.any():
        print(f"  outlier time(s): " +
              ", ".join(f"{v:.0f}s" for v in ft[out][:10]) +
              (" ..." if out.sum() > 10 else ""))

    print()
    print("--- IMU (all rows) ---")
    print(f"  accel std [m/s^2]: ax={ax_.std(ddof=1):.4f}  ay={ay_.std(ddof=1):.4f}  az={az_.std(ddof=1):.4f}")
    print(f"  accel mean       : ax={ax_.mean():.3f}  ay={ay_.mean():.3f}  az={az_.mean():.3f}")
    print(f"  gyro  std [rad/s]: gx={gx_.std(ddof=1):.5f}  gy={gy_.std(ddof=1):.5f}  gz={gz_.std(ddof=1):.5f}")

    # ---- figure ----
    fig, axs = plt.subplots(2, 3, figsize=(16, 9))
    fig.suptitle(f"Static GPS/IMU noise: {path.name}", fontsize=13)

    a = axs[0, 0]
    a.scatter(fe[keep], fn[keep], s=8, alpha=0.6, label="fixes")
    if out.any():
        a.scatter(fe[out], fn[out], s=14, color="red", label="outliers (>3σ)")
    a.plot(fe.mean(), fn.mean(), "k+", markersize=14, markeredgewidth=2, label="mean")
    a.set_xlabel("east, m"); a.set_ylabel("north, m")
    a.set_title("GPS noise cloud"); a.set_aspect("equal", adjustable="datalim")
    a.legend(fontsize=8); a.grid(alpha=0.3)

    a = axs[0, 1]
    a.plot(ft, fe, label="east", lw=1)
    a.plot(ft, fn, label="north", lw=1)
    if out.any():
        for v in ft[out]:
            a.axvline(v, color="red", alpha=0.25, lw=3)
    a.set_xlabel("t, s"); a.set_ylabel("m")
    a.set_title("east(t), north(t)" + (" — red: outliers" if out.any() else ""))
    a.legend(fontsize=8); a.grid(alpha=0.3)

    a = axs[0, 2]
    a.hist(fe, bins=30, alpha=0.6, label="east")
    a.hist(fn, bins=30, alpha=0.6, label="north")
    a.set_xlabel("m"); a.set_ylabel("count")
    a.set_title("east / north histograms"); a.legend(fontsize=8); a.grid(alpha=0.3)

    a = axs[1, 0]
    for arr, lbl in ((ax_, "ax"), (ay_, "ay"), (az_, "az")):
        a.plot(t, arr, lw=0.6, label=lbl)
    a.set_xlabel("t, s"); a.set_ylabel("m/s^2")
    a.set_title("accelerometer"); a.legend(fontsize=8); a.grid(alpha=0.3)

    a = axs[1, 1]
    for arr, lbl in ((gx_, "gx"), (gy_, "gy"), (gz_, "gz")):
        a.plot(t, arr, lw=0.6, label=lbl)
    a.set_xlabel("t, s"); a.set_ylabel("rad/s")
    a.set_title("gyroscope"); a.legend(fontsize=8); a.grid(alpha=0.3)

    a = axs[1, 2]
    dist = np.hypot(fe - fe[keep].mean(), fn - fn[keep].mean())
    a.plot(ft, dist, lw=1, color="tab:purple")
    a.set_xlabel("t, s"); a.set_ylabel("m")
    a.set_title("distance from mean (per fix)"); a.grid(alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out_png = path.with_suffix(".png")
    fig.savefig(out_png, dpi=150)
    print(f"\nsaved: {out_png}")


if __name__ == "__main__":
    main()
