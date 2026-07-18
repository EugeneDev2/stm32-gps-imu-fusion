#!/usr/bin/env python3
"""Raw GPS vs Kalman-filtered trajectory from a replay output CSV.

Usage: python tools/plot_kf.py [logs/static_test_kf.csv]
Input: t_ms,raw_east,raw_north,kf_east,kf_north (header line present).
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    root = Path(__file__).resolve().parent.parent
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "logs" / "static_test_kf.csv"

    rows = []
    with open(path, newline="") as f:
        for rec in csv.reader(f):
            if len(rec) != 5 or not rec[0][:1].isdigit():
                continue
            rows.append([float(v) for v in rec])
    d = np.array(rows)
    t = (d[:, 0] - d[0, 0]) / 1000.0
    re_, rn_, ke_, kn_ = d[:, 1], d[:, 2], d[:, 3], d[:, 4]

    # unique raw fixes for the scatter
    ch = np.ones(len(re_), dtype=bool)
    ch[1:] = (np.diff(re_) != 0) | (np.diff(rn_) != 0)

    print(f"rows: {len(d)}, duration: {t[-1]:.1f} s, fixes: {ch.sum()}")
    print(f"raw std : east={re_[ch].std(ddof=1):.2f} m, north={rn_[ch].std(ddof=1):.2f} m")
    print(f"kf  std : east={ke_.std(ddof=1):.2f} m, north={kn_.std(ddof=1):.2f} m")
    step = np.hypot(np.diff(ke_), np.diff(kn_))
    print(f"kf max step between samples: {step.max():.2f} m")

    fig, axs = plt.subplots(1, 3, figsize=(16, 5.5))
    fig.suptitle(f"Raw GPS vs Kalman: {path.name}", fontsize=13)

    a = axs[0]
    a.scatter(re_[ch], rn_[ch], s=10, alpha=0.4, label="raw GPS fixes")
    a.plot(ke_, kn_, lw=1.2, color="tab:red", label="Kalman")
    a.set_xlabel("east, m"); a.set_ylabel("north, m")
    a.set_title("trajectory"); a.set_aspect("equal", adjustable="datalim")
    a.legend(fontsize=8); a.grid(alpha=0.3)

    a = axs[1]
    a.plot(t, re_, lw=0.8, alpha=0.5, label="raw east")
    a.plot(t, ke_, lw=1.2, color="tab:red", label="kf east")
    a.set_xlabel("t, s"); a.set_ylabel("m")
    a.set_title("east(t)"); a.legend(fontsize=8); a.grid(alpha=0.3)

    a = axs[2]
    a.plot(t, rn_, lw=0.8, alpha=0.5, label="raw north")
    a.plot(t, kn_, lw=1.2, color="tab:red", label="kf north")
    a.set_xlabel("t, s"); a.set_ylabel("m")
    a.set_title("north(t)"); a.legend(fontsize=8); a.grid(alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out_png = path.with_suffix(".png")
    fig.savefig(out_png, dpi=150)
    print(f"saved: {out_png}")


if __name__ == "__main__":
    main()
