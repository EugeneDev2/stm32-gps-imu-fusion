#!/usr/bin/env python3
"""Raw GPS vs Kalman-filtered trajectory.

Usage: python tools/plot_kf.py [logs/some_log.csv]

Accepts these formats (auto-detected by column count):
  replay: t_ms,raw_east,raw_north,kf_east,kf_north
  live  : t_ms,ax,ay,az,gx,gy,gz,fix,east,north,spd,kf_east,kf_north,kf_vE,kf_vN[,gate[,roll,pitch,yaw]]
          (kf fields are empty until the filter starts - such rows keep NaN;
           with the gate column, rejected fixes are marked on the plot)
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def parse_field(s):
    return float(s) if s.strip() else float("nan")


def load(path):
    """Returns t_ms, raw_e, raw_n, kf_e, kf_n, gate (kf/gate may contain NaN)."""
    out = []
    with open(path, newline="") as f:
        for rec in csv.reader(f):
            if not rec or not rec[0][:1].isdigit():
                continue
            try:
                gate = float("nan")
                if len(rec) == 5:              # replay format
                    t, re_, rn_, ke, kn = (parse_field(v) for v in rec)
                elif len(rec) in (15, 16, 19): # live board (16 = +gate, 19 = +attitude)
                    t, re_, rn_ = float(rec[0]), float(rec[8]), float(rec[9])
                    ke, kn = parse_field(rec[11]), parse_field(rec[12])
                    if len(rec) >= 16:
                        gate = parse_field(rec[15])
                else:
                    continue
            except ValueError:
                continue
            out.append((t, re_, rn_, ke, kn, gate))
    if not out:
        sys.exit(f"no data rows found in {path}")
    return np.array(out)


def main():
    root = Path(__file__).resolve().parent.parent
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "logs" / "static_test_kf.csv"
    d = load(path)

    t = (d[:, 0] - d[0, 0]) / 1000.0
    re_, rn_, ke_, kn_, gate = d[:, 1], d[:, 2], d[:, 3], d[:, 4], d[:, 5]
    have_kf = ~np.isnan(ke_)

    ch = np.ones(len(re_), dtype=bool)
    ch[1:] = (np.diff(re_) != 0) | (np.diff(rn_) != 0)
    rejected = ch & (gate == 0)  # fix rows where the gate rejected the update

    print(f"rows: {len(d)}, duration: {t[-1]:.1f} s, fixes: {ch.sum()}, kf rows: {have_kf.sum()}")
    if not np.isnan(gate).all():
        print(f"gate: {int(rejected.sum())} of {int(ch.sum())} fixes rejected")
    print(f"raw std : east={re_[ch].std(ddof=1):.2f} m, north={rn_[ch].std(ddof=1):.2f} m")
    if have_kf.any():
        print(f"kf  std : east={np.nanstd(ke_, ddof=1):.2f} m, north={np.nanstd(kn_, ddof=1):.2f} m")
        step = np.hypot(np.diff(ke_[have_kf]), np.diff(kn_[have_kf]))
        print(f"kf max step between samples: {step.max():.2f} m")

    fig, axs = plt.subplots(1, 3, figsize=(16, 5.5))
    fig.suptitle(f"Raw GPS vs Kalman: {path.name}", fontsize=13)

    a = axs[0]
    a.scatter(re_[ch], rn_[ch], s=10, alpha=0.4, label="raw GPS fixes")
    if rejected.any():
        a.scatter(re_[rejected], rn_[rejected], s=30, marker="x", color="tab:orange",
                  label="rejected by gate")
    if have_kf.any():
        a.plot(ke_[have_kf], kn_[have_kf], lw=1.2, color="tab:red", label="Kalman")
    a.set_xlabel("east, m"); a.set_ylabel("north, m")
    a.set_title("trajectory"); a.set_aspect("equal", adjustable="datalim")
    a.legend(fontsize=8); a.grid(alpha=0.3)

    for a, raw, kf, name in ((axs[1], re_, ke_, "east"), (axs[2], rn_, kn_, "north")):
        a.plot(t, raw, lw=0.8, alpha=0.5, label=f"raw {name}")
        if have_kf.any():
            a.plot(t, kf, lw=1.2, color="tab:red", label=f"kf {name}")
        a.set_xlabel("t, s"); a.set_ylabel("m")
        a.set_title(f"{name}(t)"); a.legend(fontsize=8); a.grid(alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out_png = path.with_suffix(".png")
    fig.savefig(out_png, dpi=150)
    print(f"saved: {out_png}")


if __name__ == "__main__":
    main()
