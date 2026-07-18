#!/usr/bin/env python3
"""Before/after comparison of innovation gating on the same log.

Usage: python tools/plot_gate_compare.py <nogate.csv> <gate.csv> [out.png]
Both inputs are replay outputs: t_ms,raw_east,raw_north,kf_east,kf_north.
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load(path):
    rows = []
    with open(path, newline="") as f:
        for rec in csv.reader(f):
            if len(rec) == 5 and rec[0][:1].isdigit():
                try:
                    rows.append([float(v) for v in rec])
                except ValueError:
                    continue
    return np.array(rows)


def main():
    if len(sys.argv) < 3:
        sys.exit("usage: plot_gate_compare.py <nogate.csv> <gate.csv> [out.png]")
    a, b = load(Path(sys.argv[1])), load(Path(sys.argv[2]))
    out = Path(sys.argv[3]) if len(sys.argv) > 3 else Path(sys.argv[2]).with_name("gate_compare.png")

    t = (a[:, 0] - a[0, 0]) / 1000.0
    re_, rn_ = a[:, 1], a[:, 2]
    ke0, kn0 = a[:, 3], a[:, 4]   # no gate
    ke1, kn1 = b[:, 3], b[:, 4]   # gated

    ch = np.ones(len(re_), dtype=bool)
    ch[1:] = (np.diff(re_) != 0) | (np.diff(rn_) != 0)

    for name, ke, kn in (("no gate", ke0, kn0), ("gated", ke1, kn1)):
        step = np.hypot(np.diff(ke), np.diff(kn))
        print(f"{name:8s}: kf std east={ke.std(ddof=1):6.2f} m north={kn.std(ddof=1):5.2f} m, "
              f"max step={step.max():6.2f} m")

    fig, axs = plt.subplots(1, 3, figsize=(16, 5.5))
    fig.suptitle("Innovation gating: before vs after (same log)", fontsize=13)

    ax = axs[0]
    ax.scatter(re_[ch], rn_[ch], s=10, alpha=0.3, label="raw GPS fixes")
    ax.plot(ke0, kn0, lw=1.0, color="tab:orange", alpha=0.8, label="kf no gate")
    ax.plot(ke1, kn1, lw=1.3, color="tab:red", label="kf gated")
    ax.set_xlabel("east, m"); ax.set_ylabel("north, m")
    ax.set_title("trajectory"); ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    for ax, raw, k0, k1, name in ((axs[1], re_, ke0, ke1, "east"),
                                  (axs[2], rn_, kn0, kn1, "north")):
        ax.plot(t, raw, lw=0.7, alpha=0.4, label=f"raw {name}")
        ax.plot(t, k0, lw=0.9, color="tab:orange", alpha=0.8, label="kf no gate")
        ax.plot(t, k1, lw=1.2, color="tab:red", label="kf gated")
        ax.set_xlabel("t, s"); ax.set_ylabel("m")
        ax.set_title(f"{name}(t)"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out, dpi=150)
    print(f"saved: {out}")


if __name__ == "__main__":
    main()
