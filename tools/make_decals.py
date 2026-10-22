#!/usr/bin/env python3
"""Calibration decals (Section 14.4) -> deploy/calibration_decals.pdf, printed at 100 % scale.

One page per robot: a FRONT and a REAR crosshair decal with their positions in the robot frame
(from config/robot_*.yaml calibration_decals). Stick them on the top plate at those positions;
the headset user touches the controller tip to each crosshair centre.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from eyesteer_core import config  # noqa: E402

A4 = (8.27, 11.69)
MM = 1 / 25.4


def decal(ax, cx, cy, label, sub):
    r = 30 * MM
    ax.add_patch(plt.Circle((cx, cy), r, fill=False, lw=2, color="#0b0b0b"))
    ax.add_patch(plt.Circle((cx, cy), r * 0.5, fill=False, lw=1, color="#0b0b0b"))
    ax.plot([cx - r * 1.2, cx + r * 1.2], [cy, cy], color="#0b0b0b", lw=1)
    ax.plot([cx, cx], [cy - r * 1.2, cy + r * 1.2], color="#0b0b0b", lw=1)
    ax.add_patch(plt.Circle((cx, cy), 1.2 * MM, color="#d03b3b"))
    ax.text(cx, cy + r * 1.45, label, ha="center", va="bottom", fontsize=22, weight="bold")
    ax.text(cx, cy - r * 1.45, sub, ha="center", va="top", fontsize=10, color="#52514e")
    # forward arrow (robot +x)
    ax.annotate("", xy=(cx + r * 1.6, cy - r * 0.9), xytext=(cx + r * 0.9, cy - r * 0.9),
                arrowprops=dict(arrowstyle="->", lw=1.5, color="#2a78d6"))
    ax.text(cx + r * 1.65, cy - r * 0.9, "+x (forward)", va="center", fontsize=8, color="#2a78d6")


def main() -> int:
    out = ROOT / "deploy" / "calibration_decals.pdf"
    with PdfPages(out) as pdf:
        for robot in ("field_robot", "walker"):
            d = config.robot(robot)["calibration_decals"]
            fig = plt.figure(figsize=A4)
            ax = fig.add_axes([0, 0, 1, 1])
            ax.set_xlim(0, A4[0])
            ax.set_ylim(0, A4[1])
            ax.set_aspect("equal")
            ax.axis("off")
            ax.text(A4[0] / 2, A4[1] - 0.7, f"eyesteer headset calibration decals: {robot}", ha="center", fontsize=14, weight="bold")
            ax.text(A4[0] / 2, A4[1] - 1.0, "Print at 100 % (no scaling). Check: the 60 mm circle must measure 60 mm.",
                    ha="center", fontsize=9, color="#52514e")
            for i, (key, lab) in enumerate((("front", "FRONT"), ("rear", "REAR"))):
                p = d[key]
                decal(ax, A4[0] / 2, A4[1] - 3.3 - i * 4.3, lab,
                      f"centre at x = {p['x']:+.3f} m, y = {p['y']:+.3f} m (robot frame), top plate at {d['plate_height']:.2f} m")
            ax.plot([0.6, 0.6 + 100 * MM], [0.8, 0.8], color="#0b0b0b", lw=2)
            ax.text(0.6, 0.9, "100 mm scale check", fontsize=8)
            pdf.savefig(fig)
            plt.close(fig)
    print(f"[decals] wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
