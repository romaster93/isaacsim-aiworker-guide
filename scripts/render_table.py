#!/usr/bin/env python3
"""Render Table 1 (run-level metrics) to PNG from trajectory .npz files."""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

OUT_DIR = Path("/home/cho/ms_AIworker/rosbags")

plt.rcParams["font.family"] = "serif"

PAIRS = [
    ("run1", "ackermann_run1.npz", "swerve_run1.npz"),
    ("run2", "ackermann_run2.npz", "swerve_run2.npz"),
    ("run3", "ackermann_run3.npz", "swerve_run3.npz"),
]


def stats(npz_path: Path) -> tuple[float, float, float]:
    d = np.load(npz_path, allow_pickle=True)
    traj = d["traj"]
    t, x, y = traj[:, 0], traj[:, 1], traj[:, 2]
    dist = float(np.sum(np.hypot(np.diff(x), np.diff(y))))
    dur = float(t[-1] - t[0])
    v = dist / dur if dur > 0 else 0.0
    return dist, dur, v


def build_rows() -> list[list[str]]:
    rows: list[list[str]] = []
    for run, ack, swv in PAIRS:
        for method, npz in [("Ackermann", ack), ("Swerve", swv)]:
            p = OUT_DIR / npz
            if not p.exists():
                rows.append([run, method, "-", "-", "-"])
                continue
            d, t, v = stats(p)
            rows.append([run, method, f"{d:.2f}", f"{t:.2f}", f"{v:.3f}"])
    return rows


def render_table(rows: list[list[str]], out_path: Path, dpi: int = 400) -> None:
    headers = ["Run", "Method", "Distance [m]", "Time [s]", r"$v_{\mathrm{avg}}$ [m/s]"]
    col_widths = [0.10, 0.18, 0.22, 0.18, 0.22]
    fig, ax = plt.subplots(figsize=(6.5, 0.38 * (len(rows) + 1) + 0.2))
    ax.axis("off")
    tbl = ax.table(
        cellText=rows,
        colLabels=headers,
        colWidths=col_widths,
        loc="center",
        cellLoc="center",
        colLoc="center",
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    tbl.scale(1.0, 1.3)

    # header styling
    for col in range(len(headers)):
        cell = tbl[0, col]
        cell.set_facecolor("#e8e8e8")
        cell.set_text_props(weight="bold")

    # bold the winning (Swerve) method cells
    for row_idx, row in enumerate(rows, start=1):
        if row[1] == "Swerve":
            for col in range(len(headers)):
                tbl[row_idx, col].set_text_props(weight="bold")

    fig.savefig(str(out_path), dpi=dpi, bbox_inches="tight",
                pad_inches=0.1, transparent=True)
    plt.close(fig)
    print(f"saved: {out_path}")


if __name__ == "__main__":
    rows = build_rows()
    render_table(rows, OUT_DIR / "table1_metrics.png")
