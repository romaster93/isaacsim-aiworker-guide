#!/usr/bin/env python3
"""Plot trajectory comparison figure (fig3.pdf) from two .npz files — single-panel overlay."""

import matplotlib
matplotlib.use("Agg")

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

COLOR_A = "#d62728"   # red  — baseline / Ackermann
COLOR_B = "#1f77b4"   # blue — proposed / Swerve
COLOR_FREE = "#d0e8ff"
COLOR_OCC = "#555555"
COLOR_START = "#2ca02c"
COLOR_FINISH = "#000000"
COLOR_CHAIR = "#8e44ad"  # purple

# uniform color per method — runs distinguished by finish-mark run number (1/2/3)
SHADES_A = [COLOR_A, COLOR_A, COLOR_A]
SHADES_B = [COLOR_B, COLOR_B, COLOR_B]


def load_npz(path: str) -> dict:
    d = np.load(path, allow_pickle=True)
    result = {}
    for key in ("traj", "occupied_xy", "free_xy", "t_start", "t_finish",
                "label", "bag_path", "duration"):
        result[key] = d[key] if key in d else None
    for scalar_key in ("label", "bag_path", "t_start", "t_finish", "duration"):
        v = result[scalar_key]
        if v is not None and hasattr(v, "ndim") and v.ndim == 0:
            result[scalar_key] = v.item()
    for xy_key in ("occupied_xy", "free_xy"):
        v = result[xy_key]
        if v is not None and hasattr(v, "ndim") and v.ndim == 0:
            result[xy_key] = None
    return result


def rotate_xy(xy: np.ndarray, deg: int) -> np.ndarray:
    """CCW rotation by 0/90/180/270 deg on an (N,2) array."""
    if xy is None or len(xy) == 0 or deg % 360 == 0:
        return xy
    x, y = xy[:, 0], xy[:, 1]
    if deg % 360 == 90:
        return np.column_stack([-y, x])
    if deg % 360 == 180:
        return np.column_stack([-x, -y])
    if deg % 360 == 270:
        return np.column_stack([y, -x])
    return xy


def apply_rotation(data: dict, deg: int) -> dict:
    if deg % 360 == 0:
        return data
    out = dict(data)
    traj = data.get("traj")
    if traj is not None and len(traj) > 0:
        t = traj[:, 0:1]
        yaw = traj[:, 3:4] + np.deg2rad(deg)
        rotated = rotate_xy(traj[:, 1:3], deg)
        out["traj"] = np.hstack([t, rotated, yaw])
    for key in ("occupied_xy", "free_xy"):
        if data.get(key) is not None:
            out[key] = rotate_xy(data[key], deg)
    return out


def _has_map(data: dict) -> bool:
    occ = data.get("occupied_xy")
    free = data.get("free_xy")
    return (occ is not None and len(occ) > 0) or (free is not None and len(free) > 0)


def plot_map(ax, data: dict, s: float = 0.5) -> None:
    free = data.get("free_xy")
    occ = data.get("occupied_xy")
    if free is not None and len(free) > 0:
        ax.scatter(free[:, 0], free[:, 1], s=s, c=COLOR_FREE, linewidths=0, zorder=1)
    if occ is not None and len(occ) > 0:
        ax.scatter(occ[:, 0], occ[:, 1], s=s * 2, c=COLOR_OCC, linewidths=0, zorder=2)


def plot_traj(ax, traj: np.ndarray, color: str, label: str,
              linestyle: str, lw: float = 1.4, alpha: float = 1.0) -> None:
    if traj is None or len(traj) == 0:
        return
    x, y = traj[:, 1], traj[:, 2]
    ax.plot(x, y, color=color, linewidth=lw, linestyle=linestyle,
            alpha=alpha, label=label, zorder=3)
    ax.plot(x[-1], y[-1], "x", color=COLOR_FINISH, markersize=5,
            markeredgewidth=1.4, zorder=5)


RUN_STYLES = ["-", "--", (0, (1, 1))]  # run3: dense dotted (on=1, off=1)
RUN_LWS = [1.4, 1.4, 1.8]  # run3: thicker so dots stay visible at print size


def plot_traj_group(ax, runs: list, shades: list, base_color: str,
                    method_label: str, lw: float = 1.4,
                    annotate_runs: bool = True) -> None:
    """Plot multiple runs of the same method with distinct shades and run-number labels."""
    labeled = False
    for i, data in enumerate(runs):
        traj = data.get("traj")
        if traj is None or len(traj) == 0:
            continue
        color = shades[i % len(shades)]
        ls = RUN_STYLES[i % len(RUN_STYLES)]
        run_lw = RUN_LWS[i % len(RUN_LWS)]
        lbl = method_label if not labeled else None
        plot_traj(ax, traj, color, lbl, linestyle=ls, lw=run_lw)
        labeled = True
        if annotate_runs:
            xf, yf = traj[-1, 1], traj[-1, 2]
            ax.annotate(f"{i+1}", xy=(xf, yf), xytext=(4, 4),
                        textcoords="offset points",
                        fontsize=6, color=color, fontweight="bold",
                        zorder=6)


def plot_chairs(ax, chairs: list) -> None:
    if not chairs:
        return
    xs = [c[0] for c in chairs]
    ys = [c[1] for c in chairs]
    ax.scatter(xs, ys, marker="s", s=36, facecolor=COLOR_CHAIR,
               edgecolor="black", linewidths=0.8, zorder=5, label="Chair")


def union_maps(sources: list) -> dict:
    occ = [s["occupied_xy"] for s in sources
           if s.get("occupied_xy") is not None and len(s["occupied_xy"]) > 0]
    free = [s["free_xy"] for s in sources
            if s.get("free_xy") is not None and len(s["free_xy"]) > 0]
    return {
        "occupied_xy": np.vstack(occ) if occ else None,
        "free_xy": np.vstack(free) if free else None,
    }


def make_figure(runs_a: list, runs_b: list, out_path: str,
                label_a: str, label_b: str, map_from: str,
                extra_map_sources: list | None = None,
                legend_loc: str = "upper right",
                chairs: list | None = None,
                annotate_runs: bool = True) -> None:
    # Select map sources:
    # - "a"    -> runs_a only (fallback to b)
    # - "b"    -> runs_b only (fallback to a)
    # - "both" -> union of a and b
    # extra_map_sources are always appended (union).
    candidates: list = []
    if map_from == "both":
        for group in (runs_a, runs_b):
            candidates.extend([d for d in group if _has_map(d)])
    else:
        primary, secondary = (runs_b, runs_a) if map_from == "b" else (runs_a, runs_b)
        primary_maps = [d for d in primary if _has_map(d)]
        if primary_maps:
            candidates.extend(primary_maps)
        else:
            candidates.extend([d for d in secondary if _has_map(d)])
    if extra_map_sources:
        candidates.extend([s for s in extra_map_sources if _has_map(s)])

    if not candidates:
        print("WARNING: no npz provided map data — background will be empty.",
              file=sys.stderr)
        map_data = None
    else:
        map_data = union_maps(candidates)

    fig, ax = plt.subplots(figsize=(3.4, 3.0), constrained_layout=True)

    if map_data is not None:
        plot_map(ax, map_data)

    # start markers (every run; overlapping starts are harmless)
    for data in (*runs_a, *runs_b):
        traj = data.get("traj")
        if traj is not None and len(traj) > 0:
            ax.plot(traj[0, 1], traj[0, 2], "o", color=COLOR_START,
                    markersize=5, zorder=4)

    plot_traj_group(ax, runs_a, SHADES_A, COLOR_A, label_a,
                    annotate_runs=annotate_runs)
    plot_traj_group(ax, runs_b, SHADES_B, COLOR_B, label_b,
                    annotate_runs=annotate_runs)

    plot_chairs(ax, chairs or [])

    # two legends: methods/markers at user-chosen loc, run-linestyles at upper right
    start_handle = plt.Line2D([0], [0], marker="o", color="w",
                               markerfacecolor=COLOR_START, markersize=4, label="Start")
    finish_handle = plt.Line2D([0], [0], marker="x", color=COLOR_FINISH,
                                markersize=4, markeredgewidth=1.0, label="Finish",
                                linestyle="None")
    handles, labels = ax.get_legend_handles_labels()
    main_extra = [start_handle, finish_handle]
    legend_main = ax.legend(handles + main_extra,
                            labels + [h.get_label() for h in main_extra],
                            loc=legend_loc,
                            fontsize=5, frameon=True, framealpha=0.9,
                            handlelength=1.4, handletextpad=0.4,
                            labelspacing=0.3, borderpad=0.3)
    ax.add_artist(legend_main)

    run_handles = [
        plt.Line2D([0], [0], color="black", linestyle=RUN_STYLES[0], lw=RUN_LWS[0], label="Run 1"),
        plt.Line2D([0], [0], color="black", linestyle=RUN_STYLES[1], lw=RUN_LWS[1], label="Run 2"),
        plt.Line2D([0], [0], color="black", linestyle=RUN_STYLES[2], lw=RUN_LWS[2], label="Run 3"),
    ]
    ax.legend(handles=run_handles, loc="upper right",
              fontsize=5, frameon=True, framealpha=0.9,
              handlelength=1.8, handletextpad=0.4,
              labelspacing=0.3, borderpad=0.3)

    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x [m]", fontsize=8)
    ax.set_ylabel("y [m]", fontsize=8)
    ax.tick_params(labelsize=7)

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fmt = out.suffix.lstrip(".").lower() or "pdf"
    fig.savefig(str(out), format=fmt, bbox_inches="tight", dpi=300)
    plt.close(fig)
    print(f"Saved figure: {out}")
    print(f"  figsize: 3.4 x 3.0 inch (~86 x 76 mm, single-column)")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot trajectory comparison (fig3.pdf) — single-panel overlay.")
    parser.add_argument("npz_a", help="Baseline trajectory .npz (run 1)")
    parser.add_argument("npz_b", help="Proposed trajectory .npz (run 1)")
    parser.add_argument("-o", "--output", default="/tmp/fig3.pdf",
                        help="Output PDF path (default: /tmp/fig3.pdf)")
    parser.add_argument("--label-a", default=None, help="Label for method A")
    parser.add_argument("--label-b", default=None, help="Label for method B")
    parser.add_argument("--extra-a", nargs="*", default=[],
                        help="Additional baseline runs (overlaid, same color as A)")
    parser.add_argument("--extra-b", nargs="*", default=[],
                        help="Additional proposed runs (overlaid, same color as B)")
    parser.add_argument("--map-from", choices=["a", "b", "both"], default="both",
                        help="Which group to use for background map (default: both). "
                             "Maps from all runs in the chosen group(s) are unioned.")
    parser.add_argument("--extra-maps", nargs="+", default=[],
                        help="Extra .npz files whose map data is unioned into the background")
    parser.add_argument("--rotate", type=int, choices=[0, 90, 180, 270], default=0,
                        help="CCW rotation of map and trajectories in degrees")
    parser.add_argument("--legend-loc", default="upper right",
                        help="Legend location inside axes (matplotlib loc string)")
    parser.add_argument("--chairs", default="",
                        help="Chair positions as 'x,y' pairs separated by ';' "
                             "(data coords in final frame, after rotation). "
                             "Example: --chairs='-0.83,2.45;2.10,1.57;3.88,-2.73'")
    parser.add_argument("--no-annotate-runs", action="store_true",
                        help="Disable run-number labels near finish markers")
    args = parser.parse_args()

    chairs = []
    for s in (p.strip() for p in args.chairs.split(";") if p.strip()):
        try:
            x, y = s.split(",")
            chairs.append((float(x), float(y)))
        except Exception:
            print(f"WARNING: bad --chairs value '{s}', expected 'x,y'",
                  file=sys.stderr)

    runs_a = [apply_rotation(load_npz(p), args.rotate)
              for p in [args.npz_a, *args.extra_a]]
    runs_b = [apply_rotation(load_npz(p), args.rotate)
              for p in [args.npz_b, *args.extra_b]]
    extra = [apply_rotation(load_npz(p), args.rotate) for p in args.extra_maps]

    label_a = args.label_a or str(runs_a[0].get("label") or "Baseline")
    label_b = args.label_b or str(runs_b[0].get("label") or "Proposed")

    make_figure(runs_a, runs_b, args.output, label_a, label_b, args.map_from,
                extra_map_sources=extra, legend_loc=args.legend_loc,
                chairs=chairs, annotate_runs=not args.no_annotate_runs)


if __name__ == "__main__":
    main()
