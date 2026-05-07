#!/usr/bin/env python3
"""Render paper equations (revised) to PNG using matplotlib mathtext."""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path

OUT_DIR = Path("/home/cho/ms_AIworker/rosbags")

plt.rcParams["mathtext.fontset"] = "cm"
plt.rcParams["font.family"] = "serif"


def render_single(tex: str, out_path: Path, height_in: float = 0.9,
                  width_in: float = 4.5, dpi: int = 400) -> None:
    fig = plt.figure(figsize=(width_in, height_in))
    fig.text(0.5, 0.5, tex, ha="center", va="center", fontsize=14)
    fig.savefig(str(out_path), dpi=dpi, bbox_inches="tight",
                pad_inches=0.08, transparent=True)
    plt.close(fig)
    print(f"saved: {out_path}")


def render_lines(lines: list[str], out_path: Path,
                 width_in: float = 5.0, dpi: int = 400) -> None:
    n = len(lines)
    fig = plt.figure(figsize=(width_in, 0.55 * n + 0.1))
    for i, tex in enumerate(lines):
        y = 1.0 - (i + 0.5) / n
        fig.text(0.5, y, tex, ha="center", va="center", fontsize=14)
    fig.savefig(str(out_path), dpi=dpi, bbox_inches="tight",
                pad_inches=0.08, transparent=True)
    plt.close(fig)
    print(f"saved: {out_path}")


# Eq (1) revised: tight rectangular AABB check.
eq1 = (
    r"$\left|(\mathbf{p}_{\mathrm{cell}} - \mathbf{p}_{\mathrm{robot}})_x\right| "
    r"\leq \frac{L}{2} \; \wedge \; "
    r"\left|(\mathbf{p}_{\mathrm{cell}} - \mathbf{p}_{\mathrm{robot}})_y\right| "
    r"\leq \frac{W}{2}$"
)

# Eq (2) revised: v_cmd (feedforward + P), t* split, omega_cmd.
eq2_lines = [
    r"$\mathbf{v}_{\mathrm{cmd}}(t) = \dot{\mathbf{p}}_{\mathrm{ref}}(t^{*}) "
    r"+ \mathbf{K}_{p}\left(\mathbf{p}_{\mathrm{ref}}(t^{*}) - \mathbf{p}_{\mathrm{odom}}\right)$",

    r"$t_{c} = \mathrm{arg\,min}_{t}\,\left\|\mathbf{p}_{\mathrm{ref}}(t) - \mathbf{p}_{\mathrm{odom}}\right\|_{2}, "
    r"\quad t^{*} = t_{c} + \Delta t_{\ell}$",

    r"$\omega_{\mathrm{cmd}} = K_{\psi}\left(\psi_{\mathrm{vp}} - \psi_{\mathrm{odom}}\right)$",
]


if __name__ == "__main__":
    render_single(eq1, OUT_DIR / "eq1_revised.png", height_in=0.7)
    render_lines(eq2_lines, OUT_DIR / "eq2_revised.png")
