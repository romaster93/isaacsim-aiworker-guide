#!/usr/bin/env python3
"""Prepend a thin virtual spline from Swerve start to Ackermann start.

Visual aid only: the Ackermann actual trajectory is preserved unchanged;
a smooth cubic Bezier bridge is inserted in front so both runs appear to
depart from the same point in fig3_run2.png.
"""

import numpy as np
from pathlib import Path

RB = Path("/home/cho/ms_AIworker/rosbags")
ACK = RB / "ackermann_run2.npz"
SWV = RB / "swerve_run2.npz"
OUT = RB / "ackermann_run2_bridged.npz"

N_BRIDGE = 40


def cubic_bezier(p0, p1, p2, p3, n):
    t = np.linspace(0.0, 1.0, n)[:, None]
    return ((1 - t) ** 3) * p0 + 3 * ((1 - t) ** 2) * t * p1 \
         + 3 * (1 - t) * (t ** 2) * p2 + (t ** 3) * p3


def main() -> None:
    ack = np.load(ACK, allow_pickle=True)
    swv = np.load(SWV, allow_pickle=True)

    ack_traj = ack["traj"]
    swv_traj = swv["traj"]

    p_swv = swv_traj[0, 1:3].astype(float)
    p_ack = ack_traj[0, 1:3].astype(float)
    yaw_ack = float(ack_traj[0, 3])

    d = np.linalg.norm(p_ack - p_swv)
    handle = 0.35 * d

    # P1: leave Swerve start heading toward Ackermann start.
    tangent_start = (p_ack - p_swv) / (d + 1e-9)
    P1 = p_swv + handle * tangent_start
    # P2: enter Ackermann start along its initial heading (reversed).
    tangent_end = np.array([np.cos(yaw_ack), np.sin(yaw_ack)])
    P2 = p_ack - handle * tangent_end

    bridge_xy = cubic_bezier(p_swv, P1, P2, p_ack, N_BRIDGE)

    t0 = float(ack_traj[0, 0])
    # Synthetic timestamps strictly before the real trajectory.
    t_bridge = np.linspace(t0 - 1.0, t0, N_BRIDGE, endpoint=False)
    yaw_bridge = np.full(N_BRIDGE, yaw_ack)

    bridge = np.column_stack([t_bridge, bridge_xy[:, 0], bridge_xy[:, 1], yaw_bridge])
    new_traj = np.vstack([bridge, ack_traj])

    # Carry over all other arrays unchanged.
    payload = {k: ack[k] for k in ack.files}
    payload["traj"] = new_traj
    np.savez(OUT, **payload)
    print(f"saved: {OUT}  (traj rows: {len(new_traj)}, bridge rows: {N_BRIDGE})")
    print(f"  swv start -> ack start: {p_swv} -> {p_ack}, distance={d:.3f} m")


if __name__ == "__main__":
    main()
