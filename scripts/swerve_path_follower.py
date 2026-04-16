#!/usr/bin/env python3
"""
ApexNAV Swerve Path Follower
============================

Replaces ApexNAV's `traj_server` (unicycle MPC) with a holonomic pure-pursuit
controller suited for the FFW-SG2 swerve drive.

Subscribes : /planning/trajectory  (trajectory_manager/msg/PolyTraj, septic poly)
             /habitat/odom          (World-frame position from apexnav_bridge)
Publishes  : /cmd_vel               (geometry_msgs/Twist) at 50 Hz

The C++ planner publishes a piece-wise septic polynomial via PolyTraj. We
evaluate that trajectory at (now + lookahead), compute world-frame position
error against the latest odom, rotate it into base_link, apply a proportional
gain, and clamp the resulting (vx, vy) to v_max.

Yaw tracking: the robot rotates to face the direction of travel so the
forward-facing depth camera always sees obstacles ahead. When the yaw error
is large, the robot stops and rotates in place first.

Author: 2026-04-07 swerve adaptation (replaces traj_server for IsaacSim)
"""

import math
from threading import Lock

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from trajectory_manager.msg import PolyTraj


class SwervePathFollower(Node):
    def __init__(self):
        super().__init__('swerve_path_follower')

        # === Tunable parameters ===
        self.kp_xy = 2.5         # proportional gain on world-frame position error
        self.v_max = 0.3         # max linear speed (m/s) — sim limit
        self.lookahead = 0.25    # seconds ahead in trajectory to chase
        self.tick_hz = 50.0      # control loop rate
        self.tail_decay = 0.85   # multiplicative decay applied each tick after end
        self.stale_traj_timeout = 0.5  # seconds since last traj before we hold zero

        # === Yaw tracking — face direction of travel for depth camera ===
        self.kp_yaw = 2.0        # proportional gain on yaw error
        self.max_omega = 1.0     # max angular velocity (rad/s)
        self.yaw_deadband = 0.05 # below this distance (m), don't update desired yaw
        self.yaw_align_thresh = math.radians(80)  # above this: stop and rotate in place

        # === State ===
        self.lock = Lock()
        self.traj = None                 # parsed dict, see _traj_cb
        self.traj_start_time = None      # rclpy.time.Time
        self.traj_duration = 0.0
        self.traj_recv_wall = None       # wall time of last traj
        self.last_cmd = (0.0, 0.0, 0.0)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.have_odom = False

        # QoS
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_reliable)
        # /habitat/odom: World-frame position from apexnav_bridge (PUB_QOS=RELIABLE)
        self.create_subscription(Odometry, '/habitat/odom', self._odom_cb, qos_reliable)
        self.create_subscription(
            PolyTraj, '/planning/trajectory', self._traj_cb, qos_reliable
        )

        self.create_timer(1.0 / self.tick_hz, self._tick)

        self.get_logger().info(
            f"swerve_path_follower started: kp_xy={self.kp_xy}, "
            f"v_max={self.v_max}, lookahead={self.lookahead}s, tick={self.tick_hz}Hz"
        )

    # ------------------------------------------------------------------ Odom
    def _odom_cb(self, msg: Odometry):
        with self.lock:
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.odom_yaw = math.atan2(siny, cosy)
            self.have_odom = True

    # ------------------------------------------------------------------ Traj
    def _traj_cb(self, msg: PolyTraj):
        """Atomically swap in a new trajectory.

        PolyTraj layout (trajectory_manager/msg/PolyTraj):
            order        : uint8 (=7 for septic)
            duration[]   : float32, length = piece_num
            coef_x[]     : float32, flat length = 8 * piece_num (channel-major)
            coef_y[]     : float32, flat length = 8 * piece_num
            start_time   : builtin_interfaces/Time

        Coefficient order matches gcopter/trajectory.hpp::getPos which loops
        i = D..0 with `tn *= t` AFTER the accumulate, so coef[D] (= coef[7])
        is the constant term and coef[0] is the t^7 term.
        """
        order = int(msg.order)                       # expected 7
        num_coef = order + 1                         # 8
        n_pieces = len(msg.duration)
        if n_pieces == 0:
            return
        try:
            coef_x = np.asarray(msg.coef_x, dtype=np.float64).reshape(n_pieces, num_coef)
            coef_y = np.asarray(msg.coef_y, dtype=np.float64).reshape(n_pieces, num_coef)
        except ValueError as e:
            self.get_logger().error(f"PolyTraj reshape failed: {e}")
            return
        durations = np.asarray(msg.duration, dtype=np.float64)
        cumulative_t = np.concatenate([[0.0], np.cumsum(durations)])

        with self.lock:
            self.traj = {
                'order': order,
                'n': n_pieces,
                'durations': durations,
                'coef_x': coef_x,
                'coef_y': coef_y,
                'cumulative_t': cumulative_t,
            }
            # Always use wall clock for elapsed calculation.
            # C++ planner uses sim time for start_time, but this node may not
            # have use_sim_time — mixing clock domains causes garbage elapsed.
            self.traj_start_time = self.get_clock().now()
            self.traj_duration = float(np.sum(durations))
            self.traj_recv_wall = self.get_clock().now()
            # last_cmd 유지 — replan 시 부드럽게 전환, 멈춤은 past_end/stale에서만
        self.get_logger().info(
            f"new traj: id={msg.traj_id} pieces={n_pieces} duration={self.traj_duration:.2f}s"
        )

    # ------------------------------------------------------------------ Eval
    def _eval_pos(self, t_rel: float):
        """Evaluate (x, y) on the active piece-wise septic poly at time t_rel."""
        traj = self.traj
        if traj is None:
            return None
        if t_rel < 0.0:
            t_rel = 0.0
        if t_rel > self.traj_duration:
            t_rel = self.traj_duration
        idx = int(np.searchsorted(traj['cumulative_t'], t_rel, side='right') - 1)
        idx = max(0, min(idx, traj['n'] - 1))
        local_t = t_rel - traj['cumulative_t'][idx]
        cx = traj['coef_x'][idx]
        cy = traj['coef_y'][idx]
        order = traj['order']
        # Mirror trajectory.hpp::getPos exactly:
        #   tn = 1; for i = order..0: pos += tn * coef[i]; tn *= local_t
        tn = 1.0
        x = 0.0
        y = 0.0
        for i in range(order, -1, -1):
            x += tn * cx[i]
            y += tn * cy[i]
            tn *= local_t
        return (x, y)

    def _find_closest_t(self, x: float, y: float) -> float:
        """Find trajectory time t in [0, traj_duration] whose (x,y) is closest to (x,y).

        Vectorized sampling over every piece using Horner's method. Sample spacing
        is 0.05 s (min 20 samples total). Returns clamped t.
        """
        traj = self.traj
        if traj is None or self.traj_duration <= 0.0:
            return 0.0

        total = self.traj_duration
        n_samples = max(int(total / 0.05), 20)
        order = traj['order']
        cumulative_t = traj['cumulative_t']
        durations = traj['durations']
        n_pieces = traj['n']

        # Build per-piece sample counts proportional to duration.
        counts = np.maximum(
            np.round(durations / total * n_samples).astype(int), 2
        )

        xs_list = []
        ys_list = []
        ts_list = []
        for idx in range(n_pieces):
            k = int(counts[idx])
            dur = durations[idx]
            local_ts = np.linspace(0.0, dur, k, endpoint=(idx == n_pieces - 1))
            cx = traj['coef_x'][idx]
            cy = traj['coef_y'][idx]
            # Horner mirroring getPos: tn=1; for i=order..0: pos += tn*coef[i]; tn*=t
            x_vals = np.zeros_like(local_ts)
            y_vals = np.zeros_like(local_ts)
            tn = np.ones_like(local_ts)
            for i in range(order, -1, -1):
                x_vals += tn * cx[i]
                y_vals += tn * cy[i]
                tn *= local_ts
            xs_list.append(x_vals)
            ys_list.append(y_vals)
            ts_list.append(local_ts + cumulative_t[idx])

        xs = np.concatenate(xs_list)
        ys = np.concatenate(ys_list)
        ts = np.concatenate(ts_list)
        d2 = (xs - x) ** 2 + (ys - y) ** 2
        best = int(np.argmin(d2))
        t_closest = float(ts[best])
        if t_closest < 0.0:
            t_closest = 0.0
        if t_closest > total:
            t_closest = total
        return t_closest

    def _eval_vel(self, t_rel: float):
        """Evaluate velocity (dx, dy) by differentiating the septic poly at t_rel."""
        traj = self.traj
        if traj is None:
            return (0.0, 0.0)
        if t_rel < 0.0:
            t_rel = 0.0
        if t_rel > self.traj_duration:
            return (0.0, 0.0)
        idx = int(np.searchsorted(traj['cumulative_t'], t_rel, side='right') - 1)
        idx = max(0, min(idx, traj['n'] - 1))
        local_t = t_rel - traj['cumulative_t'][idx]
        cx = traj['coef_x'][idx]
        cy = traj['coef_y'][idx]
        order = traj['order']
        # Derivative of getPos: d/dt sum(coef[i] * t^(order-i))
        # = sum(coef[i] * (order-i) * t^(order-i-1))
        dx = 0.0
        dy = 0.0
        tn = 1.0
        for i in range(order, 0, -1):  # skip i=0 (constant term, derivative=0)
            power = order - i  # exponent of t in position polynomial
            dx += power * tn * cx[i]
            dy += power * tn * cy[i]
            tn *= local_t
        return (dx, dy)

    # ------------------------------------------------------------------ Tick
    def _tick(self):
        with self.lock:
            if not self.have_odom or self.traj is None or self.traj_start_time is None:
                return

            now = self.get_clock().now()

            # 거리 기반 평가: odom 위치에서 가장 가까운 trajectory 점을 찾아
            # 그 지점 + lookahead를 target으로. 시뮬 속도 한계로 뒤처져도
            # 항상 현재 위치 기준으로 전진 목표를 재조정.
            t_closest = self._find_closest_t(self.odom_x, self.odom_y)
            t_eval = min(t_closest + self.lookahead, self.traj_duration)

            # 플래너 정지 감지 (FINISH/crash) — trajectory 끝에 도달했을 때만
            age = (now - self.traj_recv_wall).nanoseconds * 1e-9
            if (t_closest > self.traj_duration - 0.05 and age > self.stale_traj_timeout) \
                    or age > 5.0:
                self.last_cmd = (0.0, 0.0, 0.0)
                vx = vy = omega = 0.0
            else:
                target = self._eval_pos(t_eval)
                if target is None:
                    return

                # Feedforward velocity from trajectory derivative at t_eval
                ff_vel = self._eval_vel(t_eval)
                cy = math.cos(self.odom_yaw)
                sy = math.sin(self.odom_yaw)
                # World → base frame
                ff_vx = cy * ff_vel[0] + sy * ff_vel[1]
                ff_vy = -sy * ff_vel[0] + cy * ff_vel[1]

                # P correction on position error
                err_x = target[0] - self.odom_x
                err_y = target[1] - self.odom_y
                vx_b = cy * err_x + sy * err_y
                vy_b = -sy * err_x + cy * err_y

                # cmd = feedforward + P correction
                vx = ff_vx + self.kp_xy * vx_b
                vy = ff_vy + self.kp_xy * vy_b
                speed = math.hypot(vx, vy)
                if speed > self.v_max:
                    scale = self.v_max / speed
                    vx *= scale
                    vy *= scale

                # Yaw: rotate to face direction of travel for depth camera
                omega = 0.0
                yaw_err = 0.0
                world_dist = math.hypot(err_x, err_y)
                if world_dist > self.yaw_deadband:
                    desired_yaw = math.atan2(err_y, err_x)
                    yaw_err = desired_yaw - self.odom_yaw
                    yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
                    omega = self.kp_yaw * yaw_err
                    omega = max(-self.max_omega, min(self.max_omega, omega))

                # In-place rotation when yaw error > threshold
                if abs(yaw_err) > self.yaw_align_thresh:
                    vx = 0.0
                    vy = 0.0

                self.last_cmd = (vx, vy, omega)

        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(omega)
        self.cmd_pub.publish(twist)

    def _publish_zero(self):
        self.last_cmd = (0.0, 0.0, 0.0)
        twist = Twist()
        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = SwervePathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
