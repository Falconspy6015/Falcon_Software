"""
quad_env.py — Custom Gymnasium environment for quadrotor hover/waypoint control.

Physics: 6-DOF rigid body, quaternion attitude, X-configuration 4-rotor mixer.
This is deliberately dependency-light (no PyBullet / no ROS) so it trains fast
and is easy to reason about. Once the policy is sane here, port it to
gym-pybullet-drones or a Gazebo/PX4 SITL loop for higher-fidelity sim-to-real
work — the observation/action contract below is written to make that swap easy.

State (13,):  [x,y,z, vx,vy,vz, qw,qx,qy,qz, wx,wy,wz]
Action (4,):  normalized rotor commands in [-1, 1] -> mapped to [0, T_max] thrust each
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces


def quat_to_rotmat(q):
    """q = [w, x, y, z] -> 3x3 rotation matrix (body -> world)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
    ])


def quat_derivative(q, omega):
    """Standard quaternion kinematics: q_dot = 0.5 * Omega(omega) * q"""
    w, x, y, z = q
    p, qx, r = omega
    return 0.5 * np.array([
        -x * p - y * qx - z * r,
         w * p - z * qx + y * r,
         z * p + w * qx - x * r,
        -y * p + x * qx + w * r,
    ])


class QuadrotorEnv(gym.Env):
    """Point-to-point / hover control task for a quadrotor."""

    metadata = {"render_modes": []}

    def __init__(self, task="hover", max_steps=500, dt=0.02, domain_randomize=False):
        super().__init__()
        assert task in ("hover", "waypoint")
        self.task = task
        self.dt = dt
        self.max_steps = max_steps
        self.domain_randomize = domain_randomize

        # --- Physical parameters (roughly a small racing-class quad) ---
        # NOTE: rather than reverse-engineering kf/rpm constants (easy to get
        # unit-wrong by orders of magnitude — ask me how I know), we specify
        # thrust directly via a thrust-to-weight ratio, which is the number
        # you actually care about when picking motors/props anyway.
        self.mass = 0.5          # kg
        self.g = 9.81
        self.arm_len = 0.17      # m, motor-to-center distance
        self.I = np.array([3.0e-3, 3.0e-3, 5.5e-3])  # diag inertia [Ixx,Iyy,Izz]
        self.thrust_to_weight = 2.5   # typical for a small racing/utility quad
        self.c_tau = 0.02             # yaw-torque-to-thrust coefficient (~drag/thrust ratio of a prop)
        self.drag_lin = 0.15          # simple linear velocity drag
        self.drag_ang = 0.02          # simple linear angular-rate drag

        weight = self.mass * self.g
        self.T_max = (self.thrust_to_weight * weight) / 4.0  # max thrust PER ROTOR (N)

        # Precompute the mixer: [T_total, tau_x, tau_y, tau_z]^T = M @ [T1,T2,T3,T4]^T
        # X-configuration, rotors numbered 1..4 at +45/135/225/315 deg, alternating spin.
        L = self.arm_len / np.sqrt(2)
        self.mixer = np.array([
            [1,           1,           1,           1],
            [-L,          L,           L,          -L],
            [ L,          L,          -L,          -L],
            [self.c_tau, -self.c_tau,  self.c_tau, -self.c_tau],
        ])  # rows: total thrust, roll torque, pitch torque, yaw torque

        # --- Spaces ---
        # obs = [pos_err(3), vel(3), quat(4), ang_vel(3)]  -> 13
        high = np.array([10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 20, 20, 20], dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(4,), dtype=np.float32)

        self.target = np.array([0.0, 0.0, 1.5])  # hover 1.5m up
        self._step_count = 0
        self._prev_action = np.zeros(4, dtype=np.float32)

    # ------------------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._step_count = 0
        self._prev_action[:] = 0.0

        start_xyz = self.np_random.uniform(-0.3, 0.3, size=3)
        start_xyz[2] += 1.0  # start near 1m altitude
        self.pos = start_xyz.astype(np.float64)
        self.vel = np.zeros(3)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])  # level attitude
        self.omega = np.zeros(3)

        if self.domain_randomize:
            self.mass *= self.np_random.uniform(0.9, 1.1)
            self.I = self.I * self.np_random.uniform(0.9, 1.1, size=3)

        if self.task == "waypoint":
            self.target = self.np_random.uniform(-1.5, 1.5, size=3)
            self.target[2] = np.abs(self.target[2]) + 0.5
        else:
            self.target = np.array([0.0, 0.0, 1.5])

        return self._get_obs(), {}

    # ------------------------------------------------------------------
    def step(self, action):
        action = np.clip(action, -1.0, 1.0)
        rotor_thrusts = (action + 1.0) / 2.0 * self.T_max  # -> [0, T_max]

        wrench = self.mixer @ rotor_thrusts  # [T, tau_x, tau_y, tau_z]
        thrust, torque = wrench[0], wrench[1:]

        R = quat_to_rotmat(self.quat)
        thrust_world = R @ np.array([0, 0, thrust])
        accel = thrust_world / self.mass - np.array([0, 0, self.g]) - self.drag_lin * self.vel

        ang_accel = torque / self.I - self.drag_ang * self.omega - np.cross(self.omega, self.I * self.omega) / self.I

        # semi-implicit Euler integration
        self.vel += accel * self.dt
        self.pos += self.vel * self.dt
        self.omega += ang_accel * self.dt
        self.quat += quat_derivative(self.quat, self.omega) * self.dt
        self.quat /= np.linalg.norm(self.quat)

        self._step_count += 1
        obs = self._get_obs()
        reward, terminated = self._compute_reward(action)
        truncated = self._step_count >= self.max_steps
        self._prev_action = action.copy()

        info = {"pos": self.pos.copy(), "target": self.target.copy()}
        return obs, reward, terminated, truncated, info

    # ------------------------------------------------------------------
    def _get_obs(self):
        pos_err = self.target - self.pos
        return np.concatenate([pos_err, self.vel, self.quat, self.omega]).astype(np.float32)

    def _compute_reward(self, action):
        pos_err = np.linalg.norm(self.target - self.pos)
        vel_pen = np.linalg.norm(self.vel)
        ang_vel_pen = np.linalg.norm(self.omega)
        action_rate_pen = np.linalg.norm(action - self._prev_action)

        # Upright-ness: z-component of body z-axis in world frame (1 = level, <0 = flipped)
        R = quat_to_rotmat(self.quat)
        uprightness = R[2, 2]

        reward = (
            1.0                       # alive bonus
            - 1.2 * pos_err
            - 0.05 * vel_pen
            - 0.02 * ang_vel_pen
            - 0.01 * action_rate_pen
            + 0.5 * uprightness
        )

        terminated = False
        crashed = self.pos[2] < 0.05 or pos_err > 5.0 or uprightness < 0.0
        if crashed:
            reward -= 20.0
            terminated = True

        return float(reward), terminated

    def render(self):
        pass  # add PyBullet / matplotlib live-plot here if you want visualization


gym.register(id="Quadrotor-v0", entry_point="quad_env:QuadrotorEnv")
