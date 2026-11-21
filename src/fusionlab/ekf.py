"""M3: Extended Kalman Filter, state = [x, y, psi, v, b_g].

Prediction (100 Hz): the IMU is the control input; the gyro bias b_g is subtracted.
    x   += v cos(psi) dt        y += v sin(psi) dt
    psi += (gyro - b_g) dt      v += acc dt        b_g stays (slow random walk)
Updates: GNSS position (5 Hz) and wheel speed (50 Hz).

New in M3 compared with M2:
    * the 5th state b_g, so the filter learns the gyro's bias instead of trusting it
    * update_wheel(), a 1-D measurement of speed
Not here yet: gating (rejecting measurements that don't fit) -> M5.

Setting p0_bias = 0 and q_bias = 0 freezes b_g at 0, which gives exactly the M2 filter.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np

N = 5  # number of states


@dataclass
class EKFConfig:
    sigma_gyro: float = 0.003   # rad/s, gyro white noise -> heading process noise
    sigma_acc: float = 0.05     # m/s^2, accelerometer noise -> speed process noise
    q_bias: float = 1e-5        # rad/s/sqrt(s), how fast the bias may wander (random walk)
    sigma_gnss: float = 1.5     # m per axis, GNSS measurement noise
    sigma_wheel: float = 0.10   # m/s, wheel-speed measurement noise
    p0_pos: float = 5.0         # initial 1-sigma uncertainties
    p0_psi: float = 0.2
    p0_v: float = 1.0
    p0_bias: float = 0.02       # rad/s: "the bias is probably within +-0.02"


def wrap(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + np.pi) % (2 * np.pi) - np.pi


class EKF:
    def __init__(self, x0, cfg: EKFConfig | None = None):
        self.cfg = cfg or EKFConfig()
        c = self.cfg
        self.x = np.array([x0[0], x0[1], x0[2], x0[3], 0.0], dtype=float)   # bias starts at 0: unknown
        self.P = np.diag([c.p0_pos**2, c.p0_pos**2, c.p0_psi**2, c.p0_v**2, c.p0_bias**2])

    def predict(self, gyro: float, acc: float, dt: float) -> None:
        """Move the state forward one IMU step and grow the uncertainty."""
        c = self.cfg
        x, y, psi, v, b = self.x
        self.x = np.array([x + v * np.cos(psi) * dt,
                           y + v * np.sin(psi) * dt,
                           wrap(psi + (gyro - b) * dt),
                           v + acc * dt,
                           b])
        F = np.eye(N)
        F[0, 2] = -v * np.sin(psi) * dt
        F[0, 3] = np.cos(psi) * dt
        F[1, 2] = v * np.cos(psi) * dt
        F[1, 3] = np.sin(psi) * dt
        F[2, 4] = -dt               # NEW: a bias error turns into a heading error over time
        # Noise: gyro and acc noise enter heading and speed; the bias wanders slowly
        G = np.zeros((N, 3))
        G[2, 0] = dt
        G[3, 1] = dt
        G[4, 2] = 1.0
        Q = np.diag([c.sigma_gyro**2, c.sigma_acc**2, c.q_bias**2 * dt])
        self.P = F @ self.P @ F.T + G @ Q @ G.T
        self.P = 0.5 * (self.P + self.P.T)

    def _update(self, z, H, R) -> float:
        """Generic Kalman update for a linear measurement z = H x + noise. Returns the NIS."""
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = wrap(self.x[2])
        I = np.eye(N)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T   # Joseph form
        self.P = 0.5 * (self.P + self.P.T)
        return float(y @ np.linalg.solve(S, y))

    def update_gnss(self, z) -> float:
        """GNSS position fix (2-D). Returns the NIS (expected average: 2)."""
        H = np.zeros((2, N)); H[0, 0] = 1.0; H[1, 1] = 1.0
        R = np.eye(2) * self.cfg.sigma_gnss**2
        return self._update(np.asarray(z, dtype=float), H, R)

    def update_wheel(self, z: float) -> float:
        """Wheel-speed measurement (1-D). Returns the NIS (expected average: 1)."""
        H = np.zeros((1, N)); H[0, 3] = 1.0
        R = np.array([[self.cfg.sigma_wheel**2]])
        return self._update(np.array([z], dtype=float), H, R)

    def pos_std(self) -> float:
        """1-sigma horizontal position uncertainty the filter believes it has [m]."""
        return float(np.sqrt(self.P[0, 0] + self.P[1, 1]))
