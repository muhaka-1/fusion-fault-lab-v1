"""M2: Extended Kalman Filter, state = [x, y, psi, v].

Prediction (100 Hz): the IMU is the control input.
    x   += v cos(psi) dt        y += v sin(psi) dt
    psi += gyro dt              v += acc dt
Update (5 Hz): a GNSS position fix.

What is NOT here yet, on purpose:
    * wheel-speed update and the gyro-bias state  -> M3
    * gating (rejecting measurements that don't fit) -> M5
Without the bias state the filter believes the gyro, so its heading drifts between
fixes and every GNSS fix has to pull it back. M3 fixes that by estimating the bias.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np

N = 4  # number of states


@dataclass
class EKFConfig:
    sigma_gyro: float = 0.003   # rad/s, gyro white noise -> heading process noise
    sigma_acc: float = 0.05     # m/s^2, accelerometer noise -> speed process noise
    sigma_gnss: float = 1.5     # m per axis, GNSS measurement noise
    p0_pos: float = 5.0         # initial 1-sigma uncertainties
    p0_psi: float = 0.2
    p0_v: float = 1.0


def wrap(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + np.pi) % (2 * np.pi) - np.pi


class EKF:
    def __init__(self, x0, cfg: EKFConfig | None = None):
        self.cfg = cfg or EKFConfig()
        c = self.cfg
        self.x = np.array(x0[:4], dtype=float)
        self.P = np.diag([c.p0_pos**2, c.p0_pos**2, c.p0_psi**2, c.p0_v**2])

    def predict(self, gyro: float, acc: float, dt: float) -> None:
        """Move the state forward one IMU step and grow the uncertainty."""
        c = self.cfg
        x, y, psi, v = self.x
        self.x = np.array([x + v * np.cos(psi) * dt,
                           y + v * np.sin(psi) * dt,
                           wrap(psi + gyro * dt),
                           v + acc * dt])
        # Jacobian of the motion model with respect to the state
        F = np.eye(N)
        F[0, 2] = -v * np.sin(psi) * dt
        F[0, 3] = np.cos(psi) * dt
        F[1, 2] = v * np.cos(psi) * dt
        F[1, 3] = np.sin(psi) * dt
        # Sensor noise on gyro and acc enters heading and speed through dt
        G = np.zeros((N, 2))
        G[2, 0] = dt
        G[3, 1] = dt
        Q = np.diag([c.sigma_gyro**2, c.sigma_acc**2])
        self.P = F @ self.P @ F.T + G @ Q @ G.T
        self.P = 0.5 * (self.P + self.P.T)          # keep it exactly symmetric

    def update_gnss(self, z) -> float:
        """Correct the state with a GNSS fix. Returns the NIS (used for gating in M5)."""
        z = np.asarray(z, dtype=float)
        H = np.zeros((2, N)); H[0, 0] = 1.0; H[1, 1] = 1.0
        R = np.eye(2) * self.cfg.sigma_gnss**2
        y = z - H @ self.x                           # innovation: what the fix says minus what we expected
        S = H @ self.P @ H.T + R                     # how big we expect the innovation to be
        K = self.P @ H.T @ np.linalg.inv(S)          # Kalman gain: how much to trust the fix
        self.x = self.x + K @ y
        self.x[2] = wrap(self.x[2])
        I = np.eye(N)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T   # Joseph form
        self.P = 0.5 * (self.P + self.P.T)
        return float(y @ np.linalg.solve(S, y))

    def pos_std(self) -> float:
        """1-sigma horizontal position uncertainty the filter believes it has [m]."""
        return float(np.sqrt(self.P[0, 0] + self.P[1, 1]))
