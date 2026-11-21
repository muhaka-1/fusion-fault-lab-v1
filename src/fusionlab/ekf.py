"""M6: Extended Kalman Filter with NIS gating and lock-out recovery, state = [x, y, psi, v, b_g].

Prediction (100 Hz): the IMU is the control input; the gyro bias b_g is subtracted.
    x   += v cos(psi) dt        y += v sin(psi) dt
    psi += (gyro - b_g) dt      v += acc dt        b_g stays (slow random walk)
Updates: GNSS position (5 Hz) and wheel speed (50 Hz).

New in M3: the 5th state b_g and update_wheel().
New in M5: gating. Before using a measurement, the filter checks its NIS against a chi-square
threshold; a measurement that is too surprising is rejected instead of corrupting the state.
Both updates now return (accepted, nis, dof).
New in M6: lock-out recovery. If GNSS has been rejected lockout_rejects times in a row (10 s at
5 Hz), the filter's own estimate is more likely wrong than the sensor: it inflates the uncertainty
of position, heading and bias so the gate can open again. lockout_rejects = 0 turns it off (= M5).

Setting p0_bias = 0 and q_bias = 0 freezes b_g at 0, which gives exactly the M2 filter.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from .stats import chi2_threshold

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
    gate_prob: float = 0.999    # accept a measurement if its NIS is below the 99.9 % chi-square value
    lockout_rejects: int = 50   # NEW: GNSS rejections in a row before recovery (10 s at 5 Hz); 0 = off
    lockout_inflate: float = 25.0  # NEW: factor on the std of position, heading and bias at recovery


def wrap(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + np.pi) % (2 * np.pi) - np.pi


class EKF:
    def __init__(self, x0, cfg: EKFConfig | None = None, gating: bool = True):
        self.cfg = cfg or EKFConfig()
        self.gating = gating
        self.gnss_rejects = 0       # GNSS fixes rejected in a row
        self.recoveries = 0         # how many times the lock-out recovery fired
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

    def _update(self, z, H, R):
        """Gated Kalman update for a linear measurement z = H x + noise.
        Returns (accepted, nis, dof). A rejected measurement leaves x and P untouched."""
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        nis = float(y @ np.linalg.solve(S, y))
        dof = len(z)
        accepted = (not self.gating) or nis <= chi2_threshold(self.cfg.gate_prob, dof)
        if not accepted:
            return False, nis, dof
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = wrap(self.x[2])
        I = np.eye(N)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T   # Joseph form
        self.P = 0.5 * (self.P + self.P.T)
        return True, nis, dof

    def update_gnss(self, z):
        """GNSS position fix (2-D). Returns (accepted, nis, dof); NIS averages 2 when all is well."""
        H = np.zeros((2, N)); H[0, 0] = 1.0; H[1, 1] = 1.0
        R = np.eye(2) * self.cfg.sigma_gnss**2
        out = self._update(np.asarray(z, dtype=float), H, R)
        self.gnss_rejects = 0 if out[0] else self.gnss_rejects + 1
        if self.gating and self.cfg.lockout_rejects > 0 and self.gnss_rejects >= self.cfg.lockout_rejects:
            # Lock-out recovery: GNSS has been rejected for so long that the filter's own estimate is
            # more likely wrong than the sensor. Inflate position, heading and bias uncertainty (not
            # speed: the wheel keeps that honest) so the next fixes fall inside the gate again.
            f = self.cfg.lockout_inflate
            D = np.diag([f, f, f, 1.0, f])
            self.P = D @ self.P @ D
            self.gnss_rejects = 0
            self.recoveries += 1
        return out

    def update_wheel(self, z: float):
        """Wheel-speed measurement (1-D). Returns (accepted, nis, dof); NIS averages 1."""
        H = np.zeros((1, N)); H[0, 3] = 1.0
        R = np.array([[self.cfg.sigma_wheel**2]])
        return self._update(np.array([z], dtype=float), H, R)

    def pos_std(self) -> float:
        """1-sigma horizontal position uncertainty the filter believes it has [m]."""
        return float(np.sqrt(self.P[0, 0] + self.P[1, 1]))
