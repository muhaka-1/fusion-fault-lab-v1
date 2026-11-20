"""Synthetic vehicle trajectory and sensor models (planar, 2D).

Truth state per step: x, y [m], heading psi [rad], speed v [m/s].
Sensors:
  * IMU (100 Hz): yaw-rate gyro (with constant bias + white noise) and
    longitudinal accelerometer (white noise).
  * Wheel speed (50 Hz): speed measurement with white noise.
  * GNSS (5 Hz): position measurement with white noise.

Truth is generated with trapezoidal integration while the filter uses Euler
integration, so the filter model is not an exact copy of the simulator.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass
class SimConfig:
    dt: float = 0.01              # IMU rate 100 Hz
    duration: float = 120.0       # s
    gnss_rate_hz: float = 5.0
    wheel_rate_hz: float = 50.0
    sigma_gyro: float = 0.003     # rad/s white noise (per sample)
    gyro_bias: float = 0.004      # rad/s true constant bias
    sigma_acc: float = 0.05       # m/s^2 white noise (per sample)
    sigma_gnss: float = 1.5       # m per axis
    sigma_wheel: float = 0.10     # m/s
    seed: int = 1


@dataclass
class SimData:
    t: np.ndarray                 # (N,)
    truth: np.ndarray             # (N,4) x, y, psi, v
    gyro: np.ndarray              # (N,) rad/s (biased, noisy)
    acc: np.ndarray               # (N,) m/s^2
    gnss_idx: np.ndarray          # indices into t where GNSS arrives
    gnss: np.ndarray              # (G,2)
    wheel_idx: np.ndarray         # indices into t where wheel speed arrives
    wheel: np.ndarray             # (W,)
    cfg: SimConfig


def _cumtrapz(y: np.ndarray, dt: float) -> np.ndarray:
    out = np.zeros_like(y)
    out[1:] = np.cumsum(0.5 * (y[1:] + y[:-1]) * dt)
    return out


def simulate(cfg: SimConfig | None = None) -> SimData:
    cfg = cfg or SimConfig()
    rng = np.random.default_rng(cfg.seed)
    n = int(round(cfg.duration / cfg.dt)) + 1
    t = np.arange(n) * cfg.dt

    # Smooth speed profile and yaw rate: curves, lane-change-like wiggles.
    v = 12.0 + 3.0 * np.sin(2 * np.pi * t / 40.0)
    a = np.gradient(v, cfg.dt)
    r = 0.15 * np.sin(2 * np.pi * t / 30.0) + 0.08 * np.sin(2 * np.pi * t / 11.0)
    psi = _cumtrapz(r, cfg.dt)
    x = _cumtrapz(v * np.cos(psi), cfg.dt)
    y = _cumtrapz(v * np.sin(psi), cfg.dt)
    truth = np.column_stack([x, y, psi, v])

    gyro = r + cfg.gyro_bias + rng.normal(0, cfg.sigma_gyro, n)
    acc = a + rng.normal(0, cfg.sigma_acc, n)

    g_step = int(round(1.0 / (cfg.gnss_rate_hz * cfg.dt)))
    w_step = int(round(1.0 / (cfg.wheel_rate_hz * cfg.dt)))
    gnss_idx = np.arange(g_step, n, g_step)
    wheel_idx = np.arange(w_step, n, w_step)
    gnss = truth[gnss_idx, :2] + rng.normal(0, cfg.sigma_gnss, (len(gnss_idx), 2))
    wheel = truth[wheel_idx, 3] + rng.normal(0, cfg.sigma_wheel, len(wheel_idx))
    return SimData(t, truth, gyro, acc, gnss_idx, gnss, wheel_idx, wheel, cfg)
