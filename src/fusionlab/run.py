"""Run the filter over a simulated drive and measure it against the truth."""
from __future__ import annotations
import argparse
from dataclasses import dataclass
import numpy as np
from .sim import SimConfig, simulate
from .ekf import EKF, EKFConfig, wrap

# EKFConfig that freezes the bias state at 0: the M2 filter, for comparisons
NO_BIAS_STATE = dict(p0_bias=0.0, q_bias=0.0)


@dataclass
class Result:
    t: np.ndarray
    est: np.ndarray        # (N, 5) x, y, psi, v, b_g
    truth: np.ndarray      # (N, 4) x, y, psi, v
    true_bias: float
    pos_err: np.ndarray    # (N,) distance estimate-truth [m]
    pos_std: np.ndarray    # (N,) the filter's own 1-sigma [m]
    bias_std: np.ndarray   # (N,) the filter's 1-sigma on the bias [rad/s]
    nis: list              # (t, nis) per GNSS fix
    wheel_nis: list        # (t, nis) per wheel sample


def run_filter(data, ekf_cfg: EKFConfig | None = None, use_wheel: bool = True) -> Result:
    ekf = EKF(data.truth[0], ekf_cfg)
    n = len(data.t)
    est, std, bstd = np.zeros((n, 5)), np.zeros(n), np.zeros(n)
    nis, wnis = [], []
    gnss_at = {int(i): k for k, i in enumerate(data.gnss_idx)}
    wheel_at = {int(i): k for k, i in enumerate(data.wheel_idx)}
    for i in range(n):
        if i > 0:
            ekf.predict(data.gyro[i], data.acc[i], data.cfg.dt)
        t = float(data.t[i])
        if use_wheel and i in wheel_at:
            wnis.append((t, ekf.update_wheel(data.wheel[wheel_at[i]])))
        if i in gnss_at:
            nis.append((t, ekf.update_gnss(data.gnss[gnss_at[i]])))
        est[i], std[i], bstd[i] = ekf.x, ekf.pos_std(), np.sqrt(ekf.P[4, 4])
    err = np.hypot(est[:, 0] - data.truth[:, 0], est[:, 1] - data.truth[:, 1])
    return Result(data.t, est, data.truth, data.cfg.gyro_bias, err, std, bstd, nis, wnis)


def metrics(res: Result) -> dict:
    head_err = np.degrees([wrap(a) for a in res.est[:, 2] - res.truth[:, 2]])
    return {
        "rmse_m": float(np.sqrt(np.mean(res.pos_err**2))),
        "max_err_m": float(res.pos_err.max()),
        "heading_rmse_deg": float(np.sqrt(np.mean(head_err**2))),
        "speed_rmse_mps": float(np.sqrt(np.mean((res.est[:, 3] - res.truth[:, 3]) ** 2))),
        "bias_final": float(res.est[-1, 4]),
        "bias_err_final": float(res.est[-1, 4] - res.true_bias),
        "within_2sigma": float(np.mean(res.pos_err <= 2 * res.pos_std)),
        "mean_nis": float(np.mean([v for _, v in res.nis])),
        "mean_wheel_nis": float(np.mean([v for _, v in res.wheel_nis])) if res.wheel_nis else float("nan"),
    }


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Run the filter on one simulated drive")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--bias", type=float, default=None, help="override the true gyro bias [rad/s]")
    ap.add_argument("--no-wheel", action="store_true", help="ignore the wheel-speed sensor")
    ap.add_argument("--no-bias-state", action="store_true", help="freeze the bias state at 0 (the M2 filter)")
    a = ap.parse_args()
    cfg = SimConfig(seed=a.seed) if a.bias is None else SimConfig(seed=a.seed, gyro_bias=a.bias)
    d = simulate(cfg)
    ekf_cfg = EKFConfig(**NO_BIAS_STATE) if a.no_bias_state else EKFConfig()
    m = metrics(run_filter(d, ekf_cfg, use_wheel=not a.no_wheel))
    raw = np.sqrt(np.mean(np.sum((d.gnss - d.truth[d.gnss_idx, :2]) ** 2, axis=1)))
    print(f"seed {a.seed}, true gyro bias {d.cfg.gyro_bias} rad/s, wheel {'off' if a.no_wheel else 'on'}, "
          f"bias state {'off' if a.no_bias_state else 'on'}")
    print(f"  raw GNSS position RMSE : {raw:.2f} m")
    for k, v in m.items():
        print(f"  {k:24s}: {v:.4f}" if "bias" in k else f"  {k:24s}: {v:.3f}")
