"""Run the filter over a simulated drive and measure it against the truth."""
from __future__ import annotations
import argparse
from dataclasses import dataclass
import numpy as np
from .sim import SimConfig, simulate
from .ekf import EKF, EKFConfig, wrap


@dataclass
class Result:
    t: np.ndarray
    est: np.ndarray        # (N, 4) x, y, psi, v
    truth: np.ndarray      # (N, 4)
    pos_err: np.ndarray    # (N,) distance estimate-truth [m]
    pos_std: np.ndarray    # (N,) the filter's own 1-sigma [m]
    nis: list              # (t, nis) per GNSS fix


def run_filter(data, ekf_cfg: EKFConfig | None = None) -> Result:
    ekf = EKF(data.truth[0], ekf_cfg)
    n = len(data.t)
    est, std, nis = np.zeros((n, 4)), np.zeros(n), []
    gnss_at = {int(i): k for k, i in enumerate(data.gnss_idx)}
    for i in range(n):
        if i > 0:
            ekf.predict(data.gyro[i], data.acc[i], data.cfg.dt)
        if i in gnss_at:
            nis.append((float(data.t[i]), ekf.update_gnss(data.gnss[gnss_at[i]])))
        est[i], std[i] = ekf.x, ekf.pos_std()
    err = np.hypot(est[:, 0] - data.truth[:, 0], est[:, 1] - data.truth[:, 1])
    return Result(data.t, est, data.truth, err, std, nis)


def metrics(res: Result) -> dict:
    head_err = np.degrees([wrap(a) for a in res.est[:, 2] - res.truth[:, 2]])
    return {
        "rmse_m": float(np.sqrt(np.mean(res.pos_err**2))),
        "max_err_m": float(res.pos_err.max()),
        "heading_rmse_deg": float(np.sqrt(np.mean(head_err**2))),
        "speed_rmse_mps": float(np.sqrt(np.mean((res.est[:, 3] - res.truth[:, 3]) ** 2))),
        "within_2sigma": float(np.mean(res.pos_err <= 2 * res.pos_std)),
        "mean_nis": float(np.mean([v for _, v in res.nis])),
    }


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Run the M2 filter on one simulated drive")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--bias", type=float, default=None, help="override the true gyro bias [rad/s]")
    a = ap.parse_args()
    cfg = SimConfig(seed=a.seed) if a.bias is None else SimConfig(seed=a.seed, gyro_bias=a.bias)
    d = simulate(cfg)
    m = metrics(run_filter(d))
    raw = np.sqrt(np.mean(np.sum((d.gnss - d.truth[d.gnss_idx, :2]) ** 2, axis=1)))
    print(f"seed {a.seed}, true gyro bias {d.cfg.gyro_bias} rad/s")
    print(f"  raw GNSS position RMSE : {raw:.2f} m")
    for k, v in m.items():
        print(f"  {k:24s}: {v:.3f}")
