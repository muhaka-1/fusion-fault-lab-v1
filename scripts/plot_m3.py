"""M3 check by eye: what the bias state and the wheel sensor each add.

    python scripts/plot_m3.py            # writes docs/m3_bias_wheel.png
"""
import argparse, pathlib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from fusionlab.ekf import EKFConfig, wrap
from fusionlab.run import run_filter, metrics, NO_BIAS_STATE
from fusionlab.sim import SimConfig, simulate

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=1)
a = ap.parse_args()
d = simulate(SimConfig(seed=a.seed))
cases = {
    "M2: no bias state, no wheel": run_filter(d, EKFConfig(**NO_BIAS_STATE), use_wheel=False),
    "wheel only": run_filter(d, EKFConfig(**NO_BIAS_STATE), use_wheel=True),
    "bias state only": run_filter(d, EKFConfig(), use_wheel=False),
    "M3: bias state + wheel": run_filter(d, EKFConfig(), use_wheel=True),
}
full = cases["M3: bias state + wheel"]

fig, ax = plt.subplots(2, 2, figsize=(13, 8.5))
for k, r in cases.items():
    ax[0, 0].plot(r.t, r.pos_err, lw=0.9, label=f"{k}: RMSE {metrics(r)['rmse_m']:.2f} m")
ax[0, 0].set(title="Position error: the bias state is what fixes it", xlabel="t [s]", ylabel="m",
             yscale="log", ylim=(0.01, 30))
ax[0, 0].legend(fontsize=8)

b, bs = full.est[:, 4], full.bias_std
ax[0, 1].fill_between(full.t, b - 2 * bs, b + 2 * bs, alpha=0.25, label="filter's ±2σ")
ax[0, 1].plot(full.t, b, label="estimated bias")
ax[0, 1].axhline(full.true_bias, color="k", ls="--", lw=1, label=f"true bias {full.true_bias}")
ax[0, 1].set(title="The filter learns the gyro bias", xlabel="t [s]", ylabel="rad/s", ylim=(-0.01, 0.015))
ax[0, 1].legend(fontsize=8)

for k in ("bias state only", "M3: bias state + wheel"):
    r = cases[k]
    ax[1, 0].plot(r.t, r.est[:, 3] - r.truth[:, 3], lw=0.8,
                  label=f"{k}: RMSE {metrics(r)['speed_rmse_mps']:.3f} m/s")
ax[1, 0].set(title="Speed error: the wheel sensor makes it ~7x better", xlabel="t [s]", ylabel="m/s")
ax[1, 0].legend(fontsize=8)

for k in ("M2: no bias state, no wheel", "M3: bias state + wheel"):
    r = cases[k]
    he = np.degrees([wrap(x) for x in r.est[:, 2] - r.truth[:, 2]])
    ax[1, 1].plot(r.t, he, lw=0.9, label=f"{k}: RMSE {metrics(r)['heading_rmse_deg']:.2f}°")
ax[1, 1].set(title="Heading error", xlabel="t [s]", ylabel="deg")
ax[1, 1].legend(fontsize=8)
fig.tight_layout()
out = pathlib.Path("docs/m3_bias_wheel.png"); out.parent.mkdir(exist_ok=True)
fig.savefig(out, dpi=110)

print(f"seed {a.seed}, true gyro bias {d.cfg.gyro_bias} rad/s")
print(f"{'case':30s} {'RMSE m':>7s} {'speed m/s':>10s} {'heading deg':>12s} {'bias':>8s} {'NIS':>6s} {'wheel NIS':>10s}")
for k, r in cases.items():
    m = metrics(r)
    print(f"{k:30s} {m['rmse_m']:7.2f} {m['speed_rmse_mps']:10.3f} {m['heading_rmse_deg']:12.2f} "
          f"{m['bias_final']:8.4f} {m['mean_nis']:6.2f} {m['mean_wheel_nis']:10.2f}")
print("(consistent filter: GNSS NIS near 2, wheel NIS near 1)")
print(f"wrote {out}")
