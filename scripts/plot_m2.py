"""M2 check by eye: when the filter works, when it fails, and why.

    python scripts/plot_m2.py            # writes docs/m2_ekf.png
"""
import argparse, pathlib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from fusionlab.ekf import EKFConfig, wrap
from fusionlab.run import run_filter, metrics
from fusionlab.sim import SimConfig, simulate

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=1)
a = ap.parse_args()

cases = {
    "no bias (model right)": (SimConfig(seed=a.seed, gyro_bias=0.0), EKFConfig()),
    "bias 0.004, honest gyro noise": (SimConfig(seed=a.seed), EKFConfig()),
    "bias 0.004, gyro noise inflated to 0.1": (SimConfig(seed=a.seed), EKFConfig(sigma_gyro=0.1)),
}
runs = {k: (simulate(s), e) for k, (s, e) in cases.items()}
res = {k: run_filter(d, e) for k, (d, e) in runs.items()}

fig, ax = plt.subplots(2, 2, figsize=(13, 8.5))
# 1. path zoom: 10 s of the biased, honest case
d, r = runs["bias 0.004, honest gyro noise"][0], res["bias 0.004, honest gyro noise"]
w = (d.t >= 30) & (d.t <= 40)
gw = (d.t[d.gnss_idx] >= 30) & (d.t[d.gnss_idx] <= 40)
ax[0, 0].plot(d.truth[w, 0], d.truth[w, 1], "k", lw=2, label="truth")
ax[0, 0].plot(d.gnss[gw, 0], d.gnss[gw, 1], ".", ms=6, label="GNSS fixes")
ax[0, 0].plot(r.est[w, 0], r.est[w, 1], lw=1.2, label="estimate (bias, honest noise)")
ax[0, 0].set(title="10 s zoom: the biased filter sits metres off; it trusts its gyro, so fixes barely move it",
             xlabel="x [m]", ylabel="y [m]"); ax[0, 0].axis("equal"); ax[0, 0].legend(fontsize=8)
# 2. position error, three cases
for k, rr in res.items():
    m = metrics(rr)
    ax[0, 1].plot(rr.t, rr.pos_err, lw=0.9, label=f"{k}: RMSE {m['rmse_m']:.2f} m")
ax[0, 1].set(title="Position error", xlabel="t [s]", ylabel="m", yscale="log", ylim=(0.01, 30))
ax[0, 1].legend(fontsize=8)
# 3. heading error, three cases
for k, rr in res.items():
    he = np.degrees([wrap(x) for x in rr.est[:, 2] - rr.truth[:, 2]])
    ax[1, 0].plot(rr.t, he, lw=0.9, label=k)
ax[1, 0].set(title="Heading error", xlabel="t [s]", ylabel="deg"); ax[1, 0].legend(fontsize=8)
# 4. the filter's own uncertainty: sawtooth over the first 3 s
r0 = res["no bias (model right)"]
s = r0.t <= 3
ax[1, 1].plot(r0.t[s], r0.pos_std[s], label="filter's 1σ (no bias case)")
for t_fix in r0.t[s][::20][1:]:
    ax[1, 1].axvline(t_fix, color="0.85", lw=0.8, zorder=0)
ax[1, 1].set(title="Uncertainty grows on predict, drops at each GNSS fix (grey lines)",
             xlabel="t [s]", ylabel="m")
ax[1, 1].legend(fontsize=8)
fig.tight_layout()
out = pathlib.Path("docs/m2_ekf.png"); out.parent.mkdir(exist_ok=True)
fig.savefig(out, dpi=110)

print(f"seed {a.seed}")
print(f"{'case':42s} {'RMSE m':>7s} {'heading deg':>12s} {'mean NIS':>9s} {'within 2σ':>10s}")
for k, rr in res.items():
    m = metrics(rr)
    print(f"{k:42s} {m['rmse_m']:7.2f} {m['heading_rmse_deg']:12.2f} {m['mean_nis']:9.2f} {m['within_2sigma']:10.0%}")
print("(a consistent filter has mean NIS near 2 and about 95 % of errors within 2σ)")
print(f"wrote {out}")
