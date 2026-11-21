"""M6 check by eye: what lock-out recovery fixes, what it breaks, and why no setting wins both.

    python scripts/plot_m6.py            # writes docs/m6_recovery.png (~30 s)
"""
import argparse, pathlib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from fusionlab.ekf import EKFConfig
from fusionlab.run import run_scenario, metrics

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=1)
a = ap.parse_args()
MODES = {"no fault handling": (EKFConfig(), False), "gating, no recovery": (EKFConfig(lockout_rejects=0), True),
         "gating + recovery": (EKFConfig(), True)}
runs = {n: {m: run_scenario(n, a.seed, c, gating=g) for m, (c, g) in MODES.items()} for n in ("gyro_bias_step", "gnss_drift")}

fig, ax = plt.subplots(2, 2, figsize=(13, 8.5))
for axk, n in ((ax[0, 0], "gyro_bias_step"), (ax[0, 1], "gnss_drift")):
    f = runs[n]["gating + recovery"].faults[0]
    axk.axvspan(f.t_start, f.t_end, color="0.9", label="fault active")
    for m, r in runs[n].items():
        axk.plot(r.t, r.pos_err, lw=1, label=f"{m}: {metrics(r)[f'rmse_during_{n}_m']:.1f} m during")
    for t in runs[n]["gating + recovery"].recovery_times:
        axk.axvline(t, color="k", ls=":", lw=1)
    axk.set_yscale("log"); axk.set_ylim(0.05, 1000)
    axk.set(title=f"{n}: recoveries at the dotted lines", xlabel="t [s]", ylabel="position error [m]")
    axk.legend(fontsize=8, loc="upper left")
ax[0, 0].set_title("gyro_bias_step: recovery rescues the filter (dotted line)")
ax[0, 1].set_title("gnss_drift: each recovery lets the drifting GNSS back in")

for m, r in runs["gyro_bias_step"].items():
    ax[1, 0].plot(r.t, r.est[:, 4], lw=1, label=m)
ax[1, 0].axhline(0.004, color="0.5", ls="--", lw=1)
ax[1, 0].plot([50, 120], [0.024, 0.024], color="k", ls="--", lw=1, label="true bias (0.004, then 0.024)")
ax[1, 0].set(title="Bias estimate after the step: only recovery learns the new value",
             xlabel="t [s]", ylabel="rad/s", ylim=(-0.005, 0.03))
ax[1, 0].legend(fontsize=8, loc="upper left")

lengths = [10, 25, 50, 75, 100, 150]
seeds = [a.seed, a.seed + 1, a.seed + 2]
g = [np.mean([metrics(run_scenario("gyro_bias_step", s, EKFConfig(lockout_rejects=L)))["rmse_during_gyro_bias_step_m"] for s in seeds]) for L in lengths]
d = [np.mean([metrics(run_scenario("gnss_drift", s, EKFConfig(lockout_rejects=L)))["rmse_during_gnss_drift_m"] for s in seeds]) for L in lengths]
ax[1, 1].plot(lengths, g, "o-", label="gyro_bias_step")
ax[1, 1].plot(lengths, d, "s-", label="gnss_drift")
ax[1, 1].set_yscale("log")
ax[1, 1].set(title=f"Lock-out length: no setting wins both (mean of seeds {seeds[0]}-{seeds[-1]})",
             xlabel="GNSS rejections in a row before recovery (5 = 1 s)", ylabel="RMSE during fault [m]")
ax[1, 1].legend(fontsize=8)
fig.tight_layout()
out = pathlib.Path("docs/m6_recovery.png"); out.parent.mkdir(exist_ok=True)
fig.savefig(out, dpi=100)

print(f"seed {a.seed}")
print(f"{'scenario':16s} {'mode':22s} {'RMSE during':>12s} {'max error':>10s} {'recoveries at':>20s}")
for n, rr in runs.items():
    for m, r in rr.items():
        mm = metrics(r)
        print(f"{n:16s} {m:22s} {mm[f'rmse_during_{n}_m']:10.2f} m {mm['max_err_m']:8.1f} m "
              f"{', '.join(f'{t:.1f}' for t in r.recovery_times) or '-':>20s}")
print("lock-out length sweep (RMSE during, m):")
for L, gg, dd in zip(lengths, g, d):
    print(f"  {L:4d} fixes ({L / 5:4.0f} s): gyro {gg:7.2f}   drift {dd:6.2f}")
print(f"wrote {out}")
