"""M4 check by eye: what each fault does to the filter when nothing protects it.

Runs with gating=False (added in M5) so it still shows the unprotected filter.

    python scripts/plot_m4.py            # writes docs/m4_faults.png and prints a table
"""
import argparse, pathlib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from fusionlab.faults import SCENARIOS
from fusionlab.run import run_scenario, metrics

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=1)
a = ap.parse_args()
nominal = run_scenario("nominal", a.seed, gating=False)
names = [n for n in SCENARIOS if n != "nominal"]
res = {n: run_scenario(n, a.seed, gating=False) for n in names}

fig, ax = plt.subplots(2, 3, figsize=(15, 8.5))
for k, n in enumerate(names):
    r, axk = res[n], ax.flat[k]
    f = r.faults[0]
    axk.axvspan(f.t_start, f.t_end, color="0.85", label="fault active")
    axk.plot(nominal.t, nominal.pos_err, color="0.5", lw=0.8, label="no fault")
    axk.plot(r.t, r.pos_err, lw=1.0, label=f"with {n}")
    m = metrics(r)
    axk.set(title=f"{n} ({f.magnitude:g}): RMSE during {m[f'rmse_during_{n}_m']:.1f} m",
            xlabel="t [s]", ylabel="position error [m]")
    axk.legend(fontsize=8, loc="upper left")
r = res["gnss_jump"]
f = r.faults[0]
t, v = np.array(r.nis).T
axn = ax.flat[5]
axn.axvspan(f.t_start, f.t_end, color="0.85", label="jump active")
axn.semilogy(t, v, ".", ms=3, label="GNSS NIS")
axn.axhline(2, color="k", ls="--", lw=1, label="expected average (2)")
axn.set(title="The NIS explodes during the jump: the signal M5 will use", xlabel="t [s]", ylabel="NIS")
axn.legend(fontsize=8)
fig.tight_layout()
out = pathlib.Path("docs/m4_faults.png"); out.parent.mkdir(exist_ok=True)
fig.savefig(out, dpi=100)

print(f"seed {a.seed}, no fault handling (nominal RMSE {metrics(nominal)['rmse_m']:.2f} m)")
print(f"{'fault':16s} {'RMSE during':>12s} {'max during':>11s} {'RMSE 20 s after':>16s} {'max NIS':>9s}")
for n in names:
    m = metrics(res[n])
    print(f"{n:16s} {m[f'rmse_during_{n}_m']:10.2f} m {m[f'max_during_{n}_m']:9.1f} m "
          f"{m[f'rmse_after_{n}_m']:14.2f} m {m[f'max_nis_during_{n}']:9.0f}")
print("(gyro_bias_step lasts to the end of the run, so it has no 'after'; a dropout has no NIS: no fixes)")
print(f"wrote {out}")
