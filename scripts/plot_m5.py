"""M5 check by eye: gating off vs on for each fault, with the monitor's status.

Runs with lock-out recovery off (added in M6) so it still shows the M5 filter.

    python scripts/plot_m5.py            # writes docs/m5_gating.png and prints a table
"""
import argparse, pathlib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from fusionlab.ekf import EKFConfig
from fusionlab.faults import SCENARIOS
from fusionlab.run import run_scenario, metrics
from fusionlab.stats import chi2_threshold

STATUS_COLOR = {"NOMINAL": "#3fa34d", "DEGRADED": "#f0b429", "UNAVAILABLE": "#d64545"}

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=1)
a = ap.parse_args()
names = [n for n in SCENARIOS if n != "nominal"]
off = {n: run_scenario(n, a.seed, gating=False) for n in names}
on = {n: run_scenario(n, a.seed, EKFConfig(lockout_rejects=0), gating=True) for n in names}


def status_strip(ax, r):
    """Thin coloured band under the plot: the monitor's status over time (gating on)."""
    st, t = r.status, r.t
    start = 0
    for i in range(1, len(st) + 1):
        if i == len(st) or st[i] != st[start]:
            ax.broken_barh([(t[start], t[min(i, len(t) - 1)] - t[start])], (0, 0.035),
                           transform=ax.get_xaxis_transform(), color=STATUS_COLOR[st[start]])
            start = i


fig, ax = plt.subplots(2, 3, figsize=(15, 8.5))
for k, n in enumerate(names):
    axk, f = ax.flat[k], on[n].faults[0]
    axk.axvspan(f.t_start, f.t_end, color="0.9", label="fault active")
    axk.plot(off[n].t, off[n].pos_err, lw=0.9, label=f"gating off: {metrics(off[n])['rmse_m']:.2f} m RMSE")
    axk.plot(on[n].t, on[n].pos_err, lw=0.9, label=f"gating on: {metrics(on[n])['rmse_m']:.2f} m RMSE")
    if n == "gyro_bias_step":
        axk.set_yscale("log"); axk.set_ylim(0.01, 1000)
    status_strip(axk, on[n])
    d = metrics(on[n])[f"detect_delay_{n}_s"]
    axk.set(title=f"{n}: detected after {d:.1f} s" if d is not None else f"{n}: not detected",
            xlabel="t [s]   (strip: status, gating on)", ylabel="position error [m]")
    axk.legend(fontsize=8, loc="upper left")
r, f = on["gnss_jump"], on["gnss_jump"].faults[0]
t, v, acc = np.array([g[0] for g in r.gnss_log]), np.array([g[1] for g in r.gnss_log]), np.array([g[2] for g in r.gnss_log])
axn = ax.flat[5]
axn.axvspan(f.t_start, f.t_end, color="0.9", label="jump active")
axn.semilogy(t[acc], v[acc], ".", ms=3, label="accepted fix")
axn.semilogy(t[~acc], v[~acc], "x", ms=5, color="#d64545", label="rejected fix")
axn.axhline(chi2_threshold(0.999, 2), color="k", ls="--", lw=1, label="gate 13.8")
axn.set(title="Gating on: every jumped fix lands above the gate", xlabel="t [s]", ylabel="GNSS NIS")
axn.legend(fontsize=8)
fig.tight_layout()
out = pathlib.Path("docs/m5_gating.png"); out.parent.mkdir(exist_ok=True)
fig.savefig(out, dpi=100)

print(f"seed {a.seed}   status strip colours: green NOMINAL, amber DEGRADED, red UNAVAILABLE")
print(f"{'fault':16s} {'RMSE off':>9s} {'RMSE on':>9s} {'detected after':>15s} {'GNSS rejected':>14s} {'wheel rejected':>15s}")
for n in names:
    mo, mn = metrics(off[n]), metrics(on[n])
    d = mn[f"detect_delay_{n}_s"]
    print(f"{n:16s} {mo['rmse_m']:7.2f} m {mn['rmse_m']:7.2f} m {('%.1f s' % d) if d is not None else '-':>15s} "
          f"{mn['gnss_reject_rate']:13.1%} {mn['wheel_reject_rate']:14.1%}")
print(f"wrote {out}")
