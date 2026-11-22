"""M12 check by eye: the C filter and the Python filter on the same data.

    python3 scripts/plot_m12.py      # in WSL, after building the C target; writes docs/m12_parity.png

Top: position error of both filters against the truth (the two lines lie on top of each other).
Bottom: how far apart C and Python are, per state, on a log scale. Also prints the speed of each.
"""
import argparse
import io
import pathlib
import subprocess
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))      # fusionlab, without PYTHONPATH or pip install
sys.path.insert(0, str(ROOT / "tests"))
from test_parity import BUILD, replay_input  # noqa: E402  (same input as the parity test)

from fusionlab.faults import SCENARIOS, apply_faults
from fusionlab.run import run_scenario
from fusionlab.sim import SimConfig, simulate

ap = argparse.ArgumentParser()
ap.add_argument("--scenario", default="gyro_bias_step", choices=list(SCENARIOS))
ap.add_argument("--seed", type=int, default=3)
a = ap.parse_args()

exe = BUILD / "nav_replay"
data = apply_faults(simulate(SimConfig(seed=a.seed)), SCENARIOS[a.scenario])
inp = replay_input(data, True)
t0 = time.perf_counter()
out = subprocess.run([str(exe)], input=inp, capture_output=True, text=True, check=True).stdout
t_c = time.perf_counter() - t0
c = np.genfromtxt(io.StringIO(out), delimiter=",", names=True, dtype=None, encoding=None)
t0 = time.perf_counter()
py = run_scenario(a.scenario, a.seed)
t_py = time.perf_counter() - t0

est_c = np.column_stack([c["x"], c["y"], c["psi"], c["v"], c["b"]])
diff = np.abs(est_c - py.est)
diff[:, 2] = np.abs((est_c[:, 2] - py.est[:, 2] + np.pi) % (2 * np.pi) - np.pi)
err_c = np.hypot(est_c[:, 0] - py.truth[:, 0], est_c[:, 1] - py.truth[:, 1])

fig, ax = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
for f in py.faults:
    ax[0].axvspan(f.t_start, min(f.t_end, py.t[-1]), color="0.9", label=f"{f.kind} active")
ax[0].plot(py.t, py.pos_err, lw=3, color="tab:blue", alpha=0.5, label="Python filter")
ax[0].plot(py.t, err_c, lw=1, color="tab:red", ls="--", label="C filter")
for t in py.recovery_times:
    ax[0].axvline(t, color="k", ls=":", lw=1)
ax[0].set(ylabel="position error [m]", yscale="log",
          title=f"{a.scenario}, seed {a.seed}: same data into both filters (dotted: recoveries)")
ax[0].legend(loc="upper left", fontsize=9)
for k, (name, unit) in enumerate([("x", "m"), ("y", "m"), ("heading", "rad"), ("speed", "m/s"), ("gyro bias", "rad/s")]):
    ax[1].plot(py.t, np.maximum(diff[:, k], 1e-18), lw=1, label=f"{name}: max {diff[:, k].max():.1e} {unit}")
ax[1].axhline(1e-6, color="k", ls="--", lw=1)
ax[1].text(py.t[-1], 1.5e-6, "parity limit for position: 1 µm", fontsize=9, ha="right")
ax[1].set(yscale="log", ylim=(1e-18, 1e-3), xlabel="t [s]", ylabel="|C − Python|",
          title="difference between the two filters (1e-16 is the last digit of a double)")
ax[1].legend(loc="upper left", fontsize=8, ncol=2)
fig.tight_layout()
outp = pathlib.Path(__file__).resolve().parents[1] / "docs" / "m12_parity.png"
fig.savefig(outp, dpi=110)

same_status = list(c["status"]) == list(py.status)
print(f"scenario {a.scenario}, seed {a.seed}, {len(py.t)} steps")
print(f"max |C - Python|: position {diff[:, :2].max():.2e} m, heading {diff[:, 2].max():.2e} rad, "
      f"speed {diff[:, 3].max():.2e} m/s, bias {diff[:, 4].max():.2e} rad/s")
print(f"status identical on every step: {same_status};  recoveries C {int(c['recoveries'][-1])}, Python {len(py.recovery_times)}")
print(f"time: C {t_c * 1000:.0f} ms (incl. starting the program and parsing text), Python {t_py * 1000:.0f} ms")
print(f"wrote {outp}")
