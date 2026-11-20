"""M1 check by eye: true path vs GNSS, and what the IMU really measures.

    python scripts/plot_m1.py            # writes docs/m1_sim.png
    python scripts/plot_m1.py --seed 4   # another random run
"""
import argparse, pathlib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from fusionlab.sim import SimConfig, simulate

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=1)
a = ap.parse_args()
d = simulate(SimConfig(seed=a.seed))
true_rate = np.gradient(d.truth[:, 2], d.cfg.dt)

# What a 60 s drive on the gyro alone does: integrate it and compare with the true heading
psi_gyro = np.cumsum(d.gyro) * d.cfg.dt
drift_60 = np.degrees(psi_gyro[6000] - d.truth[6000, 2])

fig, ax = plt.subplots(2, 2, figsize=(12, 8))
ax[0, 0].plot(d.truth[:, 0], d.truth[:, 1], "k", lw=1.5, label="true path")
ax[0, 0].plot(d.gnss[:, 0], d.gnss[:, 1], ".", ms=3, alpha=0.5, label="GNSS fixes (5 Hz)")
ax[0, 0].set(title="Path", xlabel="x [m]", ylabel="y [m]"); ax[0, 0].axis("equal"); ax[0, 0].legend()
ax[0, 1].plot(d.t, d.truth[:, 3], "k", label="true speed")
ax[0, 1].plot(d.t[d.wheel_idx], d.wheel, ".", ms=1, alpha=0.3, label="wheel speed (50 Hz)")
ax[0, 1].set(title="Speed", xlabel="t [s]", ylabel="m/s"); ax[0, 1].legend()
ax[1, 0].plot(d.t, d.gyro, lw=0.4, alpha=0.6, label="gyro (100 Hz)")
ax[1, 0].plot(d.t, true_rate, "k", lw=1, label="true yaw rate")
ax[1, 0].set(title="Yaw rate", xlabel="t [s]", ylabel="rad/s"); ax[1, 0].legend()
ax[1, 1].plot(d.t, np.degrees(psi_gyro - d.truth[:, 2]))
ax[1, 1].set(title=f"Heading error from the gyro alone ({drift_60:.1f} deg after 60 s)", xlabel="t [s]", ylabel="deg")
fig.tight_layout()
out = pathlib.Path("docs/m1_sim.png"); out.parent.mkdir(exist_ok=True)
fig.savefig(out, dpi=110)

g = d.gnss - d.truth[d.gnss_idx, :2]
print(f"seed {a.seed}: {len(d.t)} IMU, {len(d.wheel)} wheel, {len(d.gnss)} GNSS samples over {d.t[-1]:.0f} s")
print(f"GNSS error std per axis: {g.std():.2f} m (configured {d.cfg.sigma_gnss})")
print(f"gyro mean error: {np.mean(d.gyro - true_rate):.4f} rad/s (configured bias {d.cfg.gyro_bias})")
print(f"heading drift from the gyro alone after 60 s: {drift_60:.1f} deg")
print(f"wrote {out}")
