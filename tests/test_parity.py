"""M12 parity test: the C filter (target/src/nav) must reproduce the Python filter step by step.

Both get exactly the same sensor samples. tools/nav_replay.c runs the C EKF + monitor over them;
fusionlab.run runs the Python EKF + monitor. Every scenario, gating on and off, 12001 steps each.

Needs nav_replay built for this OS (cmake -S target -B target/build && cmake --build target/build).
Skipped otherwise, e.g. in Git Bash on Windows when the C target was built in WSL.
"""
import io
import pathlib
import subprocess

import numpy as np
import pytest

from fusionlab.faults import SCENARIOS, apply_faults
from fusionlab.run import run_scenario
from fusionlab.sim import SimConfig, simulate

BUILD = pathlib.Path(__file__).resolve().parents[1] / "target" / "build"
SEED = 3


def nav_replay() -> pathlib.Path:
    for exe in (BUILD / "nav_replay", BUILD / "nav_replay.exe"):
        if exe.exists():
            try:
                subprocess.run([str(exe)], input="", capture_output=True, timeout=5)
                return exe
            except OSError:
                pytest.skip("nav_replay was built for another OS (e.g. in WSL): run this test there")
    pytest.skip("nav_replay not built (cmake -S target -B target/build && cmake --build target/build)")


def replay_input(data, gating: bool) -> str:
    """The sensor samples as CSV, exactly as the Python filter sees them. repr() keeps every bit."""
    g = {int(i): k for k, i in enumerate(data.gnss_idx)}
    w = {int(i): k for k, i in enumerate(data.wheel_idx)}
    x0 = data.truth[0]
    lines = [",".join(repr(float(v)) for v in x0[:4]) + f",{int(gating)},{float(data.cfg.dt)!r}"]
    for i, t in enumerate(data.t):
        wv = repr(float(data.wheel[w[i]])) if i in w else ""
        gx, gy = (repr(float(data.gnss[g[i]][0])), repr(float(data.gnss[g[i]][1]))) if i in g else ("", "")
        lines.append(f"{float(t)!r},{float(data.gyro[i])!r},{float(data.acc[i])!r},{wv},{gx},{gy}")
    return "\n".join(lines) + "\n"


def run_c(exe, name: str, gating: bool):
    data = apply_faults(simulate(SimConfig(seed=SEED)), SCENARIOS[name])
    out = subprocess.run([str(exe)], input=replay_input(data, gating), capture_output=True, text=True,
                         check=True).stdout
    return np.genfromtxt(io.StringIO(out), delimiter=",", names=True, dtype=None, encoding=None)


@pytest.mark.parametrize("name", list(SCENARIOS))
@pytest.mark.parametrize("gating", [True, False])
def test_c_filter_matches_python(name, gating):
    c = run_c(nav_replay(), name, gating)
    ref = run_scenario(name, SEED, gating=gating)
    assert len(c) == len(ref.t)
    est = np.column_stack([c["x"], c["y"], c["psi"], c["v"], c["b"]])
    dpsi = (est[:, 2] - ref.est[:, 2] + np.pi) % (2 * np.pi) - np.pi
    assert np.max(np.abs(est[:, :2] - ref.est[:, :2])) < 1e-6          # position: 1 micrometre
    assert np.max(np.abs(dpsi)) < 1e-9                                  # heading
    assert np.max(np.abs(est[:, 3:] - ref.est[:, 3:])) < 1e-9          # speed, gyro bias
    assert list(c["status"]) == list(ref.status)                        # NOMINAL/DEGRADED/... every step
    c_ok = c["gnss_ok"][c["gnss_ok"] >= 0].astype(bool)                 # same accept/reject on every fix
    assert np.array_equal(c_ok, np.array([bool(a) for _, _, a, _ in ref.gnss_log]))
    assert int(c["recoveries"][-1]) == len(ref.recovery_times)
