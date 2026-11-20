"""M1 acceptance tests: the simulator."""
import numpy as np
from fusionlab.sim import SimConfig, simulate


def test_simulator_is_deterministic_per_seed():
    # copied from the reference project (tests/test_fusion.py)
    a, b = simulate(SimConfig(seed=3)), simulate(SimConfig(seed=3))
    assert np.allclose(a.gnss, b.gnss) and np.allclose(a.gyro, b.gyro)
    assert not np.allclose(a.gnss, simulate(SimConfig(seed=4)).gnss)


def test_sensor_rates_and_sample_counts():
    d = simulate(SimConfig(duration=120.0))
    assert len(d.t) == 12001                       # 100 Hz IMU, both ends included
    assert np.allclose(np.diff(d.t[d.gnss_idx]), 0.2)    # GNSS 5 Hz
    assert np.allclose(np.diff(d.t[d.wheel_idx]), 0.02)  # wheel 50 Hz
    assert len(d.gnss) == 600 and len(d.wheel) == 6000


def test_truth_is_consistent_with_its_own_speed_and_heading():
    d = simulate()
    x, y, psi, v = d.truth.T
    dx, dy = np.diff(x) / d.cfg.dt, np.diff(y) / d.cfg.dt
    vm = 0.5 * (v[1:] + v[:-1])
    assert np.allclose(np.hypot(dx, dy), vm, atol=0.01)       # moves at its speed
    assert np.all(d.truth[0] == [0.0, 0.0, 0.0, 12.0])        # starts at the origin, heading east


def test_noise_has_the_configured_size():
    d = simulate(SimConfig(seed=7))
    gnss_err = d.gnss - d.truth[d.gnss_idx, :2]
    assert abs(gnss_err.std() - d.cfg.sigma_gnss) < 0.1      # 1.5 m per axis
    wheel_err = d.wheel - d.truth[d.wheel_idx, 3]
    assert abs(wheel_err.std() - d.cfg.sigma_wheel) < 0.01


def test_gyro_carries_the_bias():
    d = simulate(SimConfig(seed=7))
    true_rate = np.gradient(d.truth[:, 2], d.cfg.dt)
    assert abs(np.mean(d.gyro - true_rate) - d.cfg.gyro_bias) < 0.0005
