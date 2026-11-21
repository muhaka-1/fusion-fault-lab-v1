"""M3 acceptance tests: the gyro-bias state and the wheel-speed update."""
import numpy as np
import pytest
from fusionlab.ekf import EKF, EKFConfig
from fusionlab.run import run_filter, metrics, NO_BIAS_STATE
from fusionlab.sim import SimConfig, simulate


def test_predict_subtracts_the_estimated_bias():
    ekf = EKF([0.0, 0.0, 0.0, 10.0])
    ekf.x[4] = 0.1                                  # the filter believes the gyro reads 0.1 too high
    for _ in range(100):
        ekf.predict(gyro=0.1, acc=0.0, dt=0.01)    # gyro says 0.1 rad/s ...
    assert abs(ekf.x[2]) < 1e-12                    # ... but the car is going straight


def test_bias_uncertainty_couples_into_heading():
    ekf = EKF([0.0, 0.0, 0.0, 10.0])
    assert ekf.P[2, 4] == 0.0                       # unrelated at the start
    for _ in range(100):
        ekf.predict(0.0, 0.0, 0.01)
    assert ekf.P[2, 4] < 0                          # a higher bias means a lower heading


def test_wheel_update_corrects_speed_only():
    ekf = EKF([5.0, 6.0, 0.3, 10.0])
    v_std = np.sqrt(ekf.P[3, 3])
    ok, nis, dof = ekf.update_wheel(10.5)
    assert ok and dof == 1
    assert 10.4 < ekf.x[3] < 10.5                   # nearly all the way: the wheel is precise (0.1 vs 1.0)
    assert np.allclose(ekf.x[[0, 1, 2]], [5.0, 6.0, 0.3])   # P0 is diagonal: nothing else moves yet
    assert np.sqrt(ekf.P[3, 3]) < 0.15 * v_std
    assert nis > 0


def test_nominal_error_is_below_raw_gnss_noise():
    # from the reference project: fusion must beat raw GNSS (~2.1 m radial RMS)
    for seed in (1, 2, 3):
        assert metrics(run_filter(simulate(SimConfig(seed=seed))))["rmse_m"] < 1.0


@pytest.mark.parametrize("bias", [0.0, 0.004, 0.01, -0.006])
def test_filter_learns_the_gyro_bias(bias):
    r = run_filter(simulate(SimConfig(gyro_bias=bias)))
    assert abs(r.est[-1, 4] - bias) < 0.0003
    assert abs(r.est[-1, 4] - bias) < 3 * r.bias_std[-1]    # and it knows how sure it is


def test_bias_is_learned_within_30_seconds():
    r = run_filter(simulate(SimConfig(seed=2)))
    late = r.t >= 30.0
    assert np.all(np.abs(r.est[late, 4] - 0.004) < 0.0004)  # within 10 % from 30 s on


def test_filter_is_consistent_again():
    m = metrics(run_filter(simulate(SimConfig())))
    assert 1.5 < m["mean_nis"] < 2.5                # GNSS: 2 degrees of freedom
    assert 0.7 < m["mean_wheel_nis"] < 1.3          # wheel: 1 degree of freedom
    assert m["within_2sigma"] > 0.9


def test_wheel_speed_makes_the_speed_estimate_much_better():
    d = simulate(SimConfig())
    with_wheel = metrics(run_filter(d))["speed_rmse_mps"]
    without = metrics(run_filter(d, use_wheel=False))["speed_rmse_mps"]
    assert with_wheel < without / 3                 # about 0.009 vs 0.066 m/s


def test_wheel_alone_does_not_fix_the_bias():
    m = metrics(run_filter(simulate(SimConfig()), EKFConfig(**NO_BIAS_STATE), gating=False))
    assert m["rmse_m"] > 3.0                        # only the bias state does that
