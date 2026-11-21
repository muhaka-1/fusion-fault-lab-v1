"""M2 acceptance tests: the 4-state EKF (IMU predict + GNSS update)."""
import numpy as np
from fusionlab.ekf import EKF, EKFConfig, wrap
from fusionlab.run import run_filter, metrics
from fusionlab.sim import SimConfig, simulate


def test_wrap_keeps_angles_in_minus_pi_to_pi():
    assert wrap(0.0) == 0.0
    assert np.isclose(wrap(np.pi + 0.1), -np.pi + 0.1)
    assert np.isclose(wrap(-np.pi - 0.1), np.pi - 0.1)
    assert np.isclose(wrap(4 * np.pi + 0.5), 0.5)


def test_predict_drives_along_the_heading():
    ekf = EKF([0.0, 0.0, np.pi / 2, 10.0])          # heading north at 10 m/s
    for _ in range(100):                            # 1 s
        ekf.predict(gyro=0.0, acc=0.0, dt=0.01)
    assert np.allclose(ekf.x, [0.0, 10.0, np.pi / 2, 10.0], atol=1e-9)


def test_predict_turns_and_accelerates():
    ekf = EKF([0.0, 0.0, 0.0, 10.0])
    for _ in range(100):
        ekf.predict(gyro=0.1, acc=0.5, dt=0.01)
    assert np.isclose(ekf.x[2], 0.1)                # 0.1 rad/s for 1 s
    assert np.isclose(ekf.x[3], 10.5)               # 0.5 m/s^2 for 1 s


def test_predict_grows_uncertainty_and_keeps_it_symmetric():
    ekf = EKF([0.0, 0.0, 0.0, 10.0])
    p0 = ekf.P.copy()
    for _ in range(100):
        ekf.predict(0.0, 0.0, 0.01)
    assert ekf.P[2, 2] > p0[2, 2] and ekf.P[0, 0] > p0[0, 0]
    assert np.array_equal(ekf.P, ekf.P.T)


def test_gnss_update_pulls_towards_the_fix_and_shrinks_uncertainty():
    ekf = EKF([0.0, 0.0, 0.0, 10.0])               # P0 says +-5 m
    before = ekf.pos_std()
    nis = ekf.update_gnss([3.0, -4.0])
    assert 2.0 < ekf.x[0] < 3.0 and -4.0 < ekf.x[1] < -3.0   # moved most of the way: the fix is more precise
    assert ekf.pos_std() < 0.5 * before
    assert nis > 0


def test_covariance_stays_positive_definite_over_a_whole_drive():
    d = simulate(SimConfig(duration=30))
    ekf = EKF(d.truth[0])
    gnss_at = {int(i): k for k, i in enumerate(d.gnss_idx)}
    for i in range(1, len(d.t)):
        ekf.predict(d.gyro[i], d.acc[i], d.cfg.dt)
        if i in gnss_at:
            ekf.update_gnss(d.gnss[gnss_at[i]])
    assert np.all(np.linalg.eigvalsh(ekf.P) > 0)


def test_with_a_correct_model_the_filter_beats_gnss_and_is_consistent():
    d = simulate(SimConfig(gyro_bias=0.0))          # no bias: the filter's model is right
    m = metrics(run_filter(d))
    raw = np.sqrt(np.mean(np.sum((d.gnss - d.truth[d.gnss_idx, :2]) ** 2, axis=1)))
    assert m["rmse_m"] < 0.5 < raw                  # about 0.26 m vs 2.1 m raw GNSS
    assert 1.5 < m["mean_nis"] < 2.5                # NIS averages its dof (2) when the model is right
    assert m["within_2sigma"] > 0.9


def test_unmodelled_gyro_bias_makes_the_filter_overconfident():
    """The known flaw that M3 fixes: the filter trusts a biased gyro."""
    m = metrics(run_filter(simulate(SimConfig())))  # default bias 0.004 rad/s
    assert m["rmse_m"] > 3.0                        # worse than raw GNSS
    assert m["mean_nis"] > 10                       # innovations far bigger than the filter expects
    assert m["within_2sigma"] < 0.5


def test_inflating_gyro_noise_hides_the_bias_at_a_price():
    m = metrics(run_filter(simulate(SimConfig()), EKFConfig(sigma_gyro=0.1)))
    assert m["rmse_m"] < 1.0                        # rescued, but the filter now distrusts a good gyro
