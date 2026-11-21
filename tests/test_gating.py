"""M5 acceptance tests: NIS gating and the fault monitor.

Since M6 the filter also has lock-out recovery; the tests about gating alone turn it off (NO_RECOVERY)."""
import numpy as np
import pytest
from fusionlab.ekf import EKF, EKFConfig
from fusionlab.monitor import Monitor, MonitorConfig, NOMINAL, DEGRADED, UNAVAILABLE
from fusionlab.run import run_scenario, metrics
from fusionlab.stats import chi2_threshold

NO_RECOVERY = EKFConfig(lockout_rejects=0)


# ---- building blocks -----------------------------------------------------------------
def test_gates_are_the_chi_square_99_9_percent_values():
    assert np.isclose(chi2_threshold(0.999, 2), 13.8155, atol=1e-4)    # GNSS, 2 dof
    assert np.isclose(chi2_threshold(0.999, 1), 10.8276, atol=1e-4)    # wheel, 1 dof


def test_a_rejected_measurement_changes_nothing():
    ekf = EKF([0.0, 0.0, 0.0, 10.0])
    for _ in range(20):
        ekf.update_gnss([0.0, 0.0])                   # settle: P becomes small
    x, P = ekf.x.copy(), ekf.P.copy()
    ok, nis, dof = ekf.update_gnss([40.0, 40.0])
    assert not ok and nis > 13.8 and dof == 2
    assert np.array_equal(ekf.x, x) and np.array_equal(ekf.P, P)


def test_without_gating_everything_is_accepted():
    ekf = EKF([0.0, 0.0, 0.0, 10.0], gating=False)
    for _ in range(20):
        ekf.update_gnss([0.0, 0.0])
    ok, nis, _ = ekf.update_gnss([40.0, 40.0])
    assert ok and nis > 13.8


def test_monitor_window_raises_and_clears_the_alarm():
    mon = Monitor()
    for k in range(10):
        mon.on_update("gnss", 0.2 * k, 2.0, True)     # sum 20 < 45.3
    assert not mon.alarm["gnss"]
    for k in range(10):
        mon.on_update("gnss", 2.0 + 0.2 * k, 6.0, True)   # each below the gate 13.8, but sum 60 > 45.3
    assert mon.alarm["gnss"]
    for k in range(10):
        mon.on_update("gnss", 4.0 + 0.2 * k, 2.0, True)
    assert not mon.alarm["gnss"]


def test_monitor_status_levels():
    mon = Monitor()
    mon.on_update("gnss", 10.0, 1.0, True)
    small, big = np.diag([1.0, 1.0, 0, 0, 0]), np.diag([20.0, 20.0, 0, 0, 0])
    assert mon.status(10.5, small) == NOMINAL
    assert mon.status(11.5, small) == DEGRADED        # no accepted fix for > 1 s
    assert mon.status(10.5, big) == UNAVAILABLE       # position sigma sqrt(40) = 6.3 m > 5 m


# ---- whole scenarios (adapted from the reference project) ------------------------------
def test_nominal_false_rejection_rate_is_small():
    rates = [metrics(run_scenario("nominal", s))["gnss_reject_rate"] for s in range(1, 6)]
    assert np.mean(rates) < 0.01                      # gate is 99.9 %, allow some margin


def test_gnss_jump_is_rejected_and_gating_helps():
    on = metrics(run_scenario("gnss_jump", 1, gating=True))
    off = metrics(run_scenario("gnss_jump", 1, gating=False))
    assert on["detect_delay_gnss_jump_s"] is not None and on["detect_delay_gnss_jump_s"] < 1.0
    assert on["rmse_during_gnss_jump_m"] < 0.5 * off["rmse_during_gnss_jump_m"]


def test_every_jumped_fix_is_rejected():
    r = run_scenario("gnss_jump", 1)
    inside = [ok for t, _, ok, _ in r.gnss_log if 50.0 <= t <= 56.0]
    assert len(inside) > 25 and not any(inside)


def test_dropout_flags_degraded_and_error_stays_bounded():
    res = run_scenario("gnss_dropout", 1)
    m = metrics(res)
    assert m["degraded_flag_delay_s"] is not None and m["degraded_flag_delay_s"] <= 1.5
    t = res.t
    assert res.pos_err[(t >= 40) & (t <= 60)].max() < 30.0   # dead reckoning drifts but stays bounded
    assert res.pos_err[t >= 65].mean() < 1.5                  # recovers after GNSS returns
    assert res.status[-1] == NOMINAL


def test_status_goes_through_degraded_during_dropout():
    res = run_scenario("gnss_dropout", 1)
    st, t = np.array(res.status), res.t
    assert set(st[(t > 45) & (t < 55)]) <= {DEGRADED, UNAVAILABLE}
    assert set(st[(t > 20) & (t < 35)]) == {NOMINAL}


def test_wheel_speed_scale_fault_is_detected_and_contained():
    on, off = metrics(run_scenario("wheel_scale", 1)), metrics(run_scenario("wheel_scale", 1, gating=False))
    assert on["detect_delay_wheel_scale_s"] is not None and on["detect_delay_wheel_scale_s"] < 2.0
    assert on["rmse_during_wheel_scale_m"] < 0.5 < 5.0 < off["rmse_during_wheel_scale_m"]


def test_slow_drift_is_caught_late_and_only_partly_contained():
    on, off = metrics(run_scenario("gnss_drift", 1, NO_RECOVERY)), metrics(run_scenario("gnss_drift", 1, gating=False))
    assert on["detect_delay_gnss_drift_s"] > 1.0      # a ramp starts below the gate: seconds, not 0
    assert on["rmse_during_gnss_drift_m"] < 0.5 * off["rmse_during_gnss_drift_m"]
    assert on["rmse_during_gnss_drift_m"] > 1.0       # but not as clean as the jump


def test_gyro_bias_step_makes_the_gated_filter_diverge():
    """The known flaw that M6 fixes: once its own heading is wrong, the filter rejects every
    GNSS fix (each looks like a fault) and dead-reckons away, while still feeling sure."""
    r = run_scenario("gyro_bias_step", 1, NO_RECOVERY)
    assert r.pos_err.max() > 100.0
    late = [ok for t, _, ok, _ in r.gnss_log if t > 70.0]
    assert not any(late)                              # nothing accepted any more
    assert r.status[-1] != UNAVAILABLE                # and its own sigma does not even admit it
