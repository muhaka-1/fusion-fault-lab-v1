"""M6 acceptance tests: lock-out recovery, its cost, and the benchmark."""
import numpy as np
import pytest
from fusionlab.benchmark import benchmark, to_markdown, MODES
from fusionlab.ekf import EKF, EKFConfig
from fusionlab.run import run_scenario, metrics

NO_RECOVERY = EKFConfig(lockout_rejects=0)


def _settled_ekf(cfg=None, gating=True):
    ekf = EKF([0.0, 0.0, 0.0, 10.0], cfg, gating=gating)
    for _ in range(20):
        ekf.update_gnss([0.0, 0.0])
    return ekf


# ---- the mechanism ---------------------------------------------------------------------
def test_recovery_fires_after_exactly_n_rejections_in_a_row():
    ekf = _settled_ekf(EKFConfig(lockout_rejects=5))
    for k in range(4):
        ekf.update_gnss([100.0, 100.0])
    assert ekf.recoveries == 0 and ekf.gnss_rejects == 4
    ekf.update_gnss([100.0, 100.0])
    assert ekf.recoveries == 1 and ekf.gnss_rejects == 0     # counter starts again


def test_recovery_inflates_position_heading_and_bias_but_not_speed():
    ekf = _settled_ekf(EKFConfig(lockout_rejects=5))
    P = ekf.P.copy()
    for _ in range(5):
        ekf.update_gnss([100.0, 100.0])
    ratio = np.diag(ekf.P) / np.diag(P)
    assert np.allclose(ratio[[0, 1, 2, 4]], 625.0)             # std x25 -> variance x625
    assert np.isclose(ratio[3], 1.0)                           # speed: the wheel keeps it honest


def test_an_accepted_fix_resets_the_count():
    ekf = _settled_ekf(EKFConfig(lockout_rejects=5))
    for _ in range(4):
        ekf.update_gnss([100.0, 100.0])
    ekf.update_gnss([0.0, 0.0])                                # a good fix in between
    for _ in range(4):
        ekf.update_gnss([100.0, 100.0])
    assert ekf.recoveries == 0                                 # never 5 in a row


@pytest.mark.parametrize("cfg, gating", [(EKFConfig(lockout_rejects=0), True), (EKFConfig(), False)])
def test_recovery_can_be_turned_off(cfg, gating):
    ekf = _settled_ekf(cfg, gating)
    for _ in range(200):
        ekf.update_gnss([100.0, 100.0])
    assert ekf.recoveries == 0


# ---- what it does --------------------------------------------------------------------------
def test_lockout_recovery_prevents_divergence_after_gyro_bias_step():
    # from the reference project: without recovery the filter diverges by hundreds of metres
    res = run_scenario("gyro_bias_step", 1, gating=True)
    assert res.pos_err.max() < 60.0


def test_after_recovery_the_filter_learns_the_new_bias():
    r = run_scenario("gyro_bias_step", 1)
    assert len(r.recovery_times) >= 1
    assert abs(r.est[-1, 4] - 0.024) < 0.001                    # 0.004 + 0.02 step
    assert r.pos_err[r.t > 100].max() < 1.0                     # and is back to normal accuracy


def test_recovery_never_fires_on_faults_gating_already_handles():
    for name in ("nominal", "gnss_jump", "wheel_scale", "gnss_dropout"):
        with_rec, without = run_scenario(name, 1), run_scenario(name, 1, NO_RECOVERY)
        assert with_rec.recovery_times == []
        assert metrics(with_rec)["rmse_m"] == metrics(without)["rmse_m"]


def test_the_price_recovery_makes_slow_drift_much_worse():
    """The known cost: after 10 s of rejections the filter re-opens the gate to a GNSS
    that really is drifting, follows it, and can report NOMINAL while far off."""
    with_rec = run_scenario("gnss_drift", 1)
    without = run_scenario("gnss_drift", 1, NO_RECOVERY)
    assert metrics(with_rec)["rmse_during_gnss_drift_m"] > 3 * metrics(without)["rmse_during_gnss_drift_m"]
    nominal_but_far = [(s == "NOMINAL") and (e > 20.0) for s, e in zip(with_rec.status, with_rec.pos_err)]
    assert any(nominal_but_far)


def test_no_lockout_length_fixes_both_faults():
    short, long_ = EKFConfig(lockout_rejects=25), EKFConfig(lockout_rejects=100)
    gyro = [metrics(run_scenario("gyro_bias_step", 1, c))["rmse_during_gyro_bias_step_m"] for c in (short, long_)]
    drift = [metrics(run_scenario("gnss_drift", 1, c))["rmse_during_gnss_drift_m"] for c in (short, long_)]
    assert gyro[0] < gyro[1] and drift[0] > drift[1]           # each setting wins one, loses the other
    assert max(gyro[0], drift[0]) > 10 and max(gyro[1], drift[1]) > 10


# ---- the benchmark -------------------------------------------------------------------------
def test_benchmark_runs_every_scenario_in_every_mode():
    rows = benchmark([1])
    assert len(rows) == 6 * len(MODES)
    md = to_markdown(rows, 1)
    assert md.count("\n| ") == 6 * len(MODES) + 1              # header + one row each
    assert rows["gyro_bias_step|gating+recovery"]["rmse_m"] < rows["gyro_bias_step|gating"]["rmse_m"] / 10
