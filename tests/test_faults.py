"""M4 acceptance tests: fault injection and what each fault does to an unprotected filter.

Since M5 the filter gates by default; the "what the faults do" tests use gating=False."""
import numpy as np
import pytest
from fusionlab.faults import SCENARIOS, Fault, apply_faults
from fusionlab.run import run_filter, run_scenario, metrics
from fusionlab.sim import SimConfig, simulate


# ---- the injection itself ------------------------------------------------------------
def test_unknown_fault_raises():
    # from the reference project
    with pytest.raises(ValueError):
        apply_faults(simulate(SimConfig(duration=5)), [Fault("nope", 0, 1)])


def test_fault_injection_removes_gnss_in_window():
    # from the reference project
    d = simulate(SimConfig(duration=20))
    f = apply_faults(d, [Fault("gnss_dropout", 5.0, 10.0)])
    assert len(f.gnss) < len(d.gnss)
    assert not np.any((f.t[f.gnss_idx] >= 5.0) & (f.t[f.gnss_idx] <= 10.0))


def test_jump_moves_fixes_by_its_magnitude_only_inside_the_window():
    d = simulate(SimConfig(duration=20))
    f = apply_faults(d, [Fault("gnss_jump", 5.0, 10.0, 40.0)])
    shift = np.hypot(*(f.gnss - d.gnss).T)
    inside = (d.t[d.gnss_idx] >= 5.0) & (d.t[d.gnss_idx] <= 10.0)
    assert np.allclose(shift[inside], 40.0) and np.allclose(shift[~inside], 0.0)


def test_drift_grows_linearly_with_time():
    d = simulate(SimConfig(duration=30))
    f = apply_faults(d, [Fault("gnss_drift", 10.0, 20.0, 1.0)])
    shift = np.hypot(*(f.gnss - d.gnss).T)
    t = d.t[d.gnss_idx]
    inside = (t >= 10.0) & (t <= 20.0)
    assert np.allclose(shift[inside], t[inside] - 10.0)      # 1 m/s ramp: 0 m at start, 10 m at the end
    assert np.allclose(shift[t > 20.0], 0.0)                 # gone after the window


def test_wheel_scale_and_gyro_bias_step():
    d = simulate(SimConfig(duration=20))
    f = apply_faults(d, [Fault("wheel_scale", 5.0, 10.0, 0.15), Fault("gyro_bias_step", 5.0, 10.0, 0.02)])
    w = (d.t[d.wheel_idx] >= 5.0) & (d.t[d.wheel_idx] <= 10.0)
    assert np.allclose(f.wheel[w], 1.15 * d.wheel[w]) and np.allclose(f.wheel[~w], d.wheel[~w])
    g = (d.t >= 5.0) & (d.t <= 10.0)
    assert np.allclose(f.gyro[g] - d.gyro[g], 0.02) and np.allclose(f.gyro[~g], d.gyro[~g])


def test_faults_never_change_the_truth_or_the_original_data():
    d = simulate(SimConfig(duration=20))
    before = d.gnss.copy()
    f = apply_faults(d, [Fault("gnss_jump", 5.0, 10.0, 40.0)])
    assert np.array_equal(d.gnss, before)                   # a copy was changed, not the original
    assert np.array_equal(f.truth, d.truth)                  # ground truth is untouched


def test_scenarios_cover_all_five_fault_kinds():
    kinds = {f.kind for faults in SCENARIOS.values() for f in faults}
    assert kinds == {"gnss_dropout", "gnss_jump", "gnss_drift", "wheel_scale", "gyro_bias_step"}
    assert SCENARIOS["nominal"] == []


def test_run_scenario_nominal_is_the_plain_filter():
    a = metrics(run_scenario("nominal", 1))
    b = metrics(run_filter(simulate(SimConfig(seed=1))))
    assert a["rmse_m"] == b["rmse_m"]


# ---- what the faults do (no fault handling yet) ----------------------------------------
def test_a_short_dropout_is_harmless_for_a_good_filter():
    m = metrics(run_scenario("gnss_dropout", 1, gating=False))
    assert m["max_during_gnss_dropout_m"] < 2.0               # 20 s on IMU + wheel + learned bias


@pytest.mark.parametrize("name", ["gnss_jump", "gnss_drift", "wheel_scale", "gyro_bias_step"])
def test_measurement_faults_hurt_an_unprotected_filter(name):
    m = metrics(run_scenario(name, 1, gating=False))
    assert m[f"rmse_during_{name}_m"] > 5.0                   # nominal is ~0.3 m


@pytest.mark.parametrize("name", ["gnss_jump", "gnss_drift", "wheel_scale", "gyro_bias_step"])
def test_the_nis_shows_every_measurement_fault(name):
    m = metrics(run_scenario(name, 1, gating=False))
    assert m[f"max_nis_during_{name}"] > 100                  # nominal NIS averages 2: this is the M5 detector


def test_the_damage_outlasts_the_fault():
    m = metrics(run_scenario("gnss_jump", 1, gating=False))
    assert m["rmse_after_gnss_jump_m"] > 3.0                  # still metres off 20 s after the jump ended
