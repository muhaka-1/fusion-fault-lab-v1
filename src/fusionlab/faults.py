"""M4: fault injection with known ground truth (when, where, how large).

Faults are applied to the recorded sensor streams, so the truth is untouched and every
later module can be measured against it. The filter is not told about any fault: finding
them is its job (from M5 on). Five kinds, each a real failure mode:

    gnss_dropout     no fixes at all (tunnel, parking garage)
    gnss_jump        position jumps by a fixed offset (multipath near buildings)
    gnss_drift       position slowly ramps away (receiver error growing)
    wheel_scale      wheel speed reads too high by a factor (slip, wrong tyre radius)
    gyro_bias_step   the gyro bias jumps (temperature, shock)
"""
from __future__ import annotations
from dataclasses import dataclass, replace
import numpy as np
from .sim import SimData


@dataclass(frozen=True)
class Fault:
    kind: str          # gnss_dropout | gnss_jump | gnss_drift | wheel_scale | gyro_bias_step
    t_start: float
    t_end: float
    magnitude: float = 0.0   # jump [m], drift [m/s], scale error [-], bias [rad/s]

    @property
    def sensor(self) -> str:
        return "gnss" if self.kind.startswith("gnss") else ("wheel" if self.kind == "wheel_scale" else "imu")


def apply_faults(data: SimData, faults: list[Fault]) -> SimData:
    """Return a copy of ``data`` with faults applied to the sensor streams."""
    t = data.t
    gnss, gnss_idx = data.gnss.copy(), data.gnss_idx.copy()
    wheel, gyro = data.wheel.copy(), data.gyro.copy()
    for f in faults:
        if f.kind == "gnss_dropout":
            keep = ~((t[gnss_idx] >= f.t_start) & (t[gnss_idx] <= f.t_end))
            gnss, gnss_idx = gnss[keep], gnss_idx[keep]
        elif f.kind == "gnss_jump":
            m = (t[gnss_idx] >= f.t_start) & (t[gnss_idx] <= f.t_end)
            gnss[m] += np.array([f.magnitude, f.magnitude]) / np.sqrt(2)
        elif f.kind == "gnss_drift":
            m = (t[gnss_idx] >= f.t_start) & (t[gnss_idx] <= f.t_end)
            ramp = (t[gnss_idx][m] - f.t_start) * f.magnitude
            gnss[m] += np.column_stack([ramp, ramp]) / np.sqrt(2)
        elif f.kind == "wheel_scale":
            m = (t[data.wheel_idx] >= f.t_start) & (t[data.wheel_idx] <= f.t_end)
            wheel[m] *= (1.0 + f.magnitude)
        elif f.kind == "gyro_bias_step":
            m = (t >= f.t_start) & (t <= f.t_end)
            gyro[m] += f.magnitude
        else:
            raise ValueError(f"unknown fault kind: {f.kind}")
    return replace(data, gnss=gnss, gnss_idx=gnss_idx, wheel=wheel, gyro=gyro)


SCENARIOS: dict[str, list[Fault]] = {
    "nominal": [],
    "gnss_dropout": [Fault("gnss_dropout", 40.0, 60.0)],                 # tunnel, 20 s
    "gnss_jump": [Fault("gnss_jump", 50.0, 56.0, 40.0)],                 # multipath, 40 m, 6 s
    "gnss_drift": [Fault("gnss_drift", 40.0, 80.0, 1.0)],                # slow ramp, 1 m/s
    "wheel_scale": [Fault("wheel_scale", 50.0, 70.0, 0.15)],             # slip/scale, +15 %
    "gyro_bias_step": [Fault("gyro_bias_step", 50.0, 120.0, 0.02)],      # IMU bias jump
}
