"""M5: fault monitor, windowed NIS test + availability / integrity status.

Instantaneous gating (in the EKF) isolates single bad measurements. The windowed
test sums NIS over the last N updates of a sensor; a sum far above its chi-square
expectation reveals persistent inconsistency (bias, drift) that single-sample
gating can miss. Status gives the graceful-degradation view: is the position
being anchored by GNSS right now, or is the filter dead-reckoning?
"""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass, field
import numpy as np
from .stats import chi2_threshold

NOMINAL, DEGRADED, UNAVAILABLE = "NOMINAL", "DEGRADED", "UNAVAILABLE"


@dataclass
class MonitorConfig:
    window: int = 10
    alarm_prob: float = 0.999
    gnss_timeout_s: float = 1.0       # no accepted GNSS for this long -> DEGRADED
    pos_std_limit_m: float = 5.0      # position std above this -> UNAVAILABLE


@dataclass
class Monitor:
    cfg: MonitorConfig = field(default_factory=MonitorConfig)

    def __post_init__(self):
        self.win = {"gnss": deque(maxlen=self.cfg.window), "wheel": deque(maxlen=self.cfg.window)}
        self.dof = {"gnss": 2, "wheel": 1}
        self.last_gnss_ok = 0.0
        self.alarm = {"gnss": False, "wheel": False}

    def on_update(self, sensor: str, t: float, nis: float, accepted: bool) -> None:
        self.win[sensor].append(nis)
        if sensor == "gnss" and accepted:
            self.last_gnss_ok = t
        w = self.win[sensor]
        if len(w) == w.maxlen:
            thr = chi2_threshold(self.cfg.alarm_prob, self.dof[sensor] * len(w))
            self.alarm[sensor] = bool(sum(w) > thr)

    def status(self, t: float, P: np.ndarray) -> str:
        pos_std = float(np.sqrt(P[0, 0] + P[1, 1]))
        if pos_std > self.cfg.pos_std_limit_m:
            return UNAVAILABLE
        if (t - self.last_gnss_ok) > self.cfg.gnss_timeout_s or any(self.alarm.values()):
            return DEGRADED
        return NOMINAL
