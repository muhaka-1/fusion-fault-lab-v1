"""M13: play a simulated drive onto the CAN bus and watch the C nav node fuse it.

    python3 -m fusionlab.sensorplay --scenario gnss_jump          # real time, 120 s
    python3 -m fusionlab.sensorplay --scenario gyro_bias_step --speed 4 --plot

The player sends what the car's sensors would send: IMU every 10 ms, wheel speed every 20 ms,
GNSS every 200 ms (a "no fix" frame while GNSS is out). The faults are the ones from M4. It also
reads NAV_STATE, NAV_POSE and NAV_DIAG back and compares the estimate with the truth, which only
the simulator knows. Start `nav` (and `cluster_core`, `display_term` if you like) first.

--speed 4 plays four times faster than real time. That works because the nav node counts IMU
frames as its clock, not the wall clock.
"""
from __future__ import annotations

import argparse
import pathlib
import threading
import time

import numpy as np

from . import canbus as cb
from .faults import SCENARIOS, apply_faults
from .sim import SimConfig, simulate

STATUS = ["NOMINAL", "DEGRADED", "UNAVAILABLE", "ALIGNING"]


class SensorPlayer:
    def __init__(self, scenario: str = "nominal", seed: int = 1, speed: float = 1.0,
                 bus_cfg: cb.BusConfig | None = None, imu_gap: tuple[float, float] | None = None,
                 corrupt_imu: tuple[float, float] | None = None):
        clean = simulate(SimConfig(seed=seed))
        self.data = apply_faults(clean, SCENARIOS[scenario])
        self.gnss_slots = {int(i) for i in clean.gnss_idx}         # where a receiver would report
        self.gnss_at = {int(i): k for k, i in enumerate(self.data.gnss_idx)}
        self.wheel_at = {int(i): k for k, i in enumerate(self.data.wheel_idx)}
        self.seed, self.speed, self.imu_gap, self.corrupt_imu = seed, speed, imu_gap, corrupt_imu
        self.bus = cb.Bus(bus_cfg)
        self.tx = {i: cb.E2ETx() for i in (cb.IMU, cb.WHEEL, cb.GNSS)}
        self.rx = {i: cb.E2ERx(max_delta=3) for i in (cb.NAV_STATE, cb.NAV_POSE, cb.NAV_DIAG)}
        self.lock = threading.Lock()
        self.sent = -1                     # index of the last IMU sample sent
        self.state = self.diag = None
        self.log: list[tuple] = []         # (step, x, y, status, pos_std, flags)
        self._stop = threading.Event()

    def _send(self, can_id, payload, corrupt=False):
        self.tx[can_id].protect(can_id, payload)
        if corrupt:
            payload[3] ^= 0x01
        self.bus.send(can_id, payload)

    def _recv_loop(self):
        while not self._stop.is_set():
            r = self.bus.recv()
            if r is None or r[0] not in self.rx:
                continue
            can_id, p = r
            if self.rx[can_id].check(can_id, p) not in ("OK", "OK_SOME_LOST", "INITIAL"):
                continue                       # check first, decode only trusted frames
            with self.lock:
                if can_id == cb.NAV_STATE:
                    self.state = cb.dec_nav_state(p)
                elif can_id == cb.NAV_DIAG:
                    self.diag = cb.dec_nav_diag(p)
                elif can_id == cb.NAV_POSE and self.state is not None:
                    # the node publishes after every 10th IMU step: match the pose to that step
                    step = (max(self.sent, 0) // 10) * 10
                    pose = cb.dec_nav_pose(p)
                    self.log.append((step, pose["x"], pose["y"], self.state["status"], self.state["pos_std_m"],
                                     tuple(self.state["flag_names"])))

    def run(self, duration: float | None = None, every_second=None) -> None:
        d = self.data
        n = len(d.t) if duration is None else min(len(d.t), int(round(duration / d.cfg.dt)) + 1)
        rx = threading.Thread(target=self._recv_loop, daemon=True)
        rx.start()
        t0 = time.monotonic()
        try:
            for i in range(n):
                target = t0 + d.t[i] / self.speed
                delay = target - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                ti = d.t[i]
                with self.lock:
                    self.sent = i                  # before sending: the answer can come back at once
                if not (self.imu_gap and self.imu_gap[0] <= ti < self.imu_gap[1]):
                    bad = bool(self.corrupt_imu and self.corrupt_imu[0] <= ti < self.corrupt_imu[1])
                    self._send(cb.IMU, cb.enc_imu(float(d.gyro[i]), float(d.acc[i])), corrupt=bad)
                if i in self.wheel_at:
                    self._send(cb.WHEEL, cb.enc_wheel(float(d.wheel[self.wheel_at[i]])))
                if i in self.gnss_at:
                    g = d.gnss[self.gnss_at[i]]
                    self._send(cb.GNSS, cb.enc_gnss(True, float(g[0]), float(g[1])))
                elif i in self.gnss_slots:
                    self._send(cb.GNSS, cb.enc_gnss(False))            # receiver: "no fix"
                if every_second and i % int(round(1.0 / d.cfg.dt)) == 0 and i > 0:
                    every_second(self, i)
            time.sleep(0.03)                       # the last answer; less than nav's 100 ms IMU timeout
        finally:
            self._stop.set()
            rx.join(timeout=1.0)
            self.bus.close()

    # ---- results ------------------------------------------------------------------------
    def errors(self):
        """(t, error, status, pos_std, flags) for every pose received while aligned."""
        out = []
        tr = self.data.truth
        for step, x, y, st, sd, fl in self.log:
            if st == "ALIGNING" or step >= len(tr):
                continue
            out.append((self.data.t[step], float(np.hypot(x - tr[step, 0], y - tr[step, 1])), st, sd, fl))
        return out


def _print_second(p: SensorPlayer, i: int) -> None:
    with p.lock:
        st, dg = p.state, p.diag
        last = p.log[-1] if p.log else None
    if st is None:
        print(f"t={p.data.t[i]:6.1f} s  (no NAV_STATE yet: is `nav` running?)")
        return
    err = "--"
    if last and last[3] != "ALIGNING" and last[0] < len(p.data.truth):
        tr = p.data.truth[last[0]]
        err = f"{np.hypot(last[1] - tr[0], last[2] - tr[1]):5.2f} m"
    rej = f"rejects {dg['gnss_rejects']:3d}  recoveries {dg['recoveries']}" if dg else ""
    print(f"t={p.data.t[i]:6.1f} s  {st['status']:11s}  error {err:>7s}  pos_std {st['pos_std_m']:6.2f} m  "
          f"heading {st['heading_deg']:6.1f}  {rej}  flags {','.join(st['flag_names']) or '-'}", flush=True)


def _plot(p: SensorPlayer, scenario: str, path: pathlib.Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from .run import run_scenario

    e = p.errors()
    t = np.array([r[0] for r in e])
    err = np.array([r[1] for r in e])
    ref = run_scenario(scenario, p.seed)
    fig, ax = plt.subplots(2, 1, figsize=(11, 7.5), gridspec_kw={"height_ratios": [1.2, 1]})
    est = {s: [(r[1], r[2]) for r in p.log if r[3] == s] for s in ("NOMINAL", "DEGRADED", "UNAVAILABLE")}
    ax[0].plot(p.data.truth[:, 0], p.data.truth[:, 1], "k-", lw=1, label="truth")
    for s, c in (("NOMINAL", "tab:green"), ("DEGRADED", "tab:orange"), ("UNAVAILABLE", "tab:red")):
        if est[s]:
            a = np.array(est[s])
            ax[0].plot(a[:, 0], a[:, 1], ".", ms=3, color=c, label=f"C nav on the bus: {s}")
    ax[0].set(aspect="equal", xlabel="x [m]", ylabel="y [m]", title=f"{scenario}: what the nav node sent")
    ax[0].legend(fontsize=8, loc="best")
    for f in SCENARIOS[scenario]:
        ax[1].axvspan(f.t_start, min(f.t_end, p.data.t[-1]), color="0.9")
    ax[1].plot(ref.t, ref.pos_err, color="tab:blue", lw=1, alpha=0.6, label="Python filter, offline (starts from truth)")
    ax[1].plot(t, err, color="tab:red", lw=1.2, label="C nav node on the bus (aligns from GNSS)")
    ax[1].set(yscale="log", xlabel="t [s]", ylabel="position error [m]",
              title="same sensor data: the bus adds alignment and real-time delivery, not new errors")
    ax[1].legend(fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python3 -m fusionlab.sensorplay", description=__doc__.split("\n\n")[0])
    ap.add_argument("--scenario", default="nominal", choices=list(SCENARIOS))
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed (4 = four times real time)")
    ap.add_argument("--duration", type=float, default=None, help="seconds of the drive to play (default: all 120)")
    ap.add_argument("--imu-gap", default=None, help="stop the IMU between two times, e.g. 30:31")
    ap.add_argument("--plot", action="store_true", help="write docs/m13_onbus.png at the end")
    a = ap.parse_args(argv)
    gap = tuple(float(x) for x in a.imu_gap.split(":")) if a.imu_gap else None
    p = SensorPlayer(a.scenario, a.seed, a.speed, imu_gap=gap)
    faults = SCENARIOS[a.scenario]
    print(f"playing {a.scenario} (seed {a.seed}) at {a.speed:g}x real time; faults: "
          + (", ".join(f"{f.kind} {f.t_start:g}-{f.t_end:g} s" for f in faults) or "none"))
    try:
        p.run(a.duration, _print_second)
    except KeyboardInterrupt:
        pass
    e = p.errors()
    if not e:
        print("no aligned estimate received: is `nav` running on the same bus?")
        return 1
    err = np.array([r[1] for r in e])
    counts = {s: sum(r[2] == s for r in e) for s in ("NOMINAL", "DEGRADED", "UNAVAILABLE")}
    print(f"\nreceived {len(e)} estimates. position error: RMSE {np.sqrt(np.mean(err ** 2)):.2f} m, "
          f"max {err.max():.2f} m. status: " + ", ".join(f"{k} {v / len(e):.0%}" for k, v in counts.items()))
    if a.plot:
        out = pathlib.Path(__file__).resolve().parents[2] / "docs" / "m13_onbus.png"
        _plot(p, a.scenario, out)
        print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
