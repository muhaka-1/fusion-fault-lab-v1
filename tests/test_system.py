"""System tests: the real C processes on the simulated CAN bus. Every check is on what actually
crosses the bus.
  M11: cluster_core + display_term, driven by the Python mini bench
  M13: + the nav process, fed by the sensor player (simulated drive with faults)

Needs the C target built for this OS (cmake -S target -B target/build && cmake --build target/build)
and UDP multicast on loopback. Skipped otherwise, e.g. in Git Bash on Windows: run it in WSL.
"""
import os
import pathlib
import random
import signal
import socket
import struct
import subprocess
import time

import pytest

from fusionlab.busctl import MiniBench
from fusionlab.canbus import BusConfig

BUILD = pathlib.Path(__file__).resolve().parents[1] / "target" / "build"


def _multicast_ok() -> bool:
    try:
        port = 47000 + os.getpid() % 1000
        r = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        r.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        r.bind(("", port))
        r.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                     struct.pack("4s4s", socket.inet_aton("239.255.42.42"), socket.inet_aton("127.0.0.1")))
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton("127.0.0.1"))
        s.sendto(b"x", ("239.255.42.42", port))
        r.settimeout(1.0)
        return r.recv(4) == b"x"
    except OSError:
        return False


def _runnable(exe: pathlib.Path) -> bool:
    try:
        subprocess.run([str(exe), "--help"], capture_output=True, timeout=0.5)
    except subprocess.TimeoutExpired:
        return True                              # it started (and waited on the bus): fine
    except OSError:
        return False                             # missing, or built for another OS
    return True


def wait_for(pred, timeout, step=0.01):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return time.monotonic() - t0
        time.sleep(step)
    return None


@pytest.fixture
def rig(tmp_path):
    core, disp = BUILD / "cluster_core", BUILD / "display_term"
    if not core.exists() or not _runnable(core):
        pytest.skip("cluster_core not built for this OS (build the C target in WSL / Linux)")
    if not _multicast_ok():
        pytest.skip("UDP multicast on loopback not available here")
    port = 44000 + random.randrange(2000)        # own bus, so parallel runs don't mix
    env = {**os.environ, "CANBUS_PORT": str(port)}
    log = open(tmp_path / "display.log", "w")
    procs = {"core": subprocess.Popen([str(core)], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
             "display": subprocess.Popen([str(disp), "--plain"], env=env, stdout=log, stderr=subprocess.DEVNULL)}
    bench = MiniBench(BusConfig(port=port)).start()
    a = lambda: bench.cluster()["a"] or {}
    b = lambda: bench.cluster()["b"] or {}
    assert wait_for(lambda: bench.cluster()["alive"], 3.0) is not None, "cluster_core is not answering"
    yield bench, procs, a, b, tmp_path / "display.log"
    bench.stop()
    for p in procs.values():
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
    for p in procs.values():
        try:
            p.wait(timeout=2)
        except subprocess.TimeoutExpired:
            p.kill()
    log.close()


def test_gauges_and_positioning_come_up(rig):
    bench, _, a, b, _ = rig
    assert wait_for(lambda: a().get("speed_kph") == 50.0 and a().get("rpm") == 1500, 1.0) is not None
    assert wait_for(lambda: b().get("pos_text") == "OK" and b().get("heading_deg") == 90.0, 1.0) is not None


def test_indicator_blinks_at_90_per_minute_across_processes(rig):
    bench, _, a, _, _ = rig
    bench.set(stalk="left")
    assert wait_for(lambda: a().get("left"), 0.3) is not None
    rises, last, t0 = [], True, time.monotonic()
    while time.monotonic() - t0 < 2.5:
        cur = a().get("left")
        if cur and not last:
            rises.append(time.monotonic())
        last = cur
        time.sleep(0.005)
    periods = [y - x for x, y in zip(rises, rises[1:])]
    assert len(rises) >= 3 and all(abs(p - 0.667) < 0.06 for p in periods), periods
    bench.set(stalk="off")
    assert wait_for(lambda: not a().get("left"), 0.8) is not None


@pytest.mark.parametrize("mode", ["off", "corrupt", "freeze"])
def test_ecu_faults_show_dashes_then_recover(rig, mode):
    bench, _, a, _, _ = rig
    assert wait_for(lambda: a().get("speed_kph") is not None, 1.0) is not None
    bench.set(ecu=mode)
    dt = wait_for(lambda: a().get("ecu_fault") and a().get("speed_kph") is None and a().get("rpm") is None, 1.0)
    assert dt is not None and dt < 0.4, dt
    bench.set(ecu="ok")
    assert wait_for(lambda: not a().get("ecu_fault") and a().get("speed_kph") is not None, 1.0) is not None


def test_input_board_loss(rig):
    bench, _, a, _, _ = rig
    bench.set(dip=True)
    assert wait_for(lambda: a().get("dip"), 0.5) is not None
    bench.set(inputs_on=False)
    assert wait_for(lambda: a().get("input_fault") and not a().get("dip"), 0.4) is not None
    bench.set(inputs_on=True)
    assert wait_for(lambda: not a().get("input_fault") and a().get("dip"), 1.0) is not None


def test_nav_loss_and_core_crash(rig):
    bench, procs, a, b, log = rig
    assert wait_for(lambda: b().get("pos_text") == "OK", 1.0) is not None
    bench.set(nav="off")
    dt = wait_for(lambda: b().get("pos_text") == "NAV DATA LOST", 1.0)
    assert dt is not None and dt < 0.5 and a().get("pos_red") and b().get("heading_deg") is None
    procs["core"].kill()                          # the decider dies
    assert wait_for(lambda: not bench.cluster()["alive"], 1.0) is not None
    time.sleep(0.4)
    lines = log.read_text().splitlines()
    assert lines and lines[-1].endswith("NO SIGNAL FROM CORE")       # the display blanks itself
    assert any("pos=NAV DATA LOST heading=--" in line for line in lines)


# ---- M13: the nav process in the loop ---------------------------------------------------------
@pytest.fixture
def stack(tmp_path):
    """nav + cluster_core + display_term. The mini bench plays ECU and inputs only (no NAV_STATE)."""
    exes = {n: BUILD / n for n in ("nav", "cluster_core", "display_term")}
    if not all(e.exists() for e in exes.values()) or not _runnable(exes["nav"]):
        pytest.skip("nav / cluster_core not built for this OS (build the C target in WSL / Linux)")
    if not _multicast_ok():
        pytest.skip("UDP multicast on loopback not available here")
    port = 46000 + random.randrange(2000)
    cfg = BusConfig(port=port)
    env = {**os.environ, "CANBUS_PORT": str(port)}
    procs = {n: subprocess.Popen([str(e)] + (["--plain"] if n == "display_term" else []), env=env,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) for n, e in exes.items()}
    bench = MiniBench(cfg)
    bench.set(nav="off")
    bench.start()
    yield cfg, bench, procs
    bench.stop()
    for p in procs.values():
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
    for p in procs.values():
        try:
            p.wait(timeout=2)
        except subprocess.TimeoutExpired:
            p.kill()


def test_nav_node_aligns_and_tracks(stack):
    from fusionlab.sensorplay import SensorPlayer
    cfg, bench, _ = stack
    p = SensorPlayer("nominal", seed=2, speed=4, bus_cfg=cfg)
    p.run(duration=20.0)
    e = p.errors()
    assert e, "no aligned estimate from nav"
    assert e[0][0] < 3.0                                       # aligned within 3 s of driving
    late = [r[1] for r in e if r[0] > 5.0]
    assert max(late) < 2.0 and all(r[2] == "NOMINAL" for r in e if r[0] > 5.0)
    c = bench.cluster()                                        # and the cluster shows it
    assert c["b"]["pos_text"] == "OK" and c["b"]["heading_deg"] is not None


def test_gnss_jump_rejected_end_to_end(stack):
    """A 40 m GNSS jump: nav rejects it, the cluster goes amber, the position stays right."""
    from fusionlab.sensorplay import SensorPlayer
    cfg, bench, _ = stack
    p = SensorPlayer("gnss_jump", seed=2, speed=5, bus_cfg=cfg)
    amber = []

    def watch(player, i):
        c = bench.cluster()
        if c["a"] and 50.0 <= player.data.t[i] <= 57.0:
            amber.append(c["a"]["pos_amber"])

    p.run(duration=62.0, every_second=watch)
    during = [r for r in p.errors() if 50.0 <= r[0] <= 56.0]
    assert during and max(r[1] for r in during) < 2.0          # the 40 m jump did not pull the estimate
    assert any("gnss_rejecting" in r[4] for r in during)
    assert any(amber), "cluster never showed the amber POSITIONING lamp"


def test_imu_loss_makes_nav_unavailable(stack):
    from fusionlab.sensorplay import SensorPlayer
    cfg, bench, _ = stack
    p = SensorPlayer("nominal", seed=2, speed=2, bus_cfg=cfg, imu_gap=(8.0, 9.0))
    p.run(duration=12.0)
    flags = [fl for _, _, _, st, _, fl in p.log if st == "UNAVAILABLE"]
    assert any("imu_lost" in f for f in flags)
    after = [st for step, _, _, st, _, _ in p.log if p.data.t[step] > 9.0]
    assert "ALIGNING" in after                                 # 100 lost frames: time unknown, align again
    assert after[-1] == "NOMINAL"                              # and back to normal
