"""M11 system test: the real C processes (cluster_core, display_term) on the simulated CAN bus,
driven by the Python mini bench. Every check is on what actually crosses the bus.

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
