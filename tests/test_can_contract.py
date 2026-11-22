"""M8: the Python bench and the C nodes must agree byte for byte on every CAN message.

tests/can_vectors.txt is the output of target/tools/can_vectors.c (committed). These tests run
without a C compiler. If you have built the C target, test_c_still_prints_the_vectors also runs
the program and checks that its output has not changed.
"""
import pathlib
import subprocess

import pytest

from fusionlab import canbus as cb

ROOT = pathlib.Path(__file__).resolve().parents[1]
VEC = ROOT / "tests" / "can_vectors.txt"


def _vectors():
    rows = {}
    for line in VEC.read_text().splitlines():
        name, *rest = line.split()
        rows[name] = rest
    return rows


def _py(can_id, payload, ctr):
    p = cb.E2ETx(ctr).protect(can_id, bytearray(payload))
    return [f"{can_id:03x}"] + [f"{b:02x}" for b in p]


def _frame(v, name):
    return bytes(int(b, 16) for b in v[name][1:])


def test_encoders_match_c():
    v = _vectors()
    assert v["imu"] == _py(cb.IMU, cb.enc_imu(-0.0123456, 1.23456), 3)
    assert v["wheel"] == _py(cb.WHEEL, cb.enc_wheel(12.345), 14)
    assert v["gnss"] == _py(cb.GNSS, cb.enc_gnss(True, -123.456, 4567.891), 0)
    assert v["gnss_nofix"] == _py(cb.GNSS, cb.enc_gnss(False), 1)
    assert v["engine"] == _py(cb.ENGINE, cb.enc_engine(3456, 123.45, True, True), 7)
    assert v["inputs"] == _py(cb.INPUTS, cb.enc_inputs(cb.IN_STALK_LEFT | cb.IN_DIP_SWITCH | cb.IN_FLASH), 9)
    assert v["nav_cmd"] == _py(cb.NAV_CMD, cb.enc_nav_cmd(True, False, 50, 0.999), 4)


def test_decoders_match_c():
    v = _vectors()
    st = cb.dec_nav_state(_frame(v, "nav_state"))
    assert st["status"] == "DEGRADED" and set(st["flag_names"]) == {"gnss_alarm", "recovery"}
    assert abs(st["heading_deg"] - 271.5) < 1e-9 and abs(st["pos_std_m"] - 1.25) < 1e-9
    assert abs(st["gnss_age_s"] - 2.4) < 1e-9
    assert cb.dec_nav_pose(_frame(v, "nav_pose")) == {"x": 1000.5, "y": -20.25}
    d = cb.dec_nav_diag(_frame(v, "nav_diag"))
    assert abs(d["speed"] - 11.5) < 1e-9 and abs(d["bias"] - 0.00412) < 1e-9 and d["gnss_rejects"] == 37
    a = cb.dec_cluster_a(_frame(v, "cluster_a"))
    assert a["speed_kph"] == 123.4 and a["rpm"] is None            # 0xFFFF = "--" on the display
    assert a["left"] and a["pos_amber"] and a["hazard"] and not a["right"]
    assert cb.dec_cluster_b(_frame(v, "cluster_b")) == {"heading_deg": 271.5, "pos_text": "DEGRADED",
                                                         "pos_std_m": 1.25}


def test_every_c_frame_passes_the_python_crc():
    v = _vectors()
    for name, row in v.items():
        assert cb.E2ERx().check(int(row[0], 16), _frame(v, name)) == "INITIAL", name


def test_python_e2e_follows_the_c_rules():
    assert cb.crc8(b"123456789") ^ 0xFF == 0x4B          # same catalogue check value as test_e2e.c
    tx, rx = cb.E2ETx(), cb.E2ERx(max_delta=2)
    p = tx.protect(cb.ENGINE, cb.enc_engine(1000, 50, False, True))
    assert rx.check(cb.ENGINE, p) == "INITIAL" and rx.check(cb.ENGINE, p) == "REPEATED"
    tx.protect(cb.ENGINE, bytearray(8))                   # this frame is "lost"
    p = tx.protect(cb.ENGINE, cb.enc_engine(1000, 50, False, True))
    assert rx.check(cb.ENGINE, p) == "OK_SOME_LOST"
    p[3] ^= 1
    assert rx.check(cb.ENGINE, p) == "BAD_CRC"


def test_rounding_matches_c_lround():
    # Python's round() rounds halves to even: round(12.5) == 12. C's lround() gives 13.
    assert round(12.5) == 12 and cb._lround(12.5) == 13 and cb._lround(-12.5) == -13
    assert cb.dec_imu(cb.enc_imu(100.0, -1000.0)) == {"gyro": 8.388607, "acc": -83.88608}  # saturates


def test_c_still_prints_the_vectors():
    exe = [p for p in (ROOT / "target" / "build" / "can_vectors", ROOT / "target" / "build" / "can_vectors.exe")
           if p.exists()]
    if not exe:
        pytest.skip("C target not built (cmake -S target -B target/build && cmake --build target/build)")
    try:
        out = subprocess.run([str(exe[0])], capture_output=True, text=True, check=True).stdout
    except OSError:
        pytest.skip("can_vectors was built for another OS (e.g. in WSL); run this test there")
    assert out.splitlines() == VEC.read_text().splitlines()
