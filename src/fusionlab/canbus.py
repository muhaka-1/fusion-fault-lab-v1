"""Python side of the CAN message set (M8): E2E protection and message codecs.

Byte-for-byte mirror of target/include/e2e.h and target/include/can_msg.h. The bench (M14) is
written in Python and talks to the C nodes, so both sides must produce and read exactly the same
bytes. tests/test_can_contract.py checks this against tests/can_vectors.txt, which is the output
of the C program target/tools/can_vectors.c.

Python gets the encoders for what the bench sends (sensors, ECU, inputs, nav commands, and
NAV_STATE until the C nav node exists in M13) and the decoders for what it reads (nav and cluster
output). M11 adds the transport: the same UDP multicast bus as target/src/common/canbus_udp.c.

Try it:  python -m fusionlab.canbus                      decode every reference frame
         python -m fusionlab.canbus 0x200 d4378...        decode one frame (id, 16 hex digits)
"""
from __future__ import annotations
import math
import os
import pathlib
import socket
import struct
import sys
from dataclasses import dataclass, field

# ---- identifiers -------------------------------------------------------------------
IMU, WHEEL, GNSS = 0x100, 0x110, 0x120
ENGINE, INPUTS = 0x200, 0x210
NAV_STATE, NAV_POSE, NAV_DIAG, NAV_CMD = 0x300, 0x301, 0x302, 0x310
CLUSTER_A, CLUSTER_B = 0x400, 0x401
NAMES = {IMU: "IMU", WHEEL: "WHEEL", GNSS: "GNSS", ENGINE: "ENGINE", INPUTS: "INPUTS",
         NAV_STATE: "NAV_STATE", NAV_POSE: "NAV_POSE", NAV_DIAG: "NAV_DIAG", NAV_CMD: "NAV_CMD",
         CLUSTER_A: "CLUSTER_A", CLUSTER_B: "CLUSTER_B"}
PERIOD_MS = {IMU: 10, WHEEL: 20, GNSS: 200, ENGINE: 20, INPUTS: 10, NAV_STATE: 100, NAV_POSE: 100,
             NAV_DIAG: 100, CLUSTER_A: 50, CLUSTER_B: 50}
INVALID16 = 0xFFFF

IN_STALK_LEFT, IN_STALK_RIGHT, IN_HAZARD_BTN, IN_DIP_SWITCH, IN_MAIN_BEAM, IN_FLASH = (1 << i for i in range(6))
LAMP_LEFT, LAMP_RIGHT, LAMP_DIP, LAMP_MAIN, LAMP_CHECK_ENGINE, LAMP_ECU_FAULT, LAMP_POS_AMBER, LAMP_POS_RED = (1 << i for i in range(8))
LAMP2_INPUT_FAULT, LAMP2_HAZARD = 1, 2
NAV_STATUS = ["NOMINAL", "DEGRADED", "UNAVAILABLE", "ALIGNING"]
POS_TEXT = ["OK", "DEGRADED", "UNAVAILABLE", "ALIGNING", "NAV DATA LOST"]
NAVF = {"gnss_alarm": 1, "wheel_alarm": 2, "gnss_rejecting": 4, "imu_lost": 8, "recovery": 16, "input_e2e_err": 32}

# ---- E2E -----------------------------------------------------------------------------
COUNTER_MOD = 15


def crc8(data: bytes, crc: int = 0xFF) -> int:
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crc8_frame(can_id: int, payload: bytes | bytearray) -> int:
    return crc8(bytes([can_id & 0xFF, can_id >> 8]) + bytes(payload[1:8])) ^ 0xFF


class E2ETx:
    def __init__(self, counter: int = 0):
        self.counter = counter

    def protect(self, can_id: int, payload: bytearray) -> bytearray:
        payload[1] = (payload[1] & 0xF0) | (self.counter & 0x0F)
        payload[0] = crc8_frame(can_id, payload)
        self.counter = (self.counter + 1) % COUNTER_MOD
        return payload


class E2ERx:
    """Same classification as e2e_check() in C."""

    def __init__(self, max_delta: int = 2, sync_needed: int = 0):
        self.max_delta, self.sync_needed = max_delta, sync_needed
        self.last = None
        self.sync_left = sync_needed

    def check(self, can_id: int, p: bytes) -> str:
        if p[0] != crc8_frame(can_id, p):
            return "BAD_CRC"
        c = p[1] & 0x0F
        if c >= COUNTER_MOD:
            return "BAD_COUNTER"
        if self.last is None:
            self.last, self.sync_left = c, self.sync_needed
            return "INITIAL"
        d = (c - self.last) % COUNTER_MOD
        if d == 0:
            return "REPEATED"
        self.last = c
        if d > self.max_delta:
            self.sync_left = self.sync_needed
            return "WRONG_SEQUENCE"
        if self.sync_left:
            self.sync_left -= 1
            return "INITIAL"
        return "OK" if d == 1 else "OK_SOME_LOST"


def usable(status: str) -> bool:
    return status in ("OK", "OK_SOME_LOST")


# ---- codecs ------------------------------------------------------------------------
def _lround(v: float) -> int:          # C lround(): halves away from zero
    return int(math.copysign(math.floor(abs(v) + 0.5), v))


def _sat(v: float, lo: float, hi: float) -> int:
    if v != v:
        return 0
    return _lround(min(max(v, lo), hi))


I24 = (-8388608, 8388607)


def _begin(flags: int = 0) -> bytearray:
    p = bytearray(8)
    p[1] = (flags << 4) & 0xF0
    return p


def _i24(p, o, v):
    p[o:o + 3] = (v & 0xFFFFFF).to_bytes(3, "little")


def _gi24(p, o):
    u = int.from_bytes(p[o:o + 3], "little")
    return u - (1 << 24) if u & 0x800000 else u


def _u16(p, o, v):
    p[o:o + 2] = int(v).to_bytes(2, "little")


def _gu16(p, o):
    return int.from_bytes(p[o:o + 2], "little")


def enc_imu(gyro: float, acc: float) -> bytearray:
    p = _begin()
    _i24(p, 2, _sat(gyro * 1e6, *I24)); _i24(p, 5, _sat(acc * 1e5, *I24))
    return p


def dec_imu(p):
    return {"gyro": _gi24(p, 2) * 1e-6, "acc": _gi24(p, 5) * 1e-5}


def enc_wheel(speed: float) -> bytearray:
    p = _begin(); _u16(p, 2, _sat(speed * 100, 0, 65534))
    return p


def dec_wheel(p):
    return {"speed": _gu16(p, 2) * 0.01}


def enc_gnss(fix: bool, x: float = 0.0, y: float = 0.0) -> bytearray:
    p = _begin(1 if fix else 0)
    if fix:
        _i24(p, 2, _sat(x * 100, *I24)); _i24(p, 5, _sat(y * 100, *I24))
    return p


def dec_gnss(p):
    return {"fix": bool((p[1] >> 4) & 1), "x": _gi24(p, 2) * 0.01, "y": _gi24(p, 5) * 0.01}


def enc_engine(rpm: float, speed_kph: float, mil: bool, running: bool) -> bytearray:
    p = _begin((1 if mil else 0) | (2 if running else 0))
    _u16(p, 2, _sat(rpm, 0, 65534)); _u16(p, 4, _sat(speed_kph * 100, 0, 65534))
    return p


def dec_engine(p):
    f = p[1] >> 4
    return {"mil": bool(f & 1), "running": bool(f & 2), "rpm": _gu16(p, 2), "speed_kph": _gu16(p, 4) * 0.01}


def enc_inputs(bits: int) -> bytearray:
    p = _begin(); p[2] = bits & 0x3F
    return p


def dec_inputs(p):
    return {"bits": p[2] & 0x3F}


def enc_nav_cmd(gating: bool, realign: bool, lockout_rejects: int, gate_prob: float) -> bytearray:
    p = _begin((1 if gating else 0) | (2 if realign else 0))
    _u16(p, 2, max(0, min(int(lockout_rejects), 65535))); _u16(p, 4, _sat((1 - gate_prob) * 1e6, 1, 65534))
    return p


def enc_nav_state(status: int, flags: int, heading_deg: float, pos_std_m: float, gnss_age_s: float) -> bytearray:
    """Same as msg_encode_nav_state() in C. status: 0 NOMINAL, 1 DEGRADED, 2 UNAVAILABLE, 3 ALIGNING."""
    p = _begin(status & 3)
    p[2] = flags & 0xFF
    h = math.fmod(heading_deg, 360.0)
    if h < 0:
        h += 360.0
    hc = _sat(h * 100.0, 0, 36000)
    _u16(p, 3, 0 if hc >= 36000 else hc)
    _u16(p, 5, _sat(pos_std_m * 100.0, 0, 65534))
    p[7] = _sat(gnss_age_s * 10.0, 0, 255)
    return p


def dec_nav_state(p):
    return {"status": NAV_STATUS[(p[1] >> 4) & 3], "flags": p[2],
            "flag_names": [k for k, v in NAVF.items() if p[2] & v],
            "heading_deg": _gu16(p, 3) * 0.01, "pos_std_m": _gu16(p, 5) * 0.01, "gnss_age_s": p[7] * 0.1}


def dec_nav_pose(p):
    return {"x": _gi24(p, 2) * 0.01, "y": _gi24(p, 5) * 0.01}


def dec_nav_diag(p):
    b = _gu16(p, 4)
    return {"speed": _gu16(p, 2) * 0.01, "bias": (b - 65536 if b & 0x8000 else b) * 1e-5,
            "gnss_rejects": p[6], "recoveries": p[7]}


def dec_cluster_a(p):
    s, r, l1, l2 = _gu16(p, 2), _gu16(p, 4), p[6], p[7]
    return {"speed_kph": None if s == INVALID16 else s / 10, "rpm": None if r == INVALID16 else r,
            "left": bool(l1 & LAMP_LEFT), "right": bool(l1 & LAMP_RIGHT), "dip": bool(l1 & LAMP_DIP),
            "main": bool(l1 & LAMP_MAIN), "check_engine": bool(l1 & LAMP_CHECK_ENGINE),
            "ecu_fault": bool(l1 & LAMP_ECU_FAULT), "pos_amber": bool(l1 & LAMP_POS_AMBER),
            "pos_red": bool(l1 & LAMP_POS_RED), "input_fault": bool(l2 & LAMP2_INPUT_FAULT),
            "hazard": bool(l2 & LAMP2_HAZARD)}


def dec_cluster_b(p):
    h, sd, txt = _gu16(p, 2), _gu16(p, 5), p[4]
    return {"heading_deg": None if h == INVALID16 else h / 100, "pos_text": POS_TEXT[txt] if txt < len(POS_TEXT) else "?",
            "pos_std_m": None if sd == INVALID16 else sd / 100}


DECODERS = {IMU: dec_imu, WHEEL: dec_wheel, GNSS: dec_gnss, ENGINE: dec_engine, INPUTS: dec_inputs,
            NAV_STATE: dec_nav_state, NAV_POSE: dec_nav_pose, NAV_DIAG: dec_nav_diag,
            CLUSTER_A: dec_cluster_a, CLUSTER_B: dec_cluster_b}


ENCODERS = {IMU: enc_imu, WHEEL: enc_wheel, GNSS: enc_gnss, ENGINE: enc_engine, INPUTS: enc_inputs,
            NAV_CMD: enc_nav_cmd, NAV_STATE: enc_nav_state}


# ---- transport -----------------------------------------------------------------------
WIRE = struct.Struct("<IB3x8s")


def pack(can_id: int, payload: bytes) -> bytes:
    return WIRE.pack(can_id, 8, bytes(payload))


def unpack(datagram: bytes):
    if len(datagram) != WIRE.size:
        return None
    can_id, dlc, data = WIRE.unpack(datagram)
    if dlc > 8 or can_id > 0x7FF:
        return None
    return can_id, data


@dataclass
class BusConfig:
    """Same environment variables as the C nodes; read when the config is created."""
    group: str = field(default_factory=lambda: os.environ.get("CANBUS_GROUP", "239.255.42.42"))
    port: int = field(default_factory=lambda: int(os.environ.get("CANBUS_PORT", "42424")))
    iface: str = field(default_factory=lambda: os.environ.get("CANBUS_IF", "127.0.0.1"))


class Bus:
    """One UDP multicast socket for sending, one for receiving. Every node sees every frame.
    Linux / WSL / macOS. (Windows and WSL do not share a loopback: run the whole bus on one side.)"""

    def __init__(self, cfg: BusConfig | None = None):
        self.cfg = cfg or BusConfig()
        c = self.cfg
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        self.tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self.tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(c.iface))
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except OSError:
                pass
        self.rx.bind(("", c.port))
        self.rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                           struct.pack("4s4s", socket.inet_aton(c.group), socket.inet_aton(c.iface)))
        self.rx.settimeout(0.1)
        self._dst = (c.group, c.port)

    def send(self, can_id: int, payload: bytes) -> None:
        self.tx.sendto(pack(can_id, payload), self._dst)

    def recv(self):
        """Returns (id, data) or None on timeout / malformed datagram."""
        try:
            d = self.rx.recv(64)
        except (socket.timeout, OSError):
            return None
        return unpack(d)

    def close(self) -> None:
        for s in (self.tx, self.rx):
            try:
                s.close()
            except OSError:
                pass


def _show(can_id: int, p: bytes) -> None:
    rx = E2ERx()
    e2e = rx.check(can_id, p)
    dec = DECODERS.get(can_id)
    if dec:
        body = {k: round(v, 6) if isinstance(v, float) else v for k, v in dec(p).items()}
    elif can_id in NAMES:
        body = "(no Python decoder: the bench only sends this message)"
    else:
        body = "(unknown message id)"
    print(f"{NAMES.get(can_id, '?'):9s} 0x{can_id:03x}  {p.hex(' ')}  crc {e2e:7s} ctr {p[1] & 15:2d}  {body}")


def main(argv: list[str]) -> int:
    if len(argv) == 2:
        _show(int(argv[0], 16), bytes.fromhex(argv[1]))
        return 0
    vec = pathlib.Path(__file__).resolve().parents[2] / "tests" / "can_vectors.txt"
    for line in vec.read_text().splitlines():
        name, cid, *hx = line.split()
        _show(int(cid, 16), bytes(int(b, 16) for b in hx))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
