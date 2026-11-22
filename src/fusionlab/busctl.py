"""M11: a small bench on the simulated CAN bus, and a bus monitor. Python standard library only.

    python3 -m fusionlab.busctl bench      play the input board, the engine ECU and the nav
                                           process; type commands; see what the cluster shows
    python3 -m fusionlab.busctl monitor    every message on the bus: rate, E2E result, content

Run it on the same machine (or the same WSL) as cluster_core and display_term. The full bench
with a vehicle simulator and sensor faults comes in M14; this one sends fixed values that you
change by hand.
"""
from __future__ import annotations

import argparse
import sys
import threading
import time

from . import canbus as cb

NAV_CODES = {"nominal": 0, "degraded": 1, "unavailable": 2, "aligning": 3}
HELP = """commands:
  left | right | off            stalk position
  hazard                        press the hazard button once (held 150 ms)
  dip on|off  main on|off       light switches
  flash                         flash the main beam (held 500 ms)
  speed KPH   rpm RPM           what the ECU sends
  mil on|off                    the ECU asks for CHECK ENGINE
  ecu ok|off|corrupt|freeze     how the ECU frames arrive
  nav nominal|degraded|unavailable|aligning|off
  inputs on|off                 the input board sends frames, or goes silent
  status                        print what the cluster shows now
  help | quit"""


class MiniBench:
    """Plays INPUTS (10 ms), ENGINE (20 ms) and NAV_STATE (100 ms), and reads CLUSTER_A/B."""

    def __init__(self, bus_cfg: cb.BusConfig | None = None):
        self.bus = cb.Bus(bus_cfg)
        self.lock = threading.Lock()
        self.stalk, self.dip, self.main = "off", False, False
        self.hazard_until = self.flash_until = 0.0
        self.speed_kph, self.rpm, self.mil = 50.0, 1500.0, False
        self.ecu, self.nav, self.inputs_on = "ok", "nominal", True
        self.tx = {cb.INPUTS: cb.E2ETx(), cb.ENGINE: cb.E2ETx(), cb.NAV_STATE: cb.E2ETx()}
        self.rx = {cb.CLUSTER_A: cb.E2ERx(), cb.CLUSTER_B: cb.E2ERx()}
        self.a: dict | None = None
        self.b: dict | None = None
        self.last_rx = 0.0
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

    # ---- commands -------------------------------------------------------------------
    def set(self, **kw) -> None:
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def press_hazard(self) -> None:
        self.set(hazard_until=time.monotonic() + 0.15)

    def flash(self, seconds: float = 0.5) -> None:
        self.set(flash_until=time.monotonic() + seconds)

    def cluster(self) -> dict:
        with self.lock:
            return {"alive": time.monotonic() - self.last_rx < 0.25, "a": self.a, "b": self.b}

    # ---- threads --------------------------------------------------------------------
    def start(self) -> "MiniBench":
        for fn in (self._send_loop, self._recv_loop):
            t = threading.Thread(target=fn, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self.bus.close()

    def _send(self, can_id: int, payload: bytearray, mode: str = "ok") -> None:
        tx = self.tx[can_id]
        ctr = tx.counter
        tx.protect(can_id, payload)
        if mode == "freeze":                     # ECU hung: the same counter again and again
            tx.counter = ctr
        if mode == "corrupt":
            payload[4] ^= 0x01                   # one bit flips on the wire
        self.bus.send(can_id, payload)

    def _send_loop(self) -> None:
        tick, nxt = 0, time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            with self.lock:
                bits = {"left": cb.IN_STALK_LEFT, "right": cb.IN_STALK_RIGHT}.get(self.stalk, 0)
                bits |= cb.IN_HAZARD_BTN if now < self.hazard_until else 0
                bits |= cb.IN_DIP_SWITCH if self.dip else 0
                bits |= cb.IN_MAIN_BEAM if self.main else 0
                bits |= cb.IN_FLASH if now < self.flash_until else 0
                inputs_on, ecu, nav = self.inputs_on, self.ecu, self.nav
                eng = cb.enc_engine(self.rpm, self.speed_kph, self.mil, True)
            if inputs_on:
                self._send(cb.INPUTS, cb.enc_inputs(bits))
            if tick % 2 == 0 and ecu != "off":
                self._send(cb.ENGINE, eng, ecu)
            if tick % 10 == 0 and nav != "off":
                self._send(cb.NAV_STATE, cb.enc_nav_state(NAV_CODES[nav], 0, 90.0, 0.8, 0.2))
            tick += 1
            nxt += 0.010
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.1:                   # far behind: restart the grid, no burst
                nxt = time.monotonic()

    def _recv_loop(self) -> None:
        while not self._stop.is_set():
            r = self.bus.recv()
            if r is None:
                continue
            can_id, data = r
            if can_id not in self.rx or not cb.usable(self.rx[can_id].check(can_id, data)):
                continue
            with self.lock:
                if can_id == cb.CLUSTER_A:
                    self.a = cb.dec_cluster_a(data)
                else:
                    self.b = cb.dec_cluster_b(data)
                self.last_rx = time.monotonic()


def cluster_line(c: dict, blinking: frozenset = frozenset()) -> str:
    """One line of what the cluster shows. Indicators blink, so the caller says which ones are
    active (lit at least once in the last second) instead of passing the lamp of this instant."""
    if not c["alive"] or not c["a"] or not c["b"]:
        return "NO SIGNAL FROM CORE (is cluster_core running?)"
    a, b = c["a"], c["b"]
    sp = "--" if a["speed_kph"] is None else f"{a['speed_kph']:.0f}"
    rpm = "--" if a["rpm"] is None else f"{a['rpm']}"
    hd = "--" if b["heading_deg"] is None else f"{b['heading_deg']:.0f}"
    lamps = [f"{n} blinking" for n in ("left", "right") if n in blinking]
    lamps += [n for n in ("hazard", "dip", "main", "check_engine") if a[n]]
    faults = [n for n, k in (("ECU DATA", "ecu_fault"), ("INPUTS", "input_fault")) if a[k]]
    pos = b["pos_text"] + (" (red)" if a["pos_red"] else " (amber)" if a["pos_amber"] else "")
    return (f"speed {sp} km/h  rpm {rpm}  lamps {', '.join(lamps) or '-'}  faults {', '.join(faults) or '-'}"
            f"  positioning {pos}  heading {hd}")


def run_bench() -> int:
    bench = MiniBench().start()
    print("mini bench on the CAN bus. 'help' lists the commands.")
    stop = threading.Event()

    lit_at = {"left": 0.0, "right": 0.0}

    def blinking() -> frozenset:
        now = time.monotonic()
        return frozenset(n for n, t in lit_at.items() if now - t < 1.0)

    def watch():                                 # print the cluster line whenever it changes
        last = ""
        while not stop.is_set():
            c = bench.cluster()
            for n in lit_at:
                if c["a"] and c["a"][n]:
                    lit_at[n] = time.monotonic()
            line = cluster_line(c, blinking())
            if line != last:
                print(f"\r  cluster: {line}\n> ", end="", flush=True)
                last = line
            time.sleep(0.02)

    threading.Thread(target=watch, daemon=True).start()
    try:
        while True:
            try:
                words = input("> ").split()
            except EOFError:
                break
            if not words:
                continue
            cmd, arg = words[0], (words[1] if len(words) > 1 else "")
            try:
                if cmd in ("left", "right", "off"):
                    bench.set(stalk=cmd)
                elif cmd == "hazard":
                    bench.press_hazard()
                elif cmd in ("dip", "main"):
                    bench.set(**{cmd: arg == "on"})
                elif cmd == "flash":
                    bench.flash()
                elif cmd == "speed":
                    bench.set(speed_kph=float(arg))
                elif cmd == "rpm":
                    bench.set(rpm=float(arg))
                elif cmd == "mil":
                    bench.set(mil=arg == "on")
                elif cmd == "ecu" and arg in ("ok", "off", "corrupt", "freeze"):
                    bench.set(ecu=arg)
                elif cmd == "nav" and (arg in NAV_CODES or arg == "off"):
                    bench.set(nav=arg)
                elif cmd == "inputs":
                    bench.set(inputs_on=arg != "off")
                elif cmd == "status":
                    print("  cluster:", cluster_line(bench.cluster(), blinking()))
                elif cmd in ("quit", "exit"):
                    break
                else:
                    print(HELP)
            except ValueError:
                print("  needs a number, e.g. 'speed 80'")
    finally:
        stop.set()
        bench.stop()
    return 0


def run_monitor(seconds: float, ids: set[int]) -> int:
    bus = cb.Bus()
    rx: dict[int, cb.E2ERx] = {}
    count: dict[int, int] = {}
    bad: dict[int, int] = {}
    last: dict[int, bytes] = {}
    t0 = t_print = time.monotonic()
    print("listening on the bus... (Ctrl+C to stop)")
    try:
        while seconds <= 0 or time.monotonic() - t0 < seconds:
            r = bus.recv()
            now = time.monotonic()
            if r is not None:
                can_id, data = r
                st = rx.setdefault(can_id, cb.E2ERx(max_delta=3)).check(can_id, data)
                count[can_id] = count.get(can_id, 0) + 1
                if st not in ("OK", "OK_SOME_LOST", "INITIAL"):
                    bad[can_id] = bad.get(can_id, 0) + 1
                last[can_id] = data
                if can_id in ids:
                    dec = cb.DECODERS.get(can_id)
                    print(f"{now - t0:8.3f}  {cb.NAMES.get(can_id, '?'):9s} {data.hex(' ')}  {st:14s} "
                          f"{dec(data) if dec else ''}")
            if not ids and now - t_print >= 1.0:
                dt = now - t_print
                print(f"\n--- {now - t0:5.1f} s ---  {'id':5s} {'name':9s} {'rate':>7s}  {'E2E bad':>7s}  last frame")
                for i in sorted(count):
                    print(f"            0x{i:03x} {cb.NAMES.get(i, '?'):9s} {count[i] / dt:5.0f}/s  "
                          f"{bad.get(i, 0):7d}  {last[i].hex(' ')}")
                count.clear()
                bad.clear()
                t_print = now
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python3 -m fusionlab.busctl", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("bench", help="play input board, ECU and nav; type commands")
    m = sub.add_parser("monitor", help="show the traffic on the bus")
    m.add_argument("--seconds", type=float, default=0, help="stop after N seconds (default: never)")
    m.add_argument("--ids", default="", help="print every frame of these ids, e.g. 0x400,0x401")
    a = ap.parse_args(argv)
    if a.cmd == "bench":
        return run_bench()
    return run_monitor(a.seconds, {int(x, 16) for x in a.ids.split(",") if x})


if __name__ == "__main__":
    sys.exit(main())
