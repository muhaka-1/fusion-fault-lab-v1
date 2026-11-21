"""M6: run every scenario over several seeds in three modes and write the results table.

    python -m fusionlab.benchmark                 # 10 seeds, writes docs/results.md (~3 min)
    python -m fusionlab.benchmark --seeds 3       # quicker

Modes: no fault handling (M4), gating without recovery (M5), gating with lock-out recovery (M6).
"""
from __future__ import annotations
import argparse, json, pathlib, time
import numpy as np
from .ekf import EKFConfig
from .faults import SCENARIOS
from .run import run_scenario, metrics

MODES = {
    "off": dict(gating=False, cfg=EKFConfig()),
    "gating": dict(gating=True, cfg=EKFConfig(lockout_rejects=0)),
    "gating+recovery": dict(gating=True, cfg=EKFConfig()),
}


def _mean(vals):
    vals = [v for v in vals if v is not None]
    return float(np.mean(vals)) if vals else None


def benchmark(seeds):
    rows = {}
    for name in SCENARIOS:
        for mode, m in MODES.items():
            ms = [metrics(run_scenario(name, s, m["cfg"], gating=m["gating"])) for s in seeds]
            agg = {k: _mean([x[k] for x in ms]) for k in ms[0]}
            for k in ms[0]:
                if k.startswith("detect_delay"):
                    agg[k + "_missed"] = sum(x[k] is None for x in ms)
            rows[f"{name}|{mode}"] = agg
    return rows


def to_markdown(rows, n):
    out = [f"Mean over {n} seeds. Position error vs ground truth, in metres.\n",
           "| Scenario | Fault handling | RMSE all | Max error | RMSE during fault | Detected after [s] | Recoveries |",
           "|---|---|---|---|---|---|---|"]
    for key, a in rows.items():
        name, mode = key.split("|")
        rk = [k for k in a if k.startswith("rmse_during")]
        dk = [k for k in a if k.startswith("detect_delay") and not k.endswith("_missed")]
        rd = f"{a[rk[0]]:.2f}" if rk else "-"
        det = f"{a[dk[0]]:.2f}" if dk and a[dk[0]] is not None else "-"
        out.append(f"| {name} | {mode} | {a['rmse_m']:.2f} | {a['max_err_m']:.1f} | {rd} | {det} | {a['recoveries']:.1f} |")
    return "\n".join(out)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seeds", type=int, default=10)
    a = ap.parse_args()
    t0 = time.time()
    rows = benchmark(range(1, a.seeds + 1))
    md = to_markdown(rows, a.seeds)
    d = pathlib.Path("docs"); d.mkdir(exist_ok=True)
    (d / "results.md").write_text(md + "\n", encoding="utf-8")
    (d / "results.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(md)
    print(f"\n{len(SCENARIOS) * len(MODES) * a.seeds} runs in {time.time() - t0:.0f} s; wrote docs/results.md and docs/results.json")
