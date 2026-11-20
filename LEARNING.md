# Learning log

## M1 · Simulator

**My plan (write before reading sim.py):**

-

**What I learned / differences from the reference:**

-

**Explain it (answer out loud, then write one line each):**

- Why fix the seed?
- Why is the IMU faster than GNSS?
- What does a gyro bias of 0.004 rad/s do to heading over a minute? (check with `scripts/plot_m1.py`)
- Why is the truth integrated with the trapezoid rule, while the filter (M2) will use Euler?

**Done-checklist**

- [ ] `pytest -q` passes, run twice
- [ ] `python scripts/plot_m1.py` runs and the plot makes sense to me
- [ ] I can explain every line of `src/fusionlab/sim.py`
- [ ] I answered the questions above without notes
- [ ] Committed and tagged `m01-simulator`
