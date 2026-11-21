# Learning log

## M1 · Simulator

**My plan (written after reading sim.py; next time, before):**

- Make a smooth speed and yaw-rate profile, integrate them into the true x, y, heading.
- Add sensors on top of the truth: gyro = true rate + bias + noise, accelerometer, wheel speed, GNSS.
- Sample each sensor at its own rate (100 / 50 / 5 Hz) by taking every n-th IMU step.
- Use a seeded random generator so every run is reproducible.

**What I learned / differences from the reference:**

- My run gives the same numbers as the reference: 12001 IMU, 6000 wheel, 600 GNSS samples;
  GNSS error std 1.48 m (configured 1.5); gyro mean error 0.0040 rad/s; 13.7° drift after 60 s.
- Changing the seed changes the noise, not the path: the motion profile has no randomness.
- With the bias set to 0, the gyro noise alone drifts only ~0.08° in 60 s: the bias is what hurts.
- Code is the reference sim.py; I added 4 tests (rates, truth consistency, noise size, gyro bias).

**Explain it:**

- Why fix the seed? Same noise every run: deterministic tests, reproducible bugs, fair comparisons
  between filter versions. Several seeds on purpose when I want the spread (M6 uses 10).
- Why is the IMU faster than GNSS? It measures rates that must be integrated in small steps and is
  cheap to read; GNSS needs satellite processing (1–10 Hz). IMU predicts between fixes, GNSS corrects.
- What does 0.004 rad/s bias do in a minute? Heading error grows linearly to 0.24 rad ≈ 13.7°;
  position error grows with t², about 86 m sideways at 12 m/s. So the filter must estimate the bias.
- Why trapezoid for truth, Euler in the filter? Truth should be accurate; the filter uses the cheap
  model it linearises. The mismatch is deliberate: a filter tested on its own model looks too good.

**Done-checklist**

- [ ] `python -m pytest -q` passes, run twice
- [x] `python scripts/plot_m1.py` runs and the plot makes sense to me
- [ ] I can explain every line of `src/fusionlab/sim.py`
- [ ] I answered the questions above without notes
- [ ] Committed and tagged `m01-simulator`

## M2 · EKF: predict and GNSS update

**My plan (write before reading ekf.py):**

-

**What I learned / differences from the reference:**

-

**Explain it (answer out loud, then write one line each):**

- What is in the state, and why is heading in it but not acceleration?
- What does the Jacobian F linearise, and which entries change with heading and speed?
- What do P, S and K mean in words? Why does K get smaller as the run goes on?
- Why the Joseph form instead of the shorter (I − KH)P?
- What is the NIS, and why should it average 2 for a GNSS fix?
- With the bias, why is the filter *worse* than raw GNSS, and why does its NIS jump to ~25?
- Inflating the gyro noise to 0.1 fixes the RMSE. What does that cost? (hint: what will the
  filter do when GNSS itself goes wrong in M5?)

**Experiments (predict first, then run):**

- `python -m fusionlab.run --bias 0.01`: RMSE and NIS? Why?
- `EKFConfig(sigma_gnss=0.5)` on the default drive: does telling the filter GNSS is better than it
  really is help or hurt? Look at `within_2sigma`.
- Comment out `self.P = 0.5 * (self.P + self.P.T)` in both places: does anything change? Why keep it?

**Done-checklist**

- [ ] `python -m pytest -q` passes (14 tests), run twice
- [ ] `python scripts/plot_m2.py` runs and I can explain all four panels
- [ ] I can explain every line of `src/fusionlab/ekf.py`
- [ ] I answered the questions above without notes
- [ ] Committed and tagged `m02-ekf-gnss`

## M2 · EKF: predict and GNSS update

**My plan (write before reading ekf.py):**

-

**What I learned / differences from the reference:**

-

**Explain it (answer out loud, then write one line each):**

- What is in the state, and why is heading in it but not acceleration?
- What does the Jacobian F linearise, and which entries change with heading and speed?
- What do P, S and K mean in words? Why does K get smaller as the run goes on?
- Why the Joseph form instead of the shorter (I − KH)P?
- What is the NIS, and why should it average 2 for a GNSS fix?
- With the bias, why is the filter *worse* than raw GNSS, and why does its NIS jump to ~25?
- Inflating the gyro noise to 0.1 fixes the RMSE. What does that cost? (hint: what will the
  filter do when GNSS itself goes wrong in M5?)

**Experiments (predict first, then run):**

- `python -m fusionlab.run --bias 0.01`: RMSE and NIS? Why?
- `EKFConfig(sigma_gnss=0.5)` on the default drive: does telling the filter GNSS is better than it
  really is help or hurt? Look at `within_2sigma`.
- Comment out `self.P = 0.5 * (self.P + self.P.T)` in both places: does anything change? Why keep it?

**Done-checklist**

- [ ] `python -m pytest -q` passes (14 tests), run twice
- [ ] `python scripts/plot_m2.py` runs and I can explain all four panels
- [ ] I can explain every line of `src/fusionlab/ekf.py`
- [ ] I answered the questions above without notes
- [ ] Committed and tagged `m02-ekf-gnss`
