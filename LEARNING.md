# Learning log

## M1 · Simulator

**My plan (written after reading sim.py; next time, before):**

- Make a smooth speed and yaw-rate profile, integrate them into the true x, y, heading.
- Add sensors on top of the truth: gyro = true rate + bias + noise, accelerometer, wheel speed, GNSS.
- Sample each sensor at its own rate (100 / 50 / 5 Hz) by taking every n-th IMU step.
- Use a seeded random generator so every run is reproducible.

**What I learned :**

- My run gives the same numbers as the reference: 12001 IMU, 6000 wheel, 600 GNSS samples;
  GNSS error std 1.48 m (configured 1.5); gyro mean error 0.0040 rad/s; 13.7° drift after 60 s.
- Changing the seed changes the noise, not the path: the motion profile has no randomness.
- With the bias set to 0, the gyro noise alone drifts only ~0.08° in 60 s: the bias is what hurts.
- Code is the reference sim.py; I added 4 tests (rates, truth consistency, noise size, gyro bias).

**Explaination:**

- Why fix the seed? Same noise every run: deterministic tests, reproducible bugs, fair comparisons
  between filter versions. Several seeds on purpose when I want the spread (M6 uses 10).
- Why is the IMU faster than GNSS? It measures rates that must be integrated in small steps and is
  cheap to read; GNSS needs satellite processing (1–10 Hz). IMU predicts between fixes, GNSS corrects.
- What does 0.004 rad/s bias do in a minute? Heading error grows linearly to 0.24 rad ≈ 13.7°;
  position error grows with t², about 86 m sideways at 12 m/s. So the filter must estimate the bias.
- Why trapezoid for truth, Euler in the filter? Truth should be accurate; the filter uses the cheap
  model it linearises. The mismatch is deliberate: a filter tested on its own model looks too good.

**Done-checklist**

- [x] `python -m pytest -q` passes, run twice
- [x] `python scripts/plot_m1.py` runs and the plot makes sense to me
- [x] I can explain every line of `src/fusionlab/sim.py`
- [x] I answered the questions above without notes
- [x] Committed and tagged `m01-simulator`


## M2 · EKF: predict and GNSS update

The experiments, with results

Bias 0.01: 18.2 m and NIS 148. The error grows in proportion to the bias (2.5× the bias, 2.5× the error), and the NIS with its square (≈ 6×), because NIS is a squared quantity.

sigma_gnss=0.5 (telling the filter GNSS is 3× better than it is):

        Drive	    Setting	    RMSE	   Mean NIS	    Within 2σ
        with bias	honest 1.5	7.28 m	    25	        12 %
        with bias	0.5	        2.74 m	    46	        4 %
        no bias	    honest 1.5	0.26 m	    1.96	    100 %
        no bias	    0.5	        0.35 m	    17.6	    62 %


**My plan (written after reading ekf.py; next time, before):**

- State [x, y, heading, speed]; IMU (gyro, acc) as input to the prediction every 10 ms.
- Predict: move the state with the motion model, grow P with F and the IMU noise.
- Update at each GNSS fix: innovation y, its expected size S, gain K, correct x and P (Joseph form).
- Measure against the truth: RMSE, heading error, and NIS to see if the filter is honest.

**What I learned :**

- Same numbers as the reference: 0.26 m without bias, 7.28 m with bias, 0.53 m with inflated gyro noise.
- The bias error is systematic: other seeds give the same 7.3 m; 2.5× the bias gives 2.5× the error
  and ~6× the NIS. Systematic errors must be modelled, not averaged away (-> M3).
- K for x falls from 0.92 at the first fix to 0.02 after 120 s: the filter trusts itself more and more,
  which is why the biased filter ignores the fixes.
- Lying about noise (gyro 0.1, or GNSS 0.5) hides the bias but makes the filter follow GNSS harder,
  so it will follow GNSS faults too. Honest noise + a better model is the real fix.
- Early heading spike (-8.7° at 1 s): large P0 makes K ~1, noisy first fixes move the position, and
  the position-heading link in P turns part of that into heading.
- Code is the reference M2 (4-state version); 9 tests added.

**Explaination:**

- State: x, y, heading, speed. Acc and yaw rate are measured every step, so they are inputs, not states;
  heading is not measured directly, so the filter must estimate it.
- F is the slope of the non-linear motion model; dx/dψ = -v sinψ dt and dy/dψ = v cosψ dt link heading
  errors to sideways position errors, which is how GNSS corrects heading.
- P = how unsure I am; S = how big I expect the innovation to be; K = how far to move towards the fix.
  K shrinks because each fix shrinks P and little process noise is added between fixes.
- Joseph form keeps P symmetric and positive even with rounding or a slightly wrong K; the short form
  subtracts nearly equal matrices and can go negative.
- NIS = yᵀS⁻¹y, the innovation in units of what was expected; chi-square with 2 dof for a 2-D fix, so it
  averages 2 when the model is right (mine: 1.96).
- With bias: the filter trusts the drifting gyro and is overconfident (K≈0.02), so it settles metres off
  the road; fixes keep disagreeing far beyond expectation -> NIS ≈ 25.
- Inflating gyro noise makes the filter lean on GNSS; it then follows GNSS faults (M5), which defeats
  fault detection.

**Experiments:**

- Bias 0.01: RMSE 18.2 m, NIS 148. Error proportional to the bias, NIS to its square.
- sigma_gnss 0.5: no-bias drive gets worse (0.26 -> 0.35 m, NIS 17.6, 62 % within 2σ); biased drive
  "improves" to 2.74 m but only 4 % within 2σ: overconfident, follows GNSS noise. Don't lie about R.
- Without symmetrising: asymmetry ~3e-18, no change in 120 s. Keep it: cheap, protects long runs, bigger
  states and C/float code.

**Done-checklist**

- [x] `python -m pytest -q` passes (14 tests), run twice
- [x] `python scripts/plot_m2.py` runs and I can explain all four panels
- [x] I can explain every line of `src/fusionlab/ekf.py`
- [x] I answered the questions above without notes
- [x] Committed and tagged `m02-ekf-gnss`

## M3 · Gyro bias and wheel speed

**My plan (write before reading the new parts of ekf.py):**

-

**What I learned / differences from the reference:**

-

**Explain it (answer out loud, then write one line each):**

- Nothing measures the bias directly. How does the filter learn it? (look at F[2, 4] and at what
  GNSS fixes say about heading)
- Why does it take 20–30 s to learn, and why is that time the same for a bias of 0.004 and 0.01?
- Why is the bias modelled as a slow random walk (q_bias) instead of a fixed constant?
- What would happen if the car only drove straight at constant speed? (hint: observability)
- The wheel measures speed only. Why does it still improve the position a little (0.32 -> 0.27 m)?
- Why does the wheel NIS average 1 while the GNSS NIS averages 2?
- Why does `update_wheel` move only the speed at the start, but later also nudge x, y and heading?

**Experiments (predict first, then run):**

- `python -m fusionlab.run --bias -0.006`: does a negative bias work as well?
- `EKFConfig(p0_bias=0.001)`: the filter is told the bias is almost certainly tiny. How long does it
  take to learn 0.004 now? Look at the bias panel of a plot or print `r.est[::500, 4]`.
- `EKFConfig(q_bias=1e-3)`: the bias may wander 100x faster. What happens to the bias estimate and
  the heading? Is the filter still consistent?
- `python -m fusionlab.run --no-wheel --seed 4` vs with the wheel: how much does the wheel help on
  a different drive?

**Experiment results:**

| Experiment | Result | What it shows |
|---|---|---|
| `--bias -0.006` | 0.274 m, bias learned exactly (−0.0060) | sign and size don't matter |
| `p0_bias=0.001` (told the bias is tiny) | bias every 10 s: 0, 0.0003, 0.0024, 0.0037, 0.0040; RMSE 0.33 m | a too-confident start slows learning from ~10 s to ~40 s (default: 0.0043 already at 10 s) |
| `q_bias=1e-3` (bias may wander 100× faster) | RMSE 0.42 m; bias jumps between 0.0015 and 0.0063 after 30 s; σ 0.0027; NIS 1.95 | still consistent, but noisy: it chases noise as if it were bias changes |
| wheel on / off, seeds 2, 3, 4 | 0.38 / 0.52, 0.51 / 0.57, 0.43 / 0.50 m; speed 0.009 / 0.06 m/s | the wheel helps on every drive; speed is always ~7× better |

**Done-checklist**

- [ ] `python -m pytest -q` passes (26 tests), run twice
- [ ] `python scripts/plot_m3.py` runs and I can explain all four panels
- [ ] I can explain every new line in `src/fusionlab/ekf.py`
- [ ] I answered the questions above without notes
- [ ] Committed and tagged `m03-bias-wheel`
