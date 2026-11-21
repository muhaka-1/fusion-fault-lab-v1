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

**My plan (written after reading ekf.py; next time, before):**

- Add b_g as a 5th state, subtract it in predict: psi += (gyro - b_g) dt, and F[2,4] = -dt.
- Let the bias wander slowly (q_bias) and start it at 0 with a wide P0 (0.02).
- Add update_wheel: a 1-D measurement of speed (H picks state 3, R = 0.1²).
- Compare all four combinations: with/without bias state, with/without wheel.

**What I learned :**

- Same numbers as the reference: 0.275 m with both, 7.34 m without the bias state, 0.32 m
  without the wheel, 0.277 m with bias 0.01; the bias is learned as 0.0040.
- The bias state fixes position (27x) and makes the filter honest again (NIS 1.96 / 0.97).
- The wheel makes speed ~7x better on every seed, but does nothing about the bias.
- Freezing the bias state with the wheel off reproduces M2 exactly (7.281 m): M3 only adds.
- Observability: bias is learned on a straight road too (0.0040 after 40 s), but NOT at standstill
  (σ stays 0.013): heading only affects position when the car moves.

**Explainations:**

- How is the bias learned? Predict links bias -> heading (F[2,4]) -> position (F[0,2]) in P; a GNSS
  fix that shows a sideways error corrects the bias through those correlations. Without F[2,4], no learning.
- Why 20–30 s, same for any size? The bias shows only after two integrations and must stand out
  against 1.5 m noise; P and K don't depend on the true bias, so the error shrinks at the same rate.
- Why a random walk? Real biases change (temperature, age, power-on); a constant would drive P to 0
  and the filter would stop learning and miss a bias step. q_bias keeps a small floor (σ 7e-5).
- Straight at constant speed? Still learned: a heading error makes a moving car drift sideways.
  Standing still is unobservable: heading then has no effect on position.
- Wheel improves position: better speed -> less error in distance travelled between fixes
  (along-track); GNSS + bias handle the cross-track error.
- Wheel NIS 1, GNSS NIS 2: NIS averages its degrees of freedom; speed is 1 number, a fix is 2.
- update_wheel moves only speed at first because P0 is diagonal; predict then links speed to position
  (F[0,3], F[1,3]), so later a speed surprise also moves x, y (+0.3 m/s -> +0.23 m along track).

**Experiment results:**

Bias 0.01: 18.2 m and NIS 148. The error grows in proportion to the bias (2.5× the bias,
2.5× the error), and the NIS with its square (≈ 6×), because NIS is a squared quantity.

`sigma_gnss=0.5` (telling the filter GNSS is 3× better than it is):

| Drive | Setting | RMSE | Mean NIS | Within 2σ |
|---|---|---|---|---|
| with bias | honest 1.5 | 7.28 m | 25 | 12 % |
| with bias | 0.5 | 2.74 m | 46 | 4 % |
| no bias | honest 1.5 | 0.26 m | 1.96 | 100 % |
| no bias | 0.5 | 0.35 m | 17.6 | 62 % |

**Done-checklist**

- [ ] `python -m pytest -q` passes (26 tests), run twice
- [ ] `python scripts/plot_m3.py` runs and I can explain all four panels
- [ ] I can explain every new line in `src/fusionlab/ekf.py`
- [ ] I answered the questions above without notes
- [ ] Committed and tagged `m03-bias-wheel`
## M4 · Fault injection and metrics

**My plan (write before reading faults.py):**

-

**What I learned / differences from the reference:**

-

**Explain it (answer out loud, then write one line each):**

- Why inject faults into recorded data instead of into the simulator's truth?
- Why is a 20 s GNSS dropout almost harmless (0.5 m), when a 6 s jump costs 13 m?
- The jump lasts 6 s, but the error is still 7 m 20 s later. Why does the filter recover so slowly?
  (hint: what is K late in the run, from M2?)
- Why does the wheel fault hurt the position so much, when the wheel only measures speed?
  And why is the error still growing at the moment the fault ends?
- The gyro bias step: the filter does learn the new bias in the end (it reaches ~0.02). Why does it
  take so long, and what does q_bias have to do with it? (M3)
- Which fault can the NIS *not* show, and what could detect it instead?
- After the jump ends, why does the NIS stay above 2 for ~40 s?

**Experiments (predict first, then run):**

- `python -m fusionlab.run --scenario gnss_jump --no-wheel`: does the wheel help or hurt here?
- Make the jump 4 m instead of 40 m (copy the scenario in Python with `Fault("gnss_jump", 50, 56, 4.0)`
  and run it with `apply_faults` + `run_filter`). Is the max NIS still far above 2? Is a 4 m jump
  easy or hard to detect?
- `EKFConfig(q_bias=1e-4)` on `gyro_bias_step`: does a faster-wandering bias help the filter follow
  the step? What does it cost on the nominal drive?
- Run `plot_m4.py --seed 3`: are the numbers similar? Which fault varies most between seeds?

**Experiment results:**

| Experiment | Result | What it shows |
|---|---|---|
| `gnss_jump --no-wheel` | 16.0 m during (vs 9.5 m), max 24.8 m; speed error 0.30 m/s (vs 0.012) | the wheel is an independent sensor that limits the damage: without it, the jump corrupts speed too (redundancy) |
| 4 m jump instead of 40 m | 0.78 m during; max NIS 25.9 (normal fixes reach ~10, M5 gate 13.8) | small faults do little harm and are only just detectable; in-between faults are the hard case |
| `q_bias=1e-4` on `gyro_bias_step` | 3.51 m during (vs 16.25 m); nominal 0.32 m (vs 0.27 m) | following bias changes faster helps a lot on the fault but costs 18 % normally: no single setting wins both |
| `plot_m4.py --seed 3` | jump 10.1, drift 7.7, wheel 16.7, gyro 16.1 m (all within ~6 % of seed 1); dropout 0.23 vs 0.53 m | the faults are systematic; only the dropout varies, because its error is random drift |

**Done-checklist**

- [ ] `python -m pytest -q` passes (44 tests), run twice
- [ ] `python scripts/plot_m4.py` runs and I can explain all six panels
- [ ] I can explain every line of `src/fusionlab/faults.py` and the new parts of `run.py`
- [ ] I answered the questions above without notes
- [ ] Committed and tagged `m04-faults`
