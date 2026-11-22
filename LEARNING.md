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


**My plan (written after reading faults.py; next time, before):**

- A Fault = kind, start, end, magnitude; apply it to a copy of the recorded sensor data only.
- Five kinds: GNSS dropout (remove fixes), jump (offset), drift (growing ramp), wheel scale
  (multiply), gyro bias step (add to gyro). Unknown kind -> error.
- Scenarios = named fault lists; run_scenario simulates, injects and runs the filter.
- Measure inside each fault window and 20 s after: RMSE, max error, max NIS.

**What I learned / differences from the reference:**

- Same numbers as the reference "fault handling off": dropout 0.53, jump 9.47, drift 7.55,
  wheel 16.48, gyro 16.25 m during (seed 1). Seed 3 within ~6 %: the faults are systematic.
- Missing data (dropout) is cheap with a good model; wrong data (jump, wheel) is expensive.
- Faults corrupt states they don't touch directly: the jump bent heading -4.2° and bias to 0.0060.
- Damage outlasts the fault because the filter is overconfident (pos σ 0.30 m at 12.8 m error).
- After the bias step the filter is still learning at 120 s (0.0204 of 0.024).
- The wheel is an independent sensor: without it the jump does 16.0 m instead of 9.5 m.

**Explain it:**

- Faults go into sensor data, not truth: a fault is a lying sensor; the truth stays the yardstick, and the
  same seed makes faulty vs normal a fair comparison.
- Dropout 0.9 m vs jump 12.8 m: a dropout removes information (dead reckoning is good), a jump adds wrong
  information the filter acts on.
- Slow recovery: overconfident (σ 0.30 m at 12.8 m error) so K is small; and heading/bias were corrupted
  too, so they keep steering it off until corrected (~40 s).
- Wheel fault: the filter trusts wheel speed (0.1 m/s), believes +1.9 m/s too fast; position integrates
  speed, so error grows until the fault ends; then heading (+2°) and position unwind slowly.
- Bias step: the filter is 200σ sure of the old bias and q_bias lets it change only ~1e-4 in 70 s, so it
  creeps (0.0048 at 60 s ... 0.0204 at 120 s). q_bias 1e-4: 3.5 m on the fault, +18 % nominal.
- NIS can't see a dropout (no fixes): use a timeout (no accepted fix > 1 s) and the filter's own σ (> 5 m).
  Slow drift is hard per sample: sum the NIS over a window (M5).
- NIS stays high after the jump (mean 16 for 56–100 s) because the filter is still wrong and overconfident:
  a lasting high NIS means "I am wrong", not "the sensor is wrong".

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

## M5 · NIS gating and the fault monitor

**My plan (written after reading the new code; next time, before):**

- Gate each update: compute NIS, compare with the chi-square value for its dof at 99.9 %;
  reject = leave x and P alone; updates return (accepted, nis, dof).
- Monitor: sum the last 10 NIS per sensor for slow faults; status NOMINAL / DEGRADED (no accepted
  fix for 1 s, or an alarm) / UNAVAILABLE (position σ > 5 m).
- Detection delay = first rejection or alarm of the affected sensor; for a dropout, first non-NOMINAL.
- Compare gating off vs on for every fault.

**What I learned / differences from the reference:**

- Same numbers as the reference (10 seeds): jump 4.16 -> 0.41 m, wheel 11.86 -> 0.41 m,
  drift 7.27 -> 1.49 m, gyro bias step 12.33 -> 160 m (diverges).
- Keeping bad data out is far cheaper than repairing it: no recovery period after the jump.
- Gating can lock the filter out: after the gyro step it rejected every fix and never learned the
  new bias (0.0042 instead of 0.024), 454 m off while its σ said 3 m.
- Detection and protection are different: the alarm sees the jump without gating, but the filter
  still uses the bad fixes.
- Mean NIS includes rejected fixes (38.5 on the jump run although the filter was fine).

**Explain it:**

- 13.8 vs 10.8: same 99.9 % false-alarm rate, different dof (GNSS 2 values, wheel 1); more values ->
  bigger normal NIS -> higher gate.
- False rejections: ~0.1 % of 18 000 fixes/h ≈ 18/h (measured 0.12 %); harmless, next fix in 0.2 s,
  DEGRADED only after 1 s.
- Rejected = untouched x and P: changing x lets bad data in; shrinking P would make the filter more
  confident without learning anything. P keeps growing honestly in predict.
- Jump 0.0 s vs drift 3.6 s: the jump's first fix is 40 m off (NIS ~700); the drift starts at 0 and the
  filter partly follows it, so NIS creeps up (0.6 -> 8.8) before the first rejection (15). The window
  catches errors that stay just under the gate every time.
- Dropout: no measurements, nothing to gate; a timeout (no accepted fix > 1 s) catches the silence (0.8 s).
- Jump OK right after with gating: the bad fixes never entered the state, so nothing to repair; in M4
  they bent position, heading and bias and the overconfident filter needed ~40 s.
- Gyro step 13x worse: the wrong heading makes honest fixes look like faults; all rejected after ~70 s,
  so no correction. Only DEGRADED because σ only models normal noise: 3 m σ at 454 m error.
- Detection with --no-gating: the monitor runs anyway; knowing (alarm at 0.0 s) is not protecting.

**Experiments (predict first, then run):**

- `EKFConfig(gate_prob=0.99)` on nominal and on `gnss_drift`: false rejections vs drift error?
- `EKFConfig(gate_prob=0.99999)` on `gnss_jump`: is the 40 m jump still rejected? And a 4 m jump?
- `MonitorConfig(window=30)` on `gnss_drift`: is the drift detected sooner or later? Why?
- `python -m fusionlab.run --scenario gyro_bias_step` and look at the status changes line: when does
  it go DEGRADED, and does it ever come back?

**Experiment results:**

| Experiment | Result | What it shows |
|---|---|---|
| gate 99 % (9.2) vs 99.9 % (13.8) | false rejections 1.3 % vs 0.3 %; drift error during 1.79 vs 2.50 m; detection 3.6 s both | stricter gate catches the drift better but rejects 4x more good fixes: missed faults vs false alarms |
| gate 99.999 % (23.0), 40 m jump | still all rejected, 0.07 m | a big fault (NIS ~700) is far above any sensible gate |
| gate 99.999 %, 4 m jump | 2/31 rejected, 0.70 m (default gate: 6/31, 0.61 m; no gating: 0.78 m) | a small fault sits at the gate's edge: partly rejected, the hard in-between case |
| window 30 vs 10 on drift | detection 3.6 s both (first rejection 43.6 s); alarm 44.6 vs 43.6 s | here the gate found the drift first; a longer window is smoother but slower |

**Done-checklist**

- [ ] `pip install -e ".[dev]"` done (scipy), `python -m pytest -q` passes (57 tests), run twice
- [ ] `python scripts/plot_m5.py` runs and I can explain all six panels and the status strips
- [ ] I can explain every line of `stats.py`, `monitor.py` and the gating in `ekf.py`
- [ ] I answered the questions above without notes
- [ ] Committed and tagged `m05-gating-monitor`

## M6 · Lock-out recovery and the benchmark

**My plan (written after reading the new code; next time, before):**

- Count GNSS rejections in a row; after 50 (10 s), assume the filter is wrong: multiply the std of
  position, heading and bias by 25 (not speed), reset the count, count recoveries.
- lockout_rejects = 0 turns it off; record recovery times in run.py; --no-recovery flag.
- Benchmark: every scenario x {off, gating, gating+recovery} x 10 seeds -> docs/results.md.

**What I learned / differences from the reference:**

- Same benchmark as the reference: gyro 209.9 -> 7.77 m with recovery; drift 2.42 -> 17.57 m.
- Recovery = admitting "I might be wrong": σ 0.53 -> 13.2 m, status briefly UNAVAILABLE.
- After recovery the filter learns the new bias (within 10 % after 19 s, 0.0241 vs 0.024).
- Fixed a metric bug: bias_err_final ignored the bias step (now 0.0001 with recovery, -0.0198 without).
- No lock-out length wins both faults (2 s: gyro 1.8 / drift 20 m; 30 s: 194 / 2.0 m).

**Explain it:**

- Lock-out: wrong heading -> honest fixes look like faults -> rejected -> heading never corrected; self-reinforcing.
- Recovery scales std of x, y, heading, bias by 25 (variance 625), keeps their correlations; speed untouched
  because the wheel kept it right.
- NIS = yᵀS⁻¹y, S = HPHᵀ + R: y is unchanged but S grows from 2.4 to ~90, so a ~30 m innovation drops from
  NIS 391 to ~11, under the 13.8 gate.
- Bias learning needs fixes getting in AND a willing bias: M5 had no fixes, M4 had fixes but tiny bias σ
  (crept to 0.0204); recovery gives both -> 0.0241.
- Drift 7x worse: the rejections were right, recovery overrides them and follows the drifting GNSS.
  NOMINAL at 29 m off is worst because nothing downstream knows to stop trusting the position.
- 89.2 s recovery helped: GNSS honest again, filter 37 m off and locked out; recovery let the honest fixes in.
  Same mechanism, right when the filter is wrong, wrong when the sensor is wrong.
- No length wins both: both faults look the same in GNSS innovations; need non-GNSS evidence (wheel vs IMU,
  turn rate checks) or the error's shape over time (GLR / CUSUM).
- 10 seeds: one seed can be luck; jump ranged 0.0–0.6 m across seeds, gyro 7.2–8.5 m. Averages make
  comparisons fair.

**Experiments (predict first, then run):**

- `EKFConfig(lockout_inflate=5)` and `EKFConfig(lockout_inflate=100)` on `gyro_bias_step` and
  `gnss_drift`: what changes?
- `python -m fusionlab.benchmark --seeds 3` vs 10: how much do the averages move?
- Inflate only the heading and bias (not position) at recovery, by editing D in `update_gnss`:
  does the gyro case still recover? Does the drift case get better or worse? (undo afterwards)

**Done-checklist**

- [ ] `python -m pytest -q` passes (68 tests), run twice
- [ ] `python scripts/plot_m6.py` runs and I can explain all four panels
- [ ] `python -m fusionlab.benchmark` runs and `docs/results.md` matches the README table
- [ ] I can explain every new line in `ekf.py`, `run.py` and `benchmark.py`
- [ ] I answered the questions above without notes
- [ ] Committed and tagged `m06-recovery-benchmark`; Phase 1 done

## M7 · C build, test framework and E2E protection

**My plan (write before reading e2e.c):**

- (your own: e.g. "CRC over the payload, counter that increments, receiver compares to the last counter")

**What I learned / differences from the reference:**

- (your own: e.g. "I forgot the message ID in the CRC" / "I didn't think of the resync state")

**Explain it:**

- **-Werror / -Wconversion:** -Werror makes warnings fail the build, so they can't pile up and get
  ignored. -Wconversion flags silent narrowing (int → uint8_t): in C, `uint8_t << 1` becomes an int,
  and putting it back in a uint8_t drops bits without a word. That is why e2e.c casts explicitly.
- **CRC protects against:** random corruption on the wire or in buffers. It catches all 1- and 2-bit
  errors in this frame, and about 255/256 of everything else.
  **It can NOT detect:** an old frame that is still valid (frozen sender), lost or reordered frames,
  wrong data that was protected correctly (garbage in, protected garbage out), or deliberate tampering.
  The CRC has no secret key, so this is safety, not security.
- **The alive counter adds:** freshness and order. It detects repeated, lost and out-of-sequence frames,
  which all have valid CRCs.
- **Message ID in the CRC:** it catches masquerading. A frame of another message with the same layout
  (wrong mailbox, gateway routing error) would otherwise pass. The demo shows the same bytes as 0x210
  giving BAD_CRC. Both sides already know the ID, so it costs no payload bytes.
- **Counter 0–14, never 15:** 0xF is reserved as invalid (AUTOSAR Profile 1). A nibble stuck at all
  ones (erased memory, bus stuck recessive) gives BAD_COUNTER instead of looking like a valid frame.
- **REPEATED vs WRONG_SEQUENCE:** REPEATED (delta 0) means the sender is still on the bus but no longer
  updating: a hung task, or the CAN controller re-sending its last mailbox. WRONG_SEQUENCE
  (delta > max_delta) means too many frames were lost (bus overload, sender reset), so the stream is
  broken and must resync.
  Catch: the counter wraps every 15 frames, so losing exactly 15 frames looks like REPEATED, and 16
  looks like OK. A receive timeout is still needed (e2e.h leaves that to the caller).
- **INITIAL after resync:** one frame alone proves nothing. It could be the first frame after a sender
  reset, with junk start values. The receiver waits for `sync_needed` frames in sequence before passing
  data to the EKF or the cluster.
- **Own test framework:** about 40 lines and no dependencies, so it builds anywhere, including the QNX
  cross compiler later. Nothing to install, plain C (GoogleTest needs C++), and small enough to read
  in full. Trade-off: no fixtures, mocks or XML reports; Unity would be the choice for a bigger project.
- **Sanitizers:** tests only check the outputs they look at.
  - ASan catches out-of-bounds reads and writes, use-after-free and stack overflows that happen not to
    change the result (e.g. reading payload[8]).
  - UBSan catches signed overflow, shifts past the type width and misaligned access: undefined
    behaviour that works on x86 gcc today and breaks on ARM, QNX or another -O level.

**Experiment results:**

| Experiment | Prediction | Result |
|---|---|---|
| `e2e_demo 0x200 16` | wraps 14 → 0 | ctr 14 → 0, still `OK`: delta = (0 + 15 − 14) mod 15 = 1 |
| Flip two bits (`^= 0x03`) | all caught | 10/10 pass. Exhaustive check of every pair of the 64 bits: 2016/2016 caught |
| (bonus) every 3- and 4-bit flip | ? | 3-bit: 198 of 41 664 **missed** (0.48 %). 4-bit: 0.386 % missed, about 1/256 |
| Poly 0x1D → 0x07 | many tests fail | **Only `crc_check_value` fails** (0x04 vs 0x4B). The other 9 pass. |
| REPEATED → OK | `repeated_frame_detected` fails | Yes, lines 95–96 (got OK = 0, wanted REPEATED = 3). The demo shows the repeated frame as OK |

**Takeaways:**

- **Round-trip tests can't catch a symmetric bug.** Sender and receiver call the same CRC function, so
  a wrong polynomial still agrees with itself. Only a known-answer test (the catalogue check value
  0x4B) catches it. On a real car, every other ECU would reject our frames. M8's Python encoder must
  match the same vector.
- **CRC-8 J1850 has Hamming distance 3 at this frame length.** It guarantees 1- and 2-bit errors, not 3.
  0x11D has an odd number of terms, so unlike some CRCs it doesn't catch all odd-weight errors.
  CAN's own 15-bit CRC sits underneath, so the E2E CRC is a second layer.
- **REPEATED returning OK on a real car:** a hung speed ECU keeps re-sending "50 km/h", and the cluster
  shows it as fresh while the car brakes to a stop. The EKF would fuse stale wheel speed as truth.

**Done-checklist**

- [x] C compiler and CMake installed (WSL), `unit_tests` says 10/10, run twice
- [x] `e2e_demo` runs and I can explain every line of its output
- [ ] I can explain every line of `e2e.c` and `test.h`
- [ ] I answered the questions above without notes
- [ ] CI shows the C jobs green
- [ ] Committed and tagged `m07-c-e2e`

## M8 · CAN messages in C and Python

**Explain it:**

- **Scaled integers instead of floats:** fixed size and exact bytes on every CPU. A float32 would use
  4 of the 6 free bytes and a double needs all 8. Small controllers often have no float hardware.
  Cost: fixed resolution and range. 12.345 m/s goes out as 1235 → 12.35 (±half a step), and
  anything above 655.34 m/s can't be sent.
- **IMU yaw rate, int24 × 1e-6 rad/s:** resolution 0.000001 rad/s, range ±8.39 rad/s (±480 °/s).
  A car turns at most about 1 rad/s, and the 0.004 rad/s gyro bias is 4000 steps, so it is plenty.
  (Accel: 1e-5 m/s², ±83.9 m/s² ≈ ±8.5 g.)
- **Saturate, don't wrap:** a wrapped 700 m/s would become 70000 mod 65536 = 4464 → 44.64 m/s, which
  looks perfectly plausible and the EKF would believe it. Saturated it becomes 655.34, which is
  clearly at the limit. -3 m/s becomes 0 instead of wrapping to 652 m/s.
- **Little endian in the spec:** the bytes on the wire are a contract between two programs, not a
  property of a CPU. If one side writes big endian, every 16- and 24-bit value is read wrong, while
  the CRC is still valid, because the sender computed it over the wrong bytes. The code builds
  bytes with shifts, so it doesn't depend on the CPU. (Many real car DBC files use big endian,
  "Motorola" order, so always check.)
- **`get_i24` and negative numbers:** it builds the 24-bit value from 3 bytes. If bit 23 (the sign
  bit) is set, it fills the top 8 bits with ones (sign extension), then reads it as int32.
  Example: NAV_POSE y = `17 f8 ff` → 0xFFF817 → 0xFFFFF817 = -2025 → -20.25 m.
- **0xFFFF = "no valid value":** a marker must be a value real data can never have. 0 is a real
  speed, so the top value is used. The encoders cap real values at 65534, so 0xFFFF only ever
  means "--". The display then shows dashes instead of a stale or made-up number.
- **GNSS "fix valid" flag:** (0, 0) is a real position: the car starts there in our local frame.
  A marker inside the data range is ambiguous. A separate flag tells the filter to skip the update.
- **`_lround`, not `round()`:** Python rounds halves to even, so round(12.5) = 12. C's lround gives
  13. A wheel speed of 12.345 m/s is 1234.5 steps: C sends 1235 (`d3 04`), Python with round()
  sends 1234 (`d2 04`). Different bytes, different CRC.
- **Why commit `can_vectors.txt`:**
  - Python can be tested anywhere without a C compiler (Git Bash, the Windows CI runner).
  - A format change shows up as a diff in review.
  - Most important, roundtrip tests can't catch a consistent mistake. Switching both put_u16 and
    get_u16 to big endian still passes 20/20. Only the frozen reference bytes catch it.
- **Flags in byte 1, high nibble:** byte 0 is the CRC, byte 1 low nibble is the alive counter, and
  the 4 high bits were free, so flags cost no data bytes. They are inside the CRC (bytes 1–7), and
  `e2e_protect` only touches the low nibble (`(p[1] & 0xF0) | counter`).

**Experiment results:**

| Experiment | Prediction | Result |
|---|---|---|
| decode `0x110 a70ed304…` | | 0x04D3 = 1235 → **12.35 m/s**, CRC valid |
| change `d3 04` → `d3 05` | BAD_CRC | **BAD_CRC**, but the decoder still prints 14.91 m/s: check E2E *before* decoding |
| `_lround` → `round` in `_sat` | | `test_encoders_match_c` fails at **wheel** (12.345 → 1234.5, an exact half). IMU and GNSS pass: their values had no exact .5 |
| `put_u16` big endian | many fail | **15/20 pass.** Fail: wheel_and_engine, nav_state, nav_cmd_and_diag, cluster_frames, encode_protect_check_decode (all use u16). Pass: the 10 E2E tests, plus IMU, GNSS, NAV_POSE (24-bit) and the id test |
| … then `can_vectors \| diff` | differs | 7 of 12 lines differ: wheel, engine, nav_state, nav_diag, nav_cmd, cluster_a, cluster_b |
| (bonus) swap `get_u16` too | | **20/20 pass**, and only the `diff` catches it |
| wheel 700 m/s, -3 m/s | saturate | `fe ff` → **655.34** (65534, never 0xFFFF) and `00 00` → **0.0** |
| NAV_STATE heading -10° | 350 | **350.00°** (`b8 88` = 35000 = 350.00°): wrapped into 0…360 by fmod + 360 |

**Takeaways:**

- A decoder turns any bytes into numbers, even corrupted ones. The rule in every receiver is
  `e2e_check` → only if usable → decode.
- Roundtrip tests prove that encode and decode agree with each other, not that they agree with the
  spec. Golden vectors prove the bytes.

**Done-checklist**

- [x] `unit_tests` says 20/20 in WSL
- [x] `can_vectors | diff - tests/can_vectors.txt` prints nothing
- [x] `python -m pytest -q` passes in Git Bash (73 passed, 1 skipped)
- [x] Committed and tagged `m08-can-messages`

## M9 · Cluster core: inputs


**Explain it:**

- **Count samples instead of a fixed wait:** a timer started at the first edge latches whatever
  level happens to be there when it ends, which can still be bounce. Counting needs 3 samples
  *in a row* that agree, so the level has really settled. It also needs no extra timer per input.
- **One agreeing sample resets the count:** that is what makes it "3 in a row". Without the reset,
  bounce like 1,0,1,1 adds up to 3 disagreeing samples and toggles in the middle of the bounce.
- **Hazard on the rising edge:** the hazard button is a push button. If the level toggled the
  hazard, holding the button would toggle it every 10 ms. The edge gives one toggle per press.
- **Left + right = no indicator:** a stalk can't be in both positions, so this is a short or a
  wiring fault. Blinking the wrong side tells other drivers a lie, which is worse than no
  indicator. Doing nothing is the safe choice.
- **Start lit, restart the phase on a change:** the driver and other road users see the effect at
  once. If the old phase continued, a new indicator could start in its off half and stay dark for
  up to 333 ms.
- **Where the time comes from:** `blink_lit` gets `now_ms` as an argument. The core never reads a
  clock, so tests run hours of blinking in microseconds, every run gives the same result, and the
  same code runs on QNX. The main program passes the real monotonic clock (M11).
- **Reset after an input fault:** during the outage the switches may have changed (the hazard
  button pressed, the stalk released). Carrying on with the old state would be stale, so the core
  starts clean from fresh readings.
- **A corrupt frame doesn't refresh the timeout:** the timeout means "no trustworthy information".
  If garbage counted, a broken board sending garbage would keep the old lamps on forever.
- **Restart the 50 ms grid after a stall:** sending all the missed frames in a burst floods the bus
  with old states, and the display replays the past. One current frame, then a steady grid.
- **Gauges `--` in M9 (R13):** there is no ECU data yet. 0 km/h would be a lie (the car could be
  moving). `--` says "unknown".

**Experiment results:**

| Experiment | Result |
|---|---|
| `0:- 200:L 3000:-` | 7 columns on, 6 off (sometimes 6/7) ≈ 667 ms → **90/min**. 4 full blinks + a partial one in 2.8 s, cut off within 30 ms of release |
| bounced press + 20 ms taps | the bounced press lights the indicator. The 20 ms taps (2 samples) never reach 3 in a row: **nothing** |
| hazard: 50 ms vs 20 ms press | 50 ms (5 samples) **turns it off**. 20 ms is **ignored**, the hazard stays on |
| `600:DX 650:D`: 50 ms gap | **no INPUT FAULT.** The board must be silent from 600 to **710 ms or more**. The fault needs > 100 ms since the last good frame (590), and at 700 a frame arrives just in time |
| `DEBOUNCE_SAMPLES 1` | **45/49**: the 3 debounce tests and `indicator_starts_lit` fail (lamp starts 20 ms earlier). And a **20 ms tap now turns the hazard off**: noise is now a command |
| `BLINK_ON_MS 400` | **47/49**: `blink_90_per_minute_half_duty` (4000 ms lit instead of 3330) and `left_indicator_blinks` (on/off counts outside 18–22) |

**Done-checklist**

- [x] `unit_tests` says 36/36 in WSL
- [x] CI is green
- [x] Committed and tagged `m09-cluster-inputs`


## M10 · Cluster core: ECU and nav supervision

**Explain it:**

- **`--` instead of the last value (R13):** a frozen value looks valid, and the driver can't tell.
  If the ECU dies at 50 km/h while you speed up to 90, the gauge says 50. `--` clearly says
  "unknown".
- **A rejected frame doesn't refresh `ecu_last_ms`:** otherwise a stream of corrupt frames would
  count as "ECU alive", and the gauges would freeze on the last good value. That is exactly the
  stale value R13 forbids.
- **What the alive counter catches here:** the ECU software hangs, but its CAN controller keeps
  sending the last frame from its buffer. The CRC is valid and the values are frozen. Only the
  repeating counter shows it (REPEATED).
- **Range and rate:** range catches a *steady* garbage value (12000 rpm every frame passes the rate
  check). Rate catches a *jump* that is still in range (50 → 150 km/h in one frame).
- **Rate limit × `last_delta`:** if frames were lost, more time has passed and the car can
  legitimately change more. Without the scaling, a lost frame during hard acceleration would cause
  a false rejection.
- **Re-anchor:** after a rejection the next frame is the new starting point, with no rate check.
  A spike that is gone in the next frame (back to 50) never shows. A real change that stays (150)
  shows one frame later. Otherwise, after a real step the core would reject forever. Trade-off:
  a garbage value repeated twice gets through the rate check (range still guards).
- **3 good frames (R8):** on a marginal bus, one good frame is often followed by more bad ones, and
  the gauge would flicker between `--` and a value. 3 in a row (60 ms) shows the link is stable.
  This is hysteresis.
- **500 ms start-up grace, lamp off while syncing:** at key-on the ECUs boot at different speeds,
  and a red lamp at every start trains drivers to ignore it. "Not valid yet" isn't "lost", so the
  lamp stays off during the first sync (the M10 fix).
- **CHECK ENGINE off when the ECU is lost:** the lamp means "the ECU reports a fault". Without a
  trusted ECU we can't claim that either way, so keeping it would be stale. ECU DATA red now
  carries the warning.
- **Heading for DEGRADED, not for UNAVAILABLE:** degraded = still usable, less accurate (amber, σ
  reported). Unavailable = the filter itself says "don't trust me", so showing it would mislead.
- **300 ms nav, 100 ms ECU:** both are a few missed frames. ECU every 20 ms → 5 frames. Nav at
  10 Hz → 3 frames. 100 ms for nav would raise an alarm on a single late frame.

**Experiment results:**

| Experiment | Result |
|---|---|
| ECU gap 80 ms | **no `--`**: last good 0.98 s, next 1.08 s = exactly 100 ms, not > 100 |
| ECU gap 120 ms | `--` and ECU DATA red from 1.10 to 1.20 s |
| speed 60 → 70 → 80 → 95 | +10 passes (the limit is "> 10"). +15 (80 → 95) is **rejected** (rate 1), re-anchored, and 95 shows one frame later (1.10 s) |
| CHECK ENGINE + corrupt | ON at 1.00, **off** with the ECU at 1.60 (ECU DATA red), ON again at 2.10 after recovery |
| nav aligning → degraded → nominal | amber, heading `--` → amber, heading 90 → no lamp, heading 90 |
| `ECU_RECOVER_FRAMES 1` | only `ecu_start_up_has_no_false_fault` fails. The demo recovers 50 ms earlier each time (1.30 vs 1.35 s, 2.50 vs 2.55 s, 3.65 vs 3.70 s). **The R8 test didn't catch it**: after a timeout the E2E resync already delays recovery, which hid the change |
| remove the `E2E_INITIAL` line | only `ecu_start_up_has_no_false_fault` fails: unconfirmed INITIAL frames count toward the 3 good frames, so values appear before the sequence is confirmed |

**Takeaway:** a test that passes can still be a weak test. Breaking the code on purpose
(mutation testing) shows which rules are really guarded. Added
`ecu_needs_three_good_frames_after_a_freeze` (R8 measured inside the core: 80 ms with 3 frames,
40 ms with 1 frame, so the test fails with 1).

**Done-checklist**

- [x] `unit_tests` says 49/49 in WSL (50/50 with the new test)
- [ ] I can point to every line of "what happened" in the demo and name the rule
- [ ] I wrote one fault script of my own and predicted the result before running it
- [ ] I answered the questions above without notes
- [x] CI is green (all 8 jobs, including Windows)
- [x] Committed and tagged `m10-cluster-supervision`
## M11 · The bus and separate processes

**My plan (write before reading canbus_udp.c and core_main.c):**

-

**What I learned / differences from the reference:**

-

**Explain it (answer out loud, then write one line each):**

- Why multicast? What would change with plain UDP to one address, or with TCP?
- Why is the TTL 1 and the interface 127.0.0.1? What could happen with a bigger TTL on a real network?
- Why split `canbus_wire.c` from `canbus_udp.c`? What is the only file to replace for real CAN hardware?
- Why does `canbus_unpack` reject datagrams that are not exactly 16 bytes, dlc > 8, or id > 0x7FF?
- `core_main.c` waits for frames until the next 10 ms tick instead of sleeping. Why?
- Why `CLOCK_MONOTONIC` and not the time of day? What happens to the core when NTP changes the clock?
- What does `core_main.c` do when it is far behind (more than 50 ms), and why?
- The display makes no decisions. What is its one rule, and why is a frozen picture dangerous?
- Why does the display draw nothing until both CLUSTER_A and CLUSTER_B are fresh?
- What does the system test check that the unit tests can't? Name two things.
- Why can't Python in Git Bash talk to `cluster_core` in WSL on 127.0.0.1?

**Experiments (predict first, then run):**

- With all three running: `left`, then `ecu freeze`, then `ecu ok`. Watch `display_term` and the
  `cluster_core -v` log. Which lamp bits change (`lamps 0x..`)? Decode them with `docs/can_messages.md`.
- In a 4th window: `PYTHONPATH=src python3 -m fusionlab.busctl monitor`. What rates do you see for
  each message? Do they match the periods in `docs/can_messages.md`?
- Stop `cluster_core` with Ctrl+C while the display runs. How fast does it say NO SIGNAL FROM CORE?
  Start it again: what does the cluster show in the first 100 ms, and why?
- `inputs off` while `dip on`: what happens to the dipped-beam lamp, and when you send `inputs on`?
- Run two benches at once (two windows with `busctl bench`). What does the core see? Which rule
  catches it? (hint: two senders, two alive counters on the same ID)
- Run the system test with `CANBUS_PORT` unchanged while your three programs run. Why doesn't it
  get confused by your bench? (look at the `rig` fixture)

**Done-checklist**

- [ ] `unit_tests` says 51/51, and the system test passes in WSL
- [ ] cluster_core, display_term and busctl bench run in three windows, and I drove the car
- [ ] I watched a fault on the monitor and on the display at the same time
- [ ] I answered the questions above without notes
- [ ] CI is green, including the new system-test step
- [ ] Committed and tagged `m11-bus-processes`
