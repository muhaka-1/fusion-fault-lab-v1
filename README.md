# fusion-fault-lab (v1, built module by module)

Sensor fusion (IMU + wheel speed + GNSS) with fault detection. This repository grows one
module at a time; each module ends in tests that pass.

| Module | Status |
|---|---|
| M1 · Simulator | done |
| M2 · EKF: predict and GNSS update | next |

## M1 · Simulator

`src/fusionlab/sim.py` drives a car along a curving road for 120 s and produces:

| Sensor | Rate | Noise |
|---|---|---|
| Gyro (yaw rate) | 100 Hz | 0.003 rad/s white noise + 0.004 rad/s constant bias |
| Accelerometer | 100 Hz | 0.05 m/s² |
| Wheel speed | 50 Hz | 0.10 m/s |
| GNSS position | 5 Hz | 1.5 m per axis |

The true states (x, y, heading, speed) are kept, so every later module can be measured against them.

```bash
pip install -e ".[dev]"
pytest -q
python scripts/plot_m1.py
```

![M1](docs/m1_sim.png)
