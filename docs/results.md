Mean over 10 seeds. Position error vs ground truth, in metres.

| Scenario | Fault handling | RMSE all | Max error | RMSE during fault | Detected after [s] | Recoveries |
|---|---|---|---|---|---|---|
| nominal | off | 0.40 | 2.3 | - | - | 0.0 |
| nominal | gating | 0.41 | 2.3 | - | - | 0.0 |
| nominal | gating+recovery | 0.41 | 2.3 | - | - | 0.0 |
| gnss_dropout | off | 0.52 | 2.5 | 0.67 | 0.81 | 0.0 |
| gnss_dropout | gating | 0.52 | 2.5 | 0.67 | 0.81 | 0.0 |
| gnss_dropout | gating+recovery | 0.52 | 2.5 | 0.67 | 0.81 | 0.0 |
| gnss_jump | off | 4.16 | 13.3 | 9.67 | 0.00 | 0.0 |
| gnss_jump | gating | 0.41 | 2.3 | 0.28 | 0.00 | 0.0 |
| gnss_jump | gating+recovery | 0.41 | 2.3 | 0.28 | 0.00 | 0.0 |
| gnss_drift | off | 7.27 | 14.9 | 7.62 | 3.48 | 0.0 |
| gnss_drift | gating | 1.49 | 4.4 | 2.42 | 3.46 | 0.0 |
| gnss_drift | gating+recovery | 14.66 | 37.7 | 17.57 | 3.46 | 3.0 |
| wheel_scale | off | 11.86 | 24.8 | 16.54 | 0.00 | 0.0 |
| wheel_scale | gating | 0.41 | 2.3 | 0.32 | 0.00 | 0.0 |
| wheel_scale | gating+recovery | 0.41 | 2.3 | 0.32 | 0.00 | 0.0 |
| gyro_bias_step | off | 12.33 | 24.7 | 16.13 | 5.36 | 0.0 |
| gyro_bias_step | gating | 160.28 | 450.4 | 209.85 | 5.20 | 0.0 |
| gyro_bias_step | gating+recovery | 5.95 | 32.2 | 7.77 | 5.20 | 1.0 |
