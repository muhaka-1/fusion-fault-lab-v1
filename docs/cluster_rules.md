# Cluster rules

The core (`target/src/core/cluster_core.c`) is the only component that decides what the
cluster shows. The display draws what it is told, and its only rule is about itself: if
nothing valid arrives from the core for 250 ms it blanks and shows NO SIGNAL FROM CORE.
Each rule below has a unit test in `target/tests/test_core.c`. The "Module" column says when it
is built.

| # | Rule | Why | Module |
|---|---|---|---|
| R1 | Every input is debounced: a level must be stable for 3 samples (30 ms at 100 Hz) | contacts bounce for 5-15 ms | M9 |
| R2 | The hazard switch toggles on its debounced rising edge; hazard overrides the stalk | a held button must not flicker the hazards | M9 |
| R3 | Stalk left and right at the same time is implausible: no indicator | a shorted harness must not pick a side | M9 |
| R4 | Indicators blink at 90 flashes/min, 50 % duty, start lit, restart the phase on a change | UN R48 allows 90 ± 30 per minute | M9 |
| R5 | Main-beam lamp: dipped on and main selected, or flash held | flash works with the lights off | M9 |
| R6 | ECU data is used only if E2E-OK, in range (rpm ≤ 8000, speed ≤ 300 km/h) and rate-plausible (≤ 10 km/h per 20 ms frame) | a corrupted or garbage value must never reach a needle | M10 |
| R7 | ECU silent for 100 ms (or not seen 500 ms after start): ECU DATA red, speed and rpm `--` | five missed frames | M10 |
| R8 | After an ECU fault, 3 consecutive good frames are needed before values return | no flicker on a marginal bus | M10 |
| R9 | CHECK ENGINE only from a valid ECU frame with the MIL bit set | the lamp means something only if the source is trusted | M10 |
| R10 | Input board silent for 100 ms: INPUTS fault, input-driven lamps off | the cluster cannot know the switch positions | M9 |
| R11 | Nav silent for 300 ms: POSITIONING red, NAV DATA LOST, heading `--` | three missed frames | M10 |
| R12 | Nav DEGRADED or ALIGNING: amber. UNAVAILABLE: red and heading `--` | the driver is told how far to trust the position | M10 |
| R13 | A value is never shown stale: without fresh valid data it is `--` | the main safety rule of the cluster | M9 (gauges "--"), M10 |

These are safety-inspired engineering rules, not an ISO 26262 work product: there is no
hazard analysis, ASIL decomposition or certified tooling behind them.
