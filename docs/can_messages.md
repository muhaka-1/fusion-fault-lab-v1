# CAN message set

Classic CAN, 11-bit identifiers, 8 data bytes, little endian. Defined in
`target/include/can_msg.h` (C) and `src/fusionlab/canbus.py` (Python);
`tests/test_can_contract.py` checks that both encode every message to the same bytes, using the
reference frames in `tests/can_vectors.txt` (printed by `target/tools/can_vectors.c`).

## End-to-end protection (every message)

Modelled on AUTOSAR E2E Profile 1, simplified and not certified:

| Byte | Content |
|---|---|
| 0 | CRC-8 SAE J1850 (poly 0x1D, init 0xFF, xor-out 0xFF) over the 16-bit message ID (low byte first) and bytes 1..7 |
| 1, bits 0..3 | alive counter 0..14, +1 per frame (15 is invalid) |
| 1, bits 4..7 | message flags |

Including the ID in the CRC means a frame copied onto the wrong ID is rejected too.
The receiver classifies each frame: `OK`, `OK_SOME_LOST` (counter jumped but within
tolerance), `INITIAL` (first frame or resync), `REPEATED` (sender hung, counter frozen),
`WRONG_SEQUENCE`, `BAD_CRC`. Only `OK` and `OK_SOME_LOST` count as fresh data for the cluster.

## Messages

| ID | Name | Sender → receiver | Period | Bytes 2..7 | Flags (byte 1 high nibble) |
|---|---|---|---|---|---|
| 0x100 | IMU | sensors → nav | 10 ms | 2-4 yaw rate int24, 1e-6 rad/s · 5-7 accel int24, 1e-5 m/s² | – |
| 0x110 | WHEEL | sensors → nav | 20 ms | 2-3 speed u16, 0.01 m/s | – |
| 0x120 | GNSS | receiver → nav | 200 ms | 2-4 x int24, 0.01 m · 5-7 y int24, 0.01 m (local ENU) | bit 0: fix valid |
| 0x200 | ENGINE | ECU → core | 20 ms | 2-3 rpm u16 · 4-5 vehicle speed u16, 0.01 km/h | bit 0: MIL (check engine) · bit 1: running |
| 0x210 | INPUTS | input board → core | 10 ms | 2: raw levels: 0 stalk left, 1 stalk right, 2 hazard switch, 3 dipped-beam switch, 4 main beam, 5 flash | – |
| 0x300 | NAV_STATE | nav → core | 100 ms | 2 flags · 3-4 heading u16, 0.01° · 5-6 position σ u16, cm · 7 GNSS age, 0.1 s | bits 0-1: 0 NOMINAL, 1 DEGRADED, 2 UNAVAILABLE, 3 ALIGNING |
| 0x301 | NAV_POSE | nav → any | 100 ms | 2-4 x int24, cm · 5-7 y int24, cm | – |
| 0x302 | NAV_DIAG | nav → any | 100 ms | 2-3 speed u16, 0.01 m/s · 4-5 gyro bias int16, 1e-5 rad/s · 6 rejects in a row (wraps) · 7 recoveries | – |
| 0x310 | NAV_CMD | bench → nav | on change, 1 s | 2-3 recovery after N rejects u16 (0 = off) · 4-5 (1 − gate probability) u16, ppm | bit 0: fault handling on · bit 1: re-align |
| 0x400 | CLUSTER_A | core → display | 50 ms | 2-3 speed u16, 0.1 km/h · 4-5 rpm u16 · 6 lamps · 7 lamps 2 | – |
| 0x401 | CLUSTER_B | core → display | 50 ms | 2-3 heading u16, 0.01° · 4 positioning text · 5-6 position σ u16, cm | – |

`0xFFFF` in a CLUSTER field means "no valid value": the display draws `--`.

NAV_STATE flags: 0 GNSS alarm, 1 wheel alarm, 2 GNSS fix rejected in the last second, 3 IMU lost,
4 a lock-out recovery has happened, 5 corrupted sensor frames were dropped.

CLUSTER_A lamps: 0 left, 1 right, 2 dipped, 3 main beam, 4 check engine, 5 ECU data fault,
6 positioning amber, 7 positioning red. Lamps 2: 0 input fault, 1 hazard active.
Positioning text: 0 OK, 1 DEGRADED, 2 UNAVAILABLE, 3 ALIGNING, 4 NAV DATA LOST.

## Where this is used

| Module | Uses |
|---|---|
| M8 | the codecs in C (`can_msg.c`) and Python (`canbus.py`), checked against each other |
| M9, M10 | `cluster_core` decodes ENGINE and INPUTS, encodes CLUSTER_A/B |
| M11 | the frames travel over UDP multicast (the transport is added there) |
| M13 | `nav` decodes IMU, WHEEL, GNSS, NAV_CMD and encodes NAV_STATE, NAV_POSE, NAV_DIAG |
| M14 | the Python bench encodes the sensors, ECU and inputs, and decodes everything else |
