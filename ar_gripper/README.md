# AR Gripper driver package

This is the ROS driver for the AR Gripper.

## Architecture

The gripper logic is split so it can be driven with or without ROS:

- `ar_gripper/feetech.py` — `pyserial` Feetech SMS/STS servo driver (no ROS).
- `ar_gripper/gripper.py` — `Gripper` grasp state machine (no ROS).
- `ar_gripper/standalone.py` — `ARGripperStandalone`, a **ROS-free** class that
  owns the serial device, grasp state machine, startup calibration/persistence,
  and the position/effort unit maps.
- `ar_gripper/scripts/ar_gripper.py` — the rclpy node, a thin shim over
  `ARGripperStandalone` that bridges it to the `GripperCommand` action, the
  `calibrate` / `set_holding_torque` services, `/joint_states`,
  `~/gripper_state`, and diagnostics.

### Published state

`/joint_states` carries the finger position (m) and effort (N) for every
gripper. `~/gripper_state` (`ar_gripper_interfaces/GripperState`, 5 Hz, latched)
adds what the position alone cannot say: which way the fingers are being sent
and whether something stopped them on the way (`object_detection_status`), plus
`moving` and `calibrated`. It is a level, not an event — a gripper that stopped
on something keeps saying so until it is commanded otherwise.

Direction is reported whether or not anything is in the way, which is why there
are six values rather than Robotiq's four: `OPENING`/`CLOSING` for a move in
flight, `DETECTED_OPENING`/`DETECTED_CLOSING` once something stops it,
`NO_OBJECT` on arrival, and `UNKNOWN` only when the question has no answer —
nothing commanded yet, or an uncalibrated position that is not referenced to
anything. A consumer watching a held object needs the in-flight direction: the
fingers running inward because what they were pushing on has gone, and the
fingers travelling apart because they were told to let go, are otherwise the
same reading.

The same derivation backs the `GripperCommand` result's `reached_goal` /
`stalled` and the topic's `object_detection_status`
(`ARGripperStandalone.derive_grasp_state`), so they cannot disagree about the
same instant.

## Non-ROS usage

`ARGripperStandalone` needs only `pyserial`, so an external process (e.g. a
LeRobot imitation-learning pipeline) can drive the gripper with no
`rclpy`/`ament`/ROS. Install the ROS-free subset into a plain venv straight from
git (or a local checkout) — the rclpy-only modules are excluded from the wheel:

```bash
uv add "git+https://github.com/mrjogo/ar_gripper.git#subdirectory=ar_gripper"
uv add --editable /path/to/ar_gripper/ar_gripper   # local checkout for dev
```

Then:

```python
from ar_gripper.standalone import ARGripperStandalone

gripper = ARGripperStandalone(port="/dev/ttyUSB0", servo_id=1, name="primary")

# Read state
gripper.get_position("m")        # meters (0.0 closed .. 0.05 open)
gripper.get_position("percent")  # 0 closed .. 100 open
gripper.get_position("ticks")    # raw servo encoder ticks
gripper.get_state()              # dict: position_m/pct/ticks, load, effort_N,
                                 #       temperature, calibrated, moving

# Discrete grasps (managed: waits for stall, applies holding torque)
gripper.set_goal(0.02, unit="m", effort=300.0)  # effort in Newtons
gripper.open()
gripper.close()
gripper.release()                # torque off

# Fast, non-blocking position control for a policy loop (raw ticks)
gripper.write_goal_ticks(2048)
```

Unit conventions (all preserved across the three scales): percent — `100` open /
`0` closed; ticks — `150` open / `4095` closed; meters — `0.05` open / `0.0`
closed.

By default construction runs the calibration/persistence flow (reuse a valid
saved encoder position, otherwise physically rehome). Pass
`calibrate_on_init=False` for a connect-only session that reads state and drives
raw ticks without an automatic rehome. Building the `Gripper` always writes the
servo's configuration registers (`_init_servo`), with or without calibration.

### Single-owner serial

The RS-485 bus has exactly one owner at a time. The ROS node and an external
(e.g. LeRobot) process **cannot both hold `/dev/ttyUSB0`** — stop the ROS
`ar_gripper` node before driving the gripper from another process, and vice
versa. To share one bus across multiple grippers within a single process, build
one `USB2FeetechDevice` and inject it via the `device=` argument.

### Loopback testing (no hardware)

`ar_gripper.mock` ships an in-memory Feetech bus, so a consumer can integration-
test its gripper code against a fake servo — same API, no hardware, no ROS:

```python
from ar_gripper.mock import mock_gripper

with mock_gripper() as (gripper, bus):
    gripper.close()
    assert gripper.get_position("ticks") > 4000   # closed
    gripper.open()
    assert gripper.get_position("ticks") < 200      # open
    # bus.trace holds every servo packet that was written
```

Pass `saved_position=None, load_sequence=[5, 15, 15, 0]` to exercise a fresh
homing/calibration run instead of reusing a saved position. Lower-level building
blocks (`FakeServo`, `FakeSerial`, `loopback_bus`, `install_fake_serial`) are
exposed for finer control.

## Tests

Hardware-free by default (an in-memory Feetech bus stand-in):

```bash
cd ar_gripper && pytest            # ROS-node parity tests skip without rclpy
```

A guarded live smoke test talks to a real servo only when enabled:

```bash
AR_GRIPPER_LIVE=1 AR_GRIPPER_PORT=/dev/ttyUSB0 pytest test/test_live_smoke.py
```
