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
  `calibrate` / `set_holding_torque` services, `/joint_states`, and diagnostics.

## Non-ROS usage

`ARGripperStandalone` needs only `pyserial`, so an external process (e.g. a
LeRobot imitation-learning pipeline) can import it from a plain venv with this
package directory on `PYTHONPATH` — no `rclpy`/`ament`/ROS required:

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

## Tests

Hardware-free by default (an in-memory Feetech bus stand-in):

```bash
cd ar_gripper && pytest            # ROS-node parity tests skip without rclpy
```

A guarded live smoke test talks to a real servo only when enabled:

```bash
AR_GRIPPER_LIVE=1 AR_GRIPPER_PORT=/dev/ttyUSB0 pytest test/test_live_smoke.py
```
