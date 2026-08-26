"""ROS-free driver for the AR gripper.

``ARGripperStandalone`` owns everything needed to drive and read the gripper
without ROS: it builds the serial device + grasp state machine, runs the
startup calibration/persistence flow, and exposes the unit maps shared with the
ROS node. A non-ROS process (e.g. a LeRobot imitation-learning pipeline) can
import this from a bare venv that has only ``pyserial`` on ``PYTHONPATH`` — see
the package README's "Non-ROS usage" section.

The ROS node (``scripts/ar_gripper.py``) is a thin shim over this class; the
unit maps, calibration persistence, and startup flow live here so both consumers
share one definition. This module has ZERO ROS imports.

Serial ownership is exclusive: the ROS node and an external process cannot both
hold ``/dev/ttyUSB0``. Exactly one owner at a time.
"""

import json
import logging
import os
from collections import namedtuple
from math import isclose

from ar_gripper.feetech import USB2FeetechDevice
from ar_gripper.gripper import CalibrationError, Gripper

logger = logging.getLogger(__name__)

#: What the fingers are doing right now, derived once and shared by every
#: consumer (see ``ARGripperStandalone.derive_grasp_state``).
GraspState = namedtuple("GraspState", "reached_goal stalled object_detection_status")


class ARGripperStandalone:
    """ROS-free owner of one AR gripper (serial device + grasp state machine).

    Positions are inverted across three conventions, preserved exactly:
    percent (100 == open, 0 == closed), servo ticks (150 == open, 4095 ==
    closed), and meters (0.05 == open, 0.0 == closed).
    """

    SAVED_POSITION_KEY = "position"
    # Shared calibration file, used by BOTH the ROS node and every non-ROS consumer so a
    # process doesn't force a rehome just because a different process last used the gripper.
    # (The ROS node declares the same value as its `servo_position_path` parameter default.)
    # Lives under XDG_STATE_HOME (~/.local/state) — the standard spot for regenerable state
    # reused on the next run; NOT ~/.config (that's for user-edited config) or ~/.cache
    # (a cache cleaner wiping it would force an unexpected rehome).
    DEFAULT_SERVO_POSITION_PATH = os.path.join(
        os.environ.get("XDG_STATE_HOME") or os.path.expanduser("~/.local/state"),
        "ar_gripper",
        "servo_position.json",
    )
    MIN_PERCENT = 0.0
    MAX_PERCENT = 100.0
    FINGER_CLOSED_POS = 0.0
    FINGER_OPEN_POS = 0.05
    MIN_EFFORT = 0.0
    # Approximate, not accounting for losses
    # Stall torque x gear radius
    # (85 kg*cm * 9.8 m/s^2 * 0.01 m/cm) / 0.008 m
    MAX_EFFORT = 1041.25  # Newtons
    # For reference, "rated" effort is ~1/3 of stall torque: 347.0833 N

    # How close to the commanded position counts as having reached it. 1 mm of
    # a 50 mm stroke; the value the GripperCommand action result has always
    # used, kept here so the topic cannot answer the same question differently.
    GOAL_POSITION_TOL_M = 0.001

    # Object-detection status values. These MIRROR the constants on
    # ar_gripper_interfaces/GripperState, by name and by number -- this module
    # is deliberately ROS-free (see the module docstring) so it cannot import
    # them, and a non-ROS consumer needs them just as much as the node does.
    # test_grasp_state.py asserts the two sets stay equal.
    OBJECT_DETECTION_UNKNOWN = 0
    OBJECT_DETECTION_DETECTED_OPENING = 1
    OBJECT_DETECTION_DETECTED_CLOSING = 2
    OBJECT_DETECTION_NO_OBJECT = 3

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baud=115200,
        servo_id=1,
        name="primary",
        servo_position_path=DEFAULT_SERVO_POSITION_PATH,
        device=None,
        calibrate_on_init=True,
    ):
        """Build the serial device + gripper and run the startup calibration flow.

        :param port: serial port to open (ignored when ``device`` is injected).
        :param baud: baud rate for the RS-485 bus.
        :param servo_id: Feetech servo RS-485 ID for this gripper.
        :param name: gripper name (used for the joint name / logging).
        :param servo_position_path: JSON file used to remember the last encoder
            position so a valid saved value can skip a physical rehome. Defaults to
            ``DEFAULT_SERVO_POSITION_PATH`` (the same file the ROS node uses) so every
            way of running the gripper shares one calibration and a fresh process does
            not rehome just because a different process used it. Pass ``None`` to
            disable persistence (always rehomes on init, never writes a file).
        :param device: an already-open ``USB2FeetechDevice`` to share (the ROS
            node injects one device across all grippers on the bus). When
            ``None`` a new device is opened from ``port``/``baud``.
        :param calibrate_on_init: when ``True`` (default, current behaviour) run
            the load/verify/calibrate/save flow on construction. When ``False``
            skip it — an opt-in "connect only" for a policy loop that drives raw
            ticks and does not want an automatic physical rehome. ``_init_servo``
            (config-register writes) always runs when the ``Gripper`` is built.
        """
        self.name = name
        self._servo_position_path = (
            os.path.expanduser(servo_position_path)
            if servo_position_path
            else servo_position_path
        )

        if device is None:
            device = USB2FeetechDevice(port, baudrate=baud)
        self._device = device
        self.gripper = Gripper(device, name, servo_id)
        self._holding_torque = self.gripper.HOLDING_TORQUE
        # Last position this driver commanded, in metres, or None when nothing
        # is outstanding (nothing commanded yet, torque released, just homed).
        # Every command path below maintains it; the ROS node assigns it
        # directly because its action drives ``self.gripper`` rather than going
        # through ``set_goal``. Without it "stopped BEFORE the requested
        # position" has nothing to be measured against.
        self._goal_position_m = None

        if self._servo_position_path is not None:
            os.makedirs(os.path.dirname(self._servo_position_path), exist_ok=True)

        if calibrate_on_init:
            self.run_startup_calibration()

    # -- startup calibration / persistence -------------------------------------------
    def run_startup_calibration(self):
        """Reuse a valid saved position, else physically rehome and persist.

        Runs automatically from ``__init__`` when ``calibrate_on_init`` is True.
        Exposed so the ROS node can advertise its action/service endpoints first
        (matching the pre-shim ordering) and then trigger calibration. Raises
        ``CalibrationError`` if a required rehome fails.
        """
        previous_position = self._load_saved_position()
        if previous_position is None or not self.gripper.verify_calibrated(
            previous_position
        ):
            if not self.gripper.calibrate():
                raise CalibrationError("Gripper calibration failed")
            self.save_position()
        else:
            logger.info("Using previous gripper calibration")

    def _load_saved_position(self):
        if self._servo_position_path is None:
            return None
        try:
            with open(self._servo_position_path, "r") as f:
                return (json.load(f))[self.SAVED_POSITION_KEY]
        except (FileNotFoundError, json.decoder.JSONDecodeError, KeyError):
            return None

    def save_position(self):
        """Persist the current encoder position (no-op when persistence is off)."""
        if self._servo_position_path is None:
            return
        with open(self._servo_position_path, "w") as f:
            json.dump({self.SAVED_POSITION_KEY: self.gripper.get_servo_position()}, f)

    # -- state / reads ----------------------------------------------------------------
    @property
    def is_calibrated(self):
        return self.gripper.calibrated

    @property
    def holding_torque(self):
        return self._holding_torque

    @holding_torque.setter
    def holding_torque(self, value):
        if value < 0 or value >= self.gripper.OVERLOAD_TORQUE:
            raise ValueError(
                f"Holding torque {value} must be between 0 and "
                f"{self.gripper.OVERLOAD_TORQUE}"
            )
        self._holding_torque = value

    def get_position(self, unit="m"):
        """Live gripper position in ``"m"``, ``"percent"``, or ``"ticks"``."""
        if unit == "ticks":
            return self.gripper.get_servo_position()
        percent = self.gripper.get_position()
        if unit == "percent":
            return percent
        if unit == "m":
            return self.percent_to_stroke(percent)
        raise ValueError(f"unit must be 'm', 'percent', or 'ticks', not {unit!r}")

    @property
    def goal_position_m(self):
        """Last commanded position in metres, or ``None`` if none is outstanding."""
        return self._goal_position_m

    @goal_position_m.setter
    def goal_position_m(self, value):
        self._goal_position_m = None if value is None else float(value)

    @classmethod
    def derive_grasp_state(cls, position_m, effort_N, goal_position_m, calibrated):
        """Decide, from one instant's measurements, what the fingers did.

        The single definition of "reached the goal" / "stalled" / "detected an
        object" in this driver. The GripperCommand action result and the
        ``~/gripper_state`` topic both report those, about the same gripper, and
        a consumer that saw them disagree would have no way to tell which was
        right -- so there is one derivation and both call it.

        The direction is read off the goal rather than tracked separately: a
        move that stopped short stopped somewhere BETWEEN where it started and
        where it was sent, so the sign of ``goal - position`` is the sign of the
        travel that was interrupted. Closed is 0 m and open is 0.05 m, so a goal
        below the current position was a close.

        :param position_m: measured finger position, metres.
        :param effort_N: measured effort, newtons.
        :param goal_position_m: last commanded position in metres, or ``None``.
        :param calibrated: whether position means anything yet.
        :returns: a :class:`GraspState`.
        """
        commanded = goal_position_m is not None
        reached_goal = commanded and isclose(
            position_m, goal_position_m, abs_tol=cls.GOAL_POSITION_TOL_M
        )
        # Stopped short of where it was sent, while still pushing: something is
        # in the way. Effort alone does not mean contact -- holding torque at
        # the goal reads the same -- which is why both halves are required.
        stalled = commanded and not reached_goal and effort_N > 0

        if not commanded or not calibrated:
            status = cls.OBJECT_DETECTION_UNKNOWN
        elif reached_goal:
            status = cls.OBJECT_DETECTION_NO_OBJECT
        elif stalled:
            status = (
                cls.OBJECT_DETECTION_DETECTED_CLOSING
                if goal_position_m < position_m
                else cls.OBJECT_DETECTION_DETECTED_OPENING
            )
        else:
            # Short of the goal and pushing against nothing: the move is still
            # in flight. Nothing is decided until the fingers settle.
            status = cls.OBJECT_DETECTION_UNKNOWN
        return GraspState(reached_goal, stalled, status)

    def grasp_state(self, state=None):
        """Live :class:`GraspState`, measured against the last commanded goal.

        :param state: a snapshot from :meth:`get_state` to reuse. A caller that
            wants both should take one snapshot and pass it, rather than paying
            for a second round of bus reads that would describe a slightly
            different instant.
        """
        if state is None:
            state = self.get_state()
        return self.derive_grasp_state(
            state["position_m"],
            state["effort_N"],
            self._goal_position_m,
            state["calibrated"],
        )

    def get_state(self):
        """Snapshot of gripper state as a plain dict (all reads are live)."""
        percent = self.gripper.get_position()
        load = self.gripper.get_effort()
        return {
            "position_m": self.percent_to_stroke(percent),
            "position_pct": percent,
            "ticks": self.gripper.get_servo_position(),
            "load": load,
            "effort_N": self.percent_to_effort(load),
            "temperature": self.gripper.get_temperature(),
            "calibrated": self.gripper.calibrated,
            "moving": bool(self.gripper.servo.moving_sign),
        }

    # -- commands ---------------------------------------------------------------------
    def set_goal(self, pos, unit="m", effort=None, blocking=True):
        """Command a position.

        :param pos: target position in ``unit``.
        :param unit: ``"m"`` (default), ``"percent"``, or ``"ticks"``.
        :param effort: closing effort in Newtons; ``None`` uses full motor torque.
        :param blocking: when ``True`` (default) run the managed grasp (waits for
            stall/stop, applies holding torque) — use for discrete grasps. When
            ``False`` set the closing torque and write the goal once without
            waiting; for tighter loop-rate control prefer ``write_goal_ticks``.
        """
        percent = self._to_percent(pos, unit)
        closing_torque = (
            self.gripper.MAX_TORQUE
            if effort is None
            else self.effort_to_percent(effort)
        )
        if blocking:
            self._goal_position_m = self.percent_to_stroke(percent)
            return self.gripper.goto_position(
                percent, closing_torque, holding_torque=self._holding_torque
            )
        # Non-blocking: honor exact ticks when the caller gave ticks (the percent
        # round-trip would quantize them to ~39-tick buckets); otherwise convert
        # the target percent to ticks.
        ticks = int(pos) if unit == "ticks" else self._percent_to_ticks(percent)
        self.gripper.set_torque(closing_torque)
        self.write_goal_ticks(ticks)
        return None

    def write_goal_ticks(self, ticks):
        """Fast, non-blocking raw goal write (thin wrapper over ``goal_position``).

        For loop-rate policy control. Assumes torque is already enabled (e.g. by a
        prior ``set_goal`` / ``close``); this only writes the target position.
        """
        ticks = int(ticks)
        self._goal_position_m = self.percent_to_stroke(self._to_percent(ticks, "ticks"))
        self.gripper.servo.goal_position = ticks

    def open(self):
        """Fully open at full torque."""
        self._goal_position_m = self.FINGER_OPEN_POS
        return self.gripper.open()

    def close(self):
        """Fully close at full torque (holding torque applied on stall)."""
        self._goal_position_m = self.FINGER_CLOSED_POS
        return self.gripper.goto_position(
            0, self.gripper.MAX_TORQUE, holding_torque=self._holding_torque
        )

    def release(self):
        """Disable torque so the fingers go slack."""
        # Slack fingers are not on their way anywhere: no goal is outstanding,
        # and wherever they end up says nothing about what they stopped on.
        self._goal_position_m = None
        return self.gripper.release()

    def halt(self):
        """Stop motion by commanding the current position as the goal."""
        # The goal becomes wherever the fingers are, so nothing is outstanding
        # and nothing was stopped short of anything.
        self._goal_position_m = None
        return self.gripper.halt()

    def calibrate(self):
        """Run the homing routine; persist the new position on success."""
        # Homing re-references position, and moves the fingers itself; whatever
        # was commanded beforehand describes nothing that is still true.
        self._goal_position_m = None
        result = self.gripper.calibrate()
        if result:
            self.save_position()
        return result

    # -- unit conversions -------------------------------------------------------------
    def _to_percent(self, pos, unit):
        if unit == "percent":
            return pos
        if unit == "m":
            return self.stroke_to_percent(pos)
        if unit == "ticks":
            # Inverse of _percent_to_ticks; matches Gripper.get_position().
            return 100.0 - Gripper._down_scale(
                pos - Gripper._POSITION_MIN, Gripper._TOTAL_STEPS
            )
        raise ValueError(f"unit must be 'm', 'percent', or 'ticks', not {unit!r}")

    @staticmethod
    def _percent_to_ticks(percent):
        # Matches the servo_position math inside Gripper.goto_position().
        return (
            Gripper._scale(100.0 - percent, Gripper._TOTAL_STEPS)
            + Gripper._POSITION_MIN
        )

    # -- unit maps (moved verbatim from the ROS node so both consumers share them) ----
    @classmethod
    def percent_to_stroke(cls, percent):
        stroke = cls.FINGER_CLOSED_POS + (percent - cls.MIN_PERCENT) * (
            cls.FINGER_OPEN_POS - cls.FINGER_CLOSED_POS
        ) / (cls.MAX_PERCENT - cls.MIN_PERCENT)

        # If the input is within range, but the output is not, it's a computation error,
        # so clamp to min or max. If the input is not, return the raw output.
        if cls.MIN_PERCENT <= percent <= cls.MAX_PERCENT:
            if stroke < cls.FINGER_CLOSED_POS:
                stroke = cls.FINGER_CLOSED_POS
            elif stroke > cls.FINGER_OPEN_POS:
                stroke = cls.FINGER_OPEN_POS

        return stroke

    @classmethod
    def stroke_to_percent(cls, stroke):
        percent = cls.MIN_PERCENT + (stroke - cls.FINGER_CLOSED_POS) * (
            cls.MAX_PERCENT - cls.MIN_PERCENT
        ) / (cls.FINGER_OPEN_POS - cls.FINGER_CLOSED_POS)

        # If the input is within range, but the output is not, it's a computation error,
        # so clamp to min or max. If the input is not, return the raw output.
        if cls.FINGER_CLOSED_POS <= stroke <= cls.FINGER_OPEN_POS:
            if percent < cls.MIN_PERCENT:
                percent = cls.MIN_PERCENT
            elif percent > cls.MAX_PERCENT:
                percent = cls.MAX_PERCENT

        return percent

    @classmethod
    def percent_to_effort(cls, percent):
        effort = cls.MIN_EFFORT + (percent - cls.MIN_PERCENT) * (
            cls.MAX_EFFORT - cls.MIN_EFFORT
        ) / (cls.MAX_PERCENT - cls.MIN_PERCENT)

        # If the input is within range, but the output is not, it's a computation error,
        # so clamp to min or max. If the input is not, return the raw output.
        if cls.MIN_PERCENT <= percent <= cls.MAX_PERCENT:
            if effort < cls.MIN_EFFORT:
                effort = cls.MIN_EFFORT
            elif effort > cls.MAX_EFFORT:
                effort = cls.MAX_EFFORT

        return effort

    @classmethod
    def effort_to_percent(cls, effort):
        percent = cls.MIN_PERCENT + (effort - cls.MIN_EFFORT) * (
            cls.MAX_PERCENT - cls.MIN_PERCENT
        ) / (cls.MAX_EFFORT - cls.MIN_EFFORT)

        # If the input is within range, but the output is not, it's a computation error,
        # so clamp to min or max. If the input is not, return the raw output.
        if cls.MIN_EFFORT <= effort <= cls.MAX_EFFORT:
            if percent < cls.MIN_PERCENT:
                percent = cls.MIN_PERCENT
            elif percent > cls.MAX_PERCENT:
                percent = cls.MAX_PERCENT

        return percent
