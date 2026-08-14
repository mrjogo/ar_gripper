"""Isaac Sim servo backend: subclasses ``mock.FakeServo`` at the byte level.

``IsaacJointBus`` owns the rclpy subscription/publisher pair that talks to
Isaac's ``/isaac/joint_states`` and ``/isaac/gripper/joint_commands`` topics.
``IsaacServo`` is a ``FakeServo`` whose position and load come from Isaac
instead of the internal travel/current model the base class already has: the
real ``FeetechSMSServo`` packet framing, checksums and retry logic all still
run against it unmodified, only the bytes on the wire are sourced from the
simulator. See ``mock.py`` for everything this class inherits (the six
pre-seeded config registers, the sign-magnitude position encoder, the 6.5
mA/LSB current quantisation, the exactly-zero unloaded load, and the rule
that static registers never block).

This module imports ``rclpy`` and is therefore *not* part of the ROS-free
wheel (see ``pyproject.toml``'s ``only-include``); it lives in ``scripts/``,
which is excluded from it, alongside ``ar_gripper.py``.
"""

import threading
from pathlib import Path
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from sensor_msgs.msg import JointState

from ar_gripper.mock import FakeServo

# Feetech register addresses for the two writes this backend needs to see but
# that FakeServo has no constants for (they live as private attributes on a
# different class, FeetechSMSServo._DRIVE_SPEED_ADDR / ._TORQUE_LIMIT_ADDR).
# Module-level so both IsaacJointBus and IsaacServo can use the same values
# without one importing the other's class attributes.
_DRIVE_SPEED_ADDR = 0x2E
_TORQUE_LIMIT_ADDR = 0x30

# 3945 ticks over the 0.05 m stroke = 12.67 um/tick. Module-level for the same
# reason as the two addresses above: IsaacJointBus.set_drive_parameter needs
# it to convert a drive_speed write into metres/s, and reaching into
# IsaacServo.TICKS_PER_METRE for that would recreate exactly the coupling
# those two addresses were pulled out here to avoid.
_TICKS_PER_METRE = 78_900.0

# The description this package owns and every consumer of the finger joint
# shares -- Gazebo, MoveIt, the exported URDF Isaac's stage is built from, and
# the ramp below. The joint is declared with a tf_prefix, so it is matched by
# the unprefixed suffix.
_DESCRIPTION_RELPATH = Path("urdf") / "ar_gripper_macro.xacro"
_FINGER_JOINT_SUFFIX = "ar_gripper_body_finger1"


def finger_velocity_limit_mps(description_path=None):
    """Read the finger joint's ``<limit velocity="...">`` out of the description.

    The ramp in ``IsaacJointBus`` must not publish targets faster than the
    joint's own velocity limit, because that limit is also what Isaac clamps
    the simulated joint to -- publish faster and the target simply runs ahead
    of a joint that then catches up on its own schedule. That makes the limit a
    number two subsystems have to agree on, and the description is the one that
    already exists: copying it into a Python constant here would make this the
    fourth place holding it (the two ``<limit>`` elements, the imported USD's
    ``physxJoint:maxJointVelocity``, and a constant) and the first one to go
    stale, since nothing would notice the disagreement.

    The macro is read as plain XML rather than expanded through xacro: the
    value is deliberately a literal there (see the comment above the joints),
    expansion would pull in a build-time tool at node startup, and a literal
    that stops being a literal should fail loudly here rather than be silently
    re-derived. A non-numeric value therefore raises.
    """
    if description_path is None:
        description_path = (
            Path(get_package_share_directory("ar_gripper")) / _DESCRIPTION_RELPATH
        )
    root = ElementTree.parse(description_path).getroot()
    for joint in root.iter("joint"):
        name = joint.get("name") or ""
        if not name.endswith(_FINGER_JOINT_SUFFIX):
            continue
        limit = joint.find("limit")
        raw = limit.get("velocity") if limit is not None else None
        if raw is None:
            raise ValueError(
                f"{description_path}: joint {name!r} has no <limit velocity=...>"
            )
        try:
            return float(raw)
        except ValueError as exc:
            raise ValueError(
                f"{description_path}: joint {name!r} has a non-literal velocity "
                f"limit {raw!r}. This file is read as plain XML; either keep the "
                "limit a literal or teach this reader to expand xacro."
            ) from exc
    raise ValueError(
        f"{description_path}: no joint named *{_FINGER_JOINT_SUFFIX} to take a "
        "velocity limit from"
    )


class RosClock:
    """``time.time()``/``time.sleep()`` over a ROS clock, for gripper.py's clock seam.

    ``gripper.py`` measures its inrush window, stall baseline and move timeouts
    with the ``time`` module, and ``mock.py`` already establishes the seam for
    replacing it: assigning ``ar_gripper.gripper.time`` to any object exposing
    ``time()`` and ``sleep()``. Against Isaac those durations have to be
    measured in *simulated* seconds -- the simulator runs at a real-time factor
    well below 1.0, so a 0.3 s inrush window measured on the wall clock covers
    only ~0.2 s of the motion the driver is reasoning about, and a 20 s move
    timeout expires part way through a move that has not actually taken 20
    simulated seconds. Wrapping the node's own clock (which is sim time when
    ``use_sim_time`` is set) makes every one of those durations mean what it
    says.

    This is the same seam ``mock.install_ros_node_loopback`` uses and the
    opposite clock: that one installs ``FakeTime``, which advances instantly so
    offline tests never wait. Here the waiting is the point.
    """

    def __init__(self, node):
        self._clock = node.get_clock()

    def time(self):
        return self._clock.now().nanoseconds / 1e9

    def sleep(self, seconds):
        self._clock.sleep_for(Duration(nanoseconds=int(seconds * 1e9)))


class IsaacJointBus:
    """Owns the joint-state subscription and joint-command publisher.

    Creates no node of its own; the caller (``scripts/ar_gripper.py``'s
    ``isaac`` branch) passes one in, along with the executor that will spin it.
    That caller gives it a node dedicated to simulator traffic rather than the
    driver node -- see the comment where it is built for the measurement that
    forced the split -- so this class must not assume it is sharing a node with
    the driver's own timers, services or action.

    Only ``primary_ar_gripper_body_finger1`` is commanded. ``finger2`` is a
    dependent DOF held to it by a real solver constraint (a
    ``PxArticulationMimicJoint``, confirmed in ``barbot_isaac``'s
    ``import_robot_usd.py`` / ``barbot_stage.py`` to follow finger1 to within
    1.0e-4 m with no drive of its own) -- giving it an independent target
    would put a second actuator on the far side of that constraint, which the
    Isaac-side wiring deliberately avoids (``GripperController`` only
    receives finger1's command). It still shows up as its own DOF in
    ``/isaac/joint_states`` (nine total: six arm + two fingers + turntable),
    which is why this bus only ever reads/writes the one joint name it is
    constructed with.
    """

    # Isaac's bridge publishes /isaac/joint_states at 120 Hz (barbot_stage.py);
    # the ramp timer asks for the same rate so the commanded target advances on
    # every tick the simulator can actually observe. Nothing depends on the
    # timer hitting it -- the ramp integrates the interval it measures.
    COMMAND_RATE_HZ = 120.0
    # Ceiling on the interval the ramp will integrate over in one tick, so a
    # clock jump (a paused simulator, a stage reload resetting sim time) moves
    # the target by at most a tenth of a second's travel instead of by
    # however long the clock was away.
    MAX_RAMP_INTERVAL_S = 0.1
    # Measured floor of the sim's real-time factor (mean 0.79, worst 0.68 --
    # see barbot_isaac stage notes). The wall-clock joint_states period is
    # 1 / (publish_hz * RTF); at the RTF floor that is ~12.25 ms. The wait cap
    # must sit comfortably above that or every read silently returns a stale
    # sample -- set to 5x the worst-case period rather than a hardcoded
    # guess, per the concurrency rule this pays for (diagnostics must not
    # stall behind a slow grasp).
    _RTF_FLOOR = 0.68
    DEFAULT_TIMEOUT_S = 5.0 / (COMMAND_RATE_HZ * _RTF_FLOOR)

    def __init__(
        self,
        node,
        joint_states_topic,
        command_topic,
        joint_name,
        max_velocity_mps=None,
    ):
        self._node = node
        self._joint_name = joint_name
        # The ramp's speed ceiling, taken from the description rather than
        # restated here (see finger_velocity_limit_mps). drive_speed is the
        # *driver's* request and at the servo's real max converts to ~1.55 m/s,
        # which would cross the whole 0.05 m stroke in four ticks (33 ms) --
        # inside Gripper._goto_position's 0.3 s inrush window, so the baseline
        # window and the stall window would collapse into the same window
        # before a single intermediate target was ever published. The argument
        # exists so tests can pin a limit without a description on disk.
        self.max_velocity_mps = (
            finger_velocity_limit_mps()
            if max_velocity_mps is None
            else max_velocity_mps
        )

        # Guards every field below; wait_for_sample() blocks on it and
        # _on_joint_states()/command()/set_drive_parameter() notify/mutate
        # under it, so a live read and an incoming Isaac sample never race.
        self._condition = threading.Condition()
        self._seq = 0
        self._position_m = 0.0
        self._velocity_mps = 0.0
        self._effort_n = 0.0
        self.timeout_count = 0

        # Seeded from the first measured sample, not from zero: zero is a real
        # position (the fully CLOSED stop), so a bus that starts out believing
        # the goal is 0.0 commands a full close the instant the node comes up,
        # wherever the finger actually is -- observed slamming a finger from
        # 0.019 m to 0.000 m on a driver restart with no goal ever sent. Until
        # the first sample arrives there is no defensible target to publish, so
        # the ramp publishes nothing.
        self._seeded = False
        self._goal_m = 0.0
        self._published_m = 0.0
        self._last_tick_s = None
        # Steps/s from the last GOAL_SPEED (0x2E) write, converted to m/s.
        # Zero (the default) means "no drive_speed configured yet" -- until
        # the driver writes one, the ramp jumps straight to the goal, same as
        # FakeServo's own default travel model.
        self._drive_speed_mps = 0.0
        # Raw 0.1%/LSB register word from the last TORQUE_LIMIT (0x30) write.
        # No Isaac analogue yet -- recorded for a future finger-drive
        # maxForce scale, not acted on.
        self.torque_limit_raw = None

        # A private callback group each, so that nothing here can be starved
        # by, or starve, a callback that blocks. What actually keeps this bus
        # responsive is the executor it is given: ar_gripper.py puts the node
        # it passes in on a SingleThreadedExecutor of its own, because rclpy's
        # MultiThreadedExecutor cannot carry a subscription at Isaac's rate
        # (measured: 53 of 120 Hz delivered for a whole core, with gaps up to
        # 700 ms, against 118 Hz for 0.13 of a core with a worst gap of 9 ms).
        # These groups are what keeps that placement from being the *only*
        # thing standing between a blocking read and a stall: a read blocks
        # for up to DEFAULT_TIMEOUT_S waiting on a fresh sample, and a
        # MutuallyExclusiveCallbackGroup admits one callback at a time, so a
        # subscription sharing a group with any blocking caller could only
        # ever deliver the sample after that caller gave up.
        self._subscription_group = MutuallyExclusiveCallbackGroup()
        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._command_pub = node.create_publisher(JointState, command_topic, 10)
        node.create_subscription(
            JointState,
            joint_states_topic,
            self._on_joint_states,
            10,
            callback_group=self._subscription_group,
        )
        self._ramp_timer = node.create_timer(
            1.0 / self.COMMAND_RATE_HZ,
            self._publish_ramped_target,
            callback_group=self._timer_group,
        )

    def _on_joint_states(self, msg):
        try:
            index = msg.name.index(self._joint_name)
        except ValueError:
            return
        position = msg.position[index] if index < len(msg.position) else 0.0
        velocity = msg.velocity[index] if index < len(msg.velocity) else 0.0
        effort = msg.effort[index] if index < len(msg.effort) else 0.0
        with self._condition:
            self._position_m = position
            self._velocity_mps = velocity
            self._effort_n = effort
            if not self._seeded:
                self._goal_m = position
                self._published_m = position
                self._seeded = True
            self._seq += 1
            self._condition.notify_all()

    def wait_for_sample(self, timeout_s=None):
        """Block for a **new** ``/isaac/joint_states`` sample and return it.

        Returns ``(position_m, velocity_mps, effort_N)``. On timeout, returns
        the last known sample (never blocks forever) and bumps
        ``timeout_count`` with a throttled warning -- a nonzero count during a
        grasp means the wait cap or the publish rate needs revisiting.
        """
        if timeout_s is None:
            timeout_s = self.DEFAULT_TIMEOUT_S
        with self._condition:
            seen = self._seq
            fresh = self._condition.wait_for(
                lambda: self._seq != seen, timeout=timeout_s
            )
            if not fresh:
                self.timeout_count += 1
                self._node.get_logger().warning(
                    "IsaacJointBus: timed out waiting for a fresh "
                    f"/isaac/joint_states sample for {self._joint_name!r} "
                    f"(timeout_count={self.timeout_count})",
                    throttle_duration_sec=5.0,
                )
            return self._position_m, self._velocity_mps, self._effort_n

    def command(self, position_m):
        """Set the goal the ramp in ``_publish_ramped_target`` drives toward."""
        with self._condition:
            self._goal_m = position_m

    def set_drive_parameter(self, addr, value):
        """Record a drive_speed (0x2E) or torque_limit (0x30) write.

        drive_speed paces the position ramp below. torque_limit has no Isaac
        analogue yet -- recorded on ``torque_limit_raw`` for a future
        finger-drive maxForce scale, not acted on.
        """
        if addr == _DRIVE_SPEED_ADDR:
            # The wire encodes steps/s // 50 (FeetechSMSServo.drive_speed's
            # setter divides by 50 before writing, and its getter multiplies
            # back -- see feetech.py); `value` here is that raw register
            # word, not steps/s, so it must be scaled back up before it means
            # anything in metres/s.
            with self._condition:
                self._drive_speed_mps = (value * 50) / _TICKS_PER_METRE
        elif addr == _TORQUE_LIMIT_ADDR:
            with self._condition:
                self.torque_limit_raw = value

    def _publish_ramped_target(self):
        # Ramp the published target toward the goal at drive_speed rather
        # than jumping straight to it. Without this the position drive
        # reaches the goal in a couple of physics steps, the finger is
        # already at contact before the current baseline window opens, and
        # stall detection reads a contact baseline -- the Isaac analogue of
        # mock.FakeServo's travel_ticks_per_read, and it exists for the same
        # reason.
        #
        # Clamped to the description's own velocity limit (max_velocity_mps):
        # drive_speed is the *driver's* request, and at the servo's real max it
        # is far faster than the joint's <limit .../> velocity Isaac will
        # actually track -- publishing the unclamped rate just runs the target
        # ahead of the joint and leaves it to catch up on its own schedule
        # regardless.
        with self._condition:
            if not self._seeded:
                return
            goal = self._goal_m
            rate = min(self._drive_speed_mps, self.max_velocity_mps)
        # The step is rate x MEASURED elapsed time, not rate / COMMAND_RATE_HZ.
        # A fixed step per tick only travels at `rate` if the timer actually
        # achieves COMMAND_RATE_HZ, and this one runs on the simulator's clock:
        # a sim-time timer fires when the clock advances past its deadline, so
        # its rate is the simulator's to decide, not ours. It has been measured
        # at 95.7 Hz against a nominal 120, which as a fixed step is a silent
        # 20% speed error. Integrating the measured interval instead makes the
        # ramp travel at `rate` whatever the tick rate turns out to be:
        # measured against a live stage after this change, the commanded target
        # moves at 0.0310 m/s against a 0.031 m/s limit.
        now = self._node.get_clock().now()
        now_s = now.nanoseconds / 1e9
        elapsed_s = now_s - self._last_tick_s if self._last_tick_s is not None else 0.0
        self._last_tick_s = now_s
        # Capped, because the clock can jump: sim time restarts at zero on a
        # stage reload and stalls while the simulator is paused, and an
        # uncapped interval would turn either into a single step that
        # teleports the target across the whole stroke.
        elapsed_s = min(max(elapsed_s, 0.0), self.MAX_RAMP_INTERVAL_S)
        if rate <= 0.0:
            self._published_m = goal
        else:
            step = rate * elapsed_s
            delta = goal - self._published_m
            self._published_m += (
                delta if abs(delta) <= step else (step if delta > 0 else -step)
            )
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name.append(self._joint_name)
        msg.position.append(self._published_m)
        self._command_pub.publish(msg)


class IsaacServo(FakeServo):
    """A FakeServo whose position and load come from Isaac instead of the internal model.

    Everything about the *wire* -- register addresses, sign-magnitude position
    encoding, the 6.5 mA/LSB current quantisation, the exactly-zero unloaded
    load, and the rule that static registers never block -- is inherited.
    Only the source of the numbers changes, so the offline tests over the
    base class cover the encoding for this class too.
    """

    # No FakeServo constants exist for these two; they are private on a
    # different class (FeetechSMSServo._DRIVE_SPEED_ADDR / ._TORQUE_LIMIT_ADDR).
    # Mirrored here (same values as the module-level constants above) so an
    # upstream rename is greppable.
    DRIVE_SPEED = _DRIVE_SPEED_ADDR
    TORQUE_LIMIT = _TORQUE_LIMIT_ADDR

    TICKS_PER_METRE = _TICKS_PER_METRE  # mirrors the module constant above
    # tick 4095 == 0.0 m == CLOSED; tick 150 == 0.05 m == OPEN. Ticks go DOWN
    # as metres go UP -- a negative slope, agreed by three
    # independent sources: Gripper._POSITION_MAX (4095, "gripper closed") /
    # _POSITION_MIN (150, "gripper open") in gripper.py, FINGER_CLOSED_POS
    # (0.0) / FINGER_OPEN_POS (0.05) in standalone.py, and the finger link's
    # close/open groups in the description macro. A first draft of this file
    # had the slope inverted; live-verified against a running simulator that
    # commanding "closed" (goal_position = CLOSED_POSITION_TICKS) now drives
    # the finger toward 0.0 m instead of toward the open stop.
    CLOSED_POSITION_TICKS = 4095  # == Gripper._POSITION_MAX
    MAX_EFFORT_N = 1041.25  # matches ARGripperStandalone.MAX_EFFORT (100% torque)
    LOAD_DEADBAND_N = 2.0  # below this the joint counts as unloaded
    MOVING_DEADBAND_MPS = 1.0e-4  # below this, moving_sign reads 0 (stopped)

    def __init__(self, bus):
        # mock.FakeServo's internal travel/obstruction model stays off: Isaac
        # is the only source of motion here, and a second model competing
        # with it would fight the real one.
        super().__init__(travel_ticks_per_read=0, obstruction_ticks=None)
        self._bus = bus
        self._tick_offset = 0.0
        self._sample = None

    # -- the one seam: every live read pulls a fresh Isaac sample --------------------
    def _advance(self):
        self._sample = self._bus.wait_for_sample()
        position_m, _velocity, _effort = self._sample
        self.present_position_value = round(
            self.CLOSED_POSITION_TICKS
            - position_m * self.TICKS_PER_METRE
            + self._tick_offset
        )

    def _contact_load(self):
        """Percent of stall torque, exactly 0 when unloaded.

        Feeds the inherited current model as well as present_load, so the two
        cannot disagree (mock.FakeServo._current_ma() calls this directly,
        not _load_now()).
        """
        if self._sample is None:
            return 0.0
        force_n = abs(self._sample[2])
        if force_n < self.LOAD_DEADBAND_N:
            return 0.0
        return min(force_n / self.MAX_EFFORT_N * 100.0, 100.0)

    def _load_now(self):
        return self._contact_load()

    def _moving_now(self):
        if self._sample is None:
            return 0
        return 1 if abs(self._sample[1]) > self.MOVING_DEADBAND_MPS else 0

    def write(self, addr, data):
        if addr == self.GOAL_POSITION and len(data) >= 2:
            ticks = data[0] | (data[1] & 0x7F) << 8
            if data[1] & 0x80:
                ticks *= -1
            # Inverse of _advance()'s decode (negative slope: see
            # CLOSED_POSITION_TICKS), solved for metres given a target tick.
            self._bus.command(
                (self.CLOSED_POSITION_TICKS + self._tick_offset - ticks)
                / self.TICKS_PER_METRE
            )
        elif (
            addr == self.TORQUE_SWITCH
            and len(data) == 1
            and data[0] == self.RESET_MIDPOINT
        ):
            # Encoder rezero with no Isaac analogue: shift the tick offset so
            # the measured position reads 2048 from here on, rather than
            # teleporting the joint. Without it the tick<->metre map drifts
            # and MoveIt reports the wrong finger position. _calibrate() does
            # this three times. Solved from the same negative-slope decode as
            # _advance(), set equal to 2048 at the current sample's position.
            if self._sample is not None:
                self._tick_offset = (
                    2048
                    - self.CLOSED_POSITION_TICKS
                    + self._sample[0] * self.TICKS_PER_METRE
                )
        elif addr in (self.DRIVE_SPEED, self.TORQUE_LIMIT):
            value = (data[0] | data[1] << 8) if len(data) >= 2 else data[0]
            self._bus.set_drive_parameter(addr, value)
        super().write(addr, data)
