"""Isaac Sim servo backend: subclasses ``mock.FakeServo`` at the byte level.

``IsaacJointBus`` owns the rclpy subscription/publisher pair that talks to
Isaac's ``/isaac/joint_states`` and ``/isaac/gripper/joint_commands`` topics.
``IsaacServo`` is a ``FakeServo`` whose position and load come from Isaac
instead of the internal travel/current model Task 9 added: the real
``FeetechSMSServo`` packet framing, checksums and retry logic all still run
against it unmodified, only the bytes on the wire are sourced from the
simulator. See ``mock.py`` for everything this class inherits (the six
pre-seeded config registers, the sign-magnitude position encoder, the 6.5
mA/LSB current quantisation, the exactly-zero unloaded load, and the rule
that static registers never block).

This module imports ``rclpy`` and is therefore *not* part of the ROS-free
wheel (see ``pyproject.toml``'s ``only-include``); it lives in ``scripts/``,
which is excluded from it, alongside ``ar_gripper.py``.
"""

import threading

from sensor_msgs.msg import JointState

from ar_gripper.mock import FakeServo

# Feetech register addresses for the two writes this backend needs to see but
# that FakeServo has no constants for (they live as private attributes on a
# different class, FeetechSMSServo._DRIVE_SPEED_ADDR / ._TORQUE_LIMIT_ADDR).
# Module-level so both IsaacJointBus and IsaacServo can use the same values
# without one importing the other's class attributes.
_DRIVE_SPEED_ADDR = 0x2E
_TORQUE_LIMIT_ADDR = 0x30


class IsaacJointBus:
    """Owns the joint-state subscription and joint-command publisher.

    Creates no node of its own; the caller (``scripts/ar_gripper.py``'s
    ``isaac`` branch) passes the ``ARGripperNode`` itself, the same pattern
    ``IsaacTurntable`` uses for the turntable.

    Only ``primary_ar_gripper_body_finger1`` is commanded. ``finger2`` is a
    dependent DOF held to it by a real solver constraint (a
    ``PxArticulationMimicJoint``, confirmed in ``barbot_isaac``'s
    ``import_robot_usd.py`` / ``barbot_stage.py`` to follow finger1 to within
    2.6e-4 m with no drive of its own) -- giving it an independent target
    would put a second actuator on the far side of that constraint, which the
    Isaac-side wiring deliberately avoids (``GripperController`` only
    receives finger1's command). It still shows up as its own DOF in
    ``/isaac/joint_states`` (nine total: six arm + two fingers + turntable),
    which is why this bus only ever reads/writes the one joint name it is
    constructed with.
    """

    # Isaac's bridge publishes /isaac/joint_states at 120 Hz (barbot_stage.py);
    # the ramp timer matches that so the commanded target advances on every
    # tick the simulator can actually observe.
    COMMAND_RATE_HZ = 120.0
    # Measured floor of the sim's real-time factor (mean 0.79, worst 0.68 --
    # see barbot_isaac stage notes). The wall-clock joint_states period is
    # 1 / (publish_hz * RTF); at the RTF floor that is ~12.25 ms. The wait cap
    # must sit comfortably above that or every read silently returns a stale
    # sample -- set to 5x the worst-case period rather than a hardcoded
    # guess, per the concurrency rule this pays for (diagnostics must not
    # stall behind a slow grasp).
    _RTF_FLOOR = 0.68
    DEFAULT_TIMEOUT_S = 5.0 / (COMMAND_RATE_HZ * _RTF_FLOOR)

    def __init__(self, node, joint_states_topic, command_topic, joint_name):
        self._node = node
        self._joint_name = joint_name

        # Guards every field below; wait_for_sample() blocks on it and
        # _on_joint_states()/command()/set_drive_parameter() notify/mutate
        # under it, so a live read and an incoming Isaac sample never race.
        self._condition = threading.Condition()
        self._seq = 0
        self._position_m = 0.0
        self._velocity_mps = 0.0
        self._effort_n = 0.0
        self.timeout_count = 0

        self._goal_m = 0.0
        self._published_m = 0.0
        # Steps/s from the last GOAL_SPEED (0x2E) write, converted to m/s.
        # Zero (the default) means "no drive_speed configured yet" -- until
        # the driver writes one, the ramp jumps straight to the goal, same as
        # FakeServo's own default travel model.
        self._drive_speed_mps = 0.0

        self._command_pub = node.create_publisher(JointState, command_topic, 10)
        node.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, 10
        )
        self._ramp_timer = node.create_timer(
            1.0 / self.COMMAND_RATE_HZ, self._publish_ramped_target
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
        analogue yet (recorded for a future finger-drive maxForce scale, not
        acted on) -- see Task 11.
        """
        if addr == _DRIVE_SPEED_ADDR:
            # The wire encodes steps/s // 50 (FeetechSMSServo.drive_speed's
            # setter divides by 50 before writing, and its getter multiplies
            # back -- see feetech.py); `value` here is that raw register
            # word, not steps/s, so it must be scaled back up before it means
            # anything in metres/s.
            with self._condition:
                self._drive_speed_mps = (value * 50) / IsaacServo.TICKS_PER_METRE
        # TORQUE_LIMIT: nothing to record into yet.

    def _publish_ramped_target(self):
        # Ramp the published target toward the goal at drive_speed rather
        # than jumping straight to it. Without this the position drive
        # reaches the goal in a couple of physics steps, the finger is
        # already at contact before the current baseline window opens, and
        # stall detection reads a contact baseline -- the Isaac analogue of
        # Task 9's travel_ticks_per_read, and it exists for the same reason.
        with self._condition:
            goal = self._goal_m
            rate = self._drive_speed_mps
        if rate <= 0.0:
            self._published_m = goal
        else:
            step = rate / self.COMMAND_RATE_HZ
            delta = goal - self._published_m
            self._published_m += (
                delta if abs(delta) <= step else (step if delta > 0 else -step)
            )
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
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

    TICKS_PER_METRE = 78_900.0  # 3945 ticks over the 0.05 m stroke = 12.67 um/tick
    MAX_EFFORT_N = 1041.25  # matches ARGripperStandalone.MAX_EFFORT (100% torque)
    LOAD_DEADBAND_N = 2.0  # below this the joint counts as unloaded
    MOVING_DEADBAND_MPS = 1.0e-4  # below this, moving_sign reads 0 (stopped)

    def __init__(self, bus):
        # Task 9's internal travel/obstruction model stays off: Isaac is the
        # only source of motion here, and a second model competing with it
        # would fight the real one.
        super().__init__(travel_ticks_per_read=0, obstruction_ticks=None)
        self._bus = bus
        self._tick_offset = 0.0
        self._sample = None

    # -- the one seam: every live read pulls a fresh Isaac sample --------------------
    def _advance(self):
        self._sample = self._bus.wait_for_sample()
        position_m, _velocity, _effort = self._sample
        self.present_position_value = round(
            position_m * self.TICKS_PER_METRE + self._tick_offset
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
            self._bus.command((ticks - self._tick_offset) / self.TICKS_PER_METRE)
        elif (
            addr == self.TORQUE_SWITCH
            and len(data) == 1
            and data[0] == self.RESET_MIDPOINT
        ):
            # Encoder rezero with no Isaac analogue: shift the tick offset so
            # the measured position reads 2048 from here on, rather than
            # teleporting the joint. Without it the tick<->metre map drifts
            # and MoveIt reports the wrong finger position. _calibrate() does
            # this three times.
            if self._sample is not None:
                self._tick_offset = 2048 - self._sample[0] * self.TICKS_PER_METRE
        elif addr in (self.DRIVE_SPEED, self.TORQUE_LIMIT):
            value = (data[0] | data[1] << 8) if len(data) >= 2 else data[0]
            self._bus.set_drive_parameter(addr, value)
        super().write(addr, data)
