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

import inspect
import threading
from time import monotonic as _real_monotonic
from time import sleep as _real_sleep

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from sensor_msgs.msg import JointState

from ar_gripper.description import finger_velocity_limit_mps
from ar_gripper.gripper import Deadline, Gripper
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

# -- contact detection: the three numbers, and the budget they have to fit -------
#
# The speed below which the servo's moving_sign register reads 0 -- the
# driver's own definition of "stopped". The stall detector below asks the same
# question with the same number, deliberately: Gripper._wait_for_stop will not
# return until the joint is under this, so a stall can only ever be reported
# about a joint the driver already agrees has stopped, and the two cannot
# disagree about what stopped means. (Also why no separate, larger stall
# velocity is defined: one at, say, a tick per sample would sit only twice
# under the finger's whole velocity limit, so a regenerated asset carrying a
# stale limit could put a finger at full speed two ticks from "not moving".)
MOVING_DEADBAND_MPS = 1.0e-4

# How far from its target a *stopped* joint has to be before that gap counts as
# something holding it back. Bounded from both sides, and neither bound needs
# the simulated drive's gains restated here:
#
#   Below: a joint that is still settling toward a reachable target carries a
#   lag of velocity x the drive's time constant, so at MOVING_DEADBAND_MPS it
#   is tens of microns from its target -- 62 um at the 0.62 s the stage reports
#   for the finger drive, and still only 125 um if that doubled. This sits 8x
#   above the pessimistic figure, so no particular time constant is load
#   bearing.
#
#   Above: Gripper._calibrate commands exactly 2047 ticks (25.9 mm) of closing
#   per homing attempt, measured from wherever the finger already is, so that
#   is the smallest overshoot past a hard stop that homing is *guaranteed* to
#   produce. This sits 26x below it.
STALL_POSITION_ERROR_M = 1.0e-3


def _wait_for_stop_reads():
    """How many settled reads ``Gripper._wait_for_stop`` wants before returning.

    Read off the driver rather than restated, because it sets the deadline the
    detector below has to meet and a silent disagreement would show up only as
    homing failing against a simulator.
    """
    parameter = inspect.signature(Gripper._wait_for_stop).parameters.get("stop_delay")
    if parameter is None or parameter.default is inspect.Parameter.empty:
        raise RuntimeError(
            "Gripper._wait_for_stop no longer takes a stop_delay default; the "
            "simulated servo's stall dwell is derived from it (it must fit "
            "inside the window that many settled reads leave) and now has "
            "nothing to derive from"
        )
    return parameter.default


# The window this has to fit inside. Once the finger stops moving,
# Gripper._wait_for_stop returns after this long, and the homing loop reads
# present_load the moment it does -- so a stall that takes longer than this to
# report is a stall homing never sees, and it burns one of its three attempts.
# It is the binding resource in the whole design, which is why the detector
# arms on the target being paid out rather than on the error that paying it out
# eventually produces: waiting for the error would spend a slice of this window
# that varies with wherever the previous attempt happened to stop.
WAIT_FOR_STOP_WINDOW_S = Gripper._WAIT_CHECK_TIME_S * _wait_for_stop_reads()
# Two thirds of it spent, a third held back for the reads themselves and for
# the sample that starts the dwell arriving up to one publish period late.
STALL_DWELL_S = 0.6 * WAIT_FOR_STOP_WINDOW_S


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

    # How often the sleep below checks the simulated clock. Short relative to
    # the shortest wait in gripper.py (its 5 ms inrush poll) so the sleep does
    # not become the thing that sets the loop rate.
    POLL_INTERVAL_S = 0.002

    def sleep(self, seconds):
        """Sleep ``seconds`` of simulated time, bounded in real time.

        ``rclpy.clock.Clock.sleep_for`` is the obvious call and it is wrong
        here, for a sharper version of the reason ``Deadline`` carries a
        backstop: against a stopped simulator it never returns. A deadline at
        least gets to notice it has expired; a thread blocked in
        ``sleep_for`` cannot, so the wait built out of it hangs forever and
        takes the action's lock with it. Polling the simulated clock with real
        sleeps keeps the duration simulated while keeping the call bounded in
        real time, using the same backstop factor the deadlines use.
        """
        target = self._clock.now() + Duration(nanoseconds=int(seconds * 1e9))
        wall_expiry = _real_monotonic() + seconds * Deadline.WALL_BACKSTOP_FACTOR
        while self._clock.now() < target:
            if _real_monotonic() >= wall_expiry:
                return
            _real_sleep(min(self.POLL_INTERVAL_S, seconds))


class StallDetector:
    """Contact detection from motion, for a joint whose effort cannot show it.

    Isaac's measured joint effort is the joint's reaction to *external* loads,
    and a finger pressed against its own joint limit has none: the drive force
    and the limit's reaction are internal to that joint and cancel exactly. It
    measures flat at +0.0003 N over 5486 samples, identical free-running and
    pinned, and still 0.0004 N with 0.45 m of commanded drive error (~23 N at
    the finger's 52 N/m drive). The reading is not missing and there is no
    switch that turns it on -- force is simply the wrong question to ask of a
    joint limit. Contact reporting cannot answer it either, because a limit is
    a constraint rather than a collision.

    So contact is detected the way NVIDIA's own ``snap_to_limits`` sample
    detects a joint that has run into something -- from motion rather than
    force, out of the joint state that is published anyway. The joint is
    stalled when it has stopped (by the driver's own ``moving_sign``
    definition of stopped, ``velocity_threshold_mps``) while it is *still being
    commanded somewhere else*, and stays that way for ``dwell_s``.

    "Still being commanded somewhere else" has two shapes, and the detector has
    to arm on either, because they are what the same contact looks like at
    different moments:

    * **Outrun.** The target is being paid out away from the joint faster than
      the joint counts as moving, and the joint is not following. This is a
      finger that was *already* against something when the move began, and it
      arms on the first sample of the move -- it needs no error to have
      accumulated yet, which is the point. Waiting for accumulated error here
      is what makes homing's budget depend on where the previous attempt
      happened to stop (see ``WAIT_FOR_STOP_WINDOW_S``).
    * **Pressing.** The target has finished being paid out and sits more than
      ``position_error_m`` beyond the joint, which is a finger that arrived at
      the stop after the ramp had already reached its goal.

    The failure mode both are sized against is a finger that is merely *slow*
    because it is still settling into a reachable target -- the settle tail is
    around 70% of a stroke, so slow is the normal case and not evidence of
    anything. Neither shape can be produced by one. The drive is heavily
    overdamped, so its approach to a reachable target is first order and its
    lag is exactly velocity x the drive's time constant: at
    ``velocity_threshold_mps`` that is tens of microns, far inside
    ``position_error_m`` (pressing cannot arm), and its target is by definition
    stationary (outrun cannot arm).

    What the dwell is left doing is rejecting transients: a single sample that
    lands under one threshold and over the other, or the moment at the start of
    a move where the target is already moving and the joint has not yet picked
    up, must not latch a stall.

    Honest about what this is: emulation, not physics. Homing against a
    simulator exercises the driver's *sequencing*, not its stall detection, so
    a stall bug that only appears against real servo current will not be caught
    here. The reason to run homing in simulation anyway is that ``_calibrate()``
    is also what establishes the position limits, the position correction and
    the calibration torque limit that the rest of the driver reads; skipping it
    means hand-setting all four, which is a second implementation of the
    driver's own state setup, free to drift away from the real one.
    """

    def __init__(self, velocity_threshold_mps, position_error_m, dwell_s):
        self.velocity_threshold_mps = velocity_threshold_mps
        self.position_error_m = position_error_m
        self.dwell_s = dwell_s
        self._previous_target_m = None
        self._previous_time_s = None
        self._since_s = None
        self.stalled = False

    def update(self, target_m, position_m, velocity_mps, now_s):
        """Fold one sample in and return whether the joint is stalled."""
        error_m = target_m - position_m
        target_velocity_mps = 0.0
        if self._previous_target_m is not None and now_s > self._previous_time_s:
            target_velocity_mps = (target_m - self._previous_target_m) / (
                now_s - self._previous_time_s
            )
        self._previous_target_m = target_m
        self._previous_time_s = now_s

        stopped = abs(velocity_mps) < self.velocity_threshold_mps
        pressing = abs(error_m) > self.position_error_m
        # Signs, not magnitudes: a target moving *toward* the joint is a move
        # arriving, not a joint being left behind, and only the away direction
        # says anything about contact.
        outrun = (
            abs(target_velocity_mps) > self.velocity_threshold_mps
            and target_velocity_mps * error_m > 0.0
        )
        if not (stopped and (pressing or outrun)):
            self._since_s = None
            self.stalled = False
            return self.stalled
        # A clock that jumps backwards restarts the dwell rather than
        # completing it instantly: sim time restarts at zero on a stage reload,
        # and "stalled" is not the thing to report about a reloaded stage.
        if self._since_s is None or now_s < self._since_s:
            self._since_s = now_s
        self.stalled = (now_s - self._since_s) >= self.dwell_s
        return self.stalled


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
        # Contact detection, from the commanded target and the measured joint
        # state -- the effort channel cannot see a finger against its own limit
        # at all. Updated on every incoming sample rather than on the driver's
        # reads, so the dwell is measured over the rate Isaac actually
        # publishes at instead of over however often the driver happens to look.
        self._stall = StallDetector(
            velocity_threshold_mps=MOVING_DEADBAND_MPS,
            position_error_m=STALL_POSITION_ERROR_M,
            dwell_s=STALL_DWELL_S,
        )

        # Raw 0.1%/LSB register word from the last TORQUE_LIMIT (0x30) write.
        # Recorded, not acted on. See set_drive_parameter for why mapping it to
        # the finger drive's maxForce would currently change nothing.
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
            # Against the *published* target, not the goal: the published one
            # is what Isaac's drive is tracking, and the goal can be a whole
            # stroke ahead of it while the ramp is still paying it out, which
            # would read as a stall on every long move.
            self._stall.update(
                self._published_m,
                position,
                velocity,
                self._node.get_clock().now().nanoseconds / 1e9,
            )
            self._seq += 1
            self._condition.notify_all()

    def wait_for_first_sample(self, timeout_s):
        """Block until the simulator has published one joint state. True if it did.

        Startup only, and nothing like ``wait_for_sample``: this one answers
        "is there a simulator on the other end at all", so it does not count a
        timeout against the bus and it is bounded in *real* time -- a
        simulator that has not published yet has not published a clock either,
        so there is no simulated time to bound it in.
        """
        with self._condition:
            return self._condition.wait_for(lambda: self._seeded, timeout=timeout_s)

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

    @property
    def stalled(self):
        """True while the finger is commanded somewhere it is not going.

        The one thing standing in for the servo's load register during homing:
        see ``StallDetector`` for why the simulator's effort channel cannot
        answer that question about a joint limit.
        """
        with self._condition:
            return self._stall.stalled

    def command(self, position_m):
        """Set the goal the ramp in ``_publish_ramped_target`` drives toward."""
        with self._condition:
            self._goal_m = position_m

    def set_drive_parameter(self, addr, value):
        """Record a drive_speed (0x2E) or torque_limit (0x30) write.

        drive_speed paces the position ramp below. torque_limit is recorded on
        ``torque_limit_raw`` and not acted on.

        Its faithful counterpart would be the finger drive's maxForce -- a
        Feetech torque_limit is a ceiling on a position loop, and so is maxForce
        -- but that mapping **cannot bind at present force levels, so it would
        change nothing observable**. Gripper.HOLDING_TORQUE is 10% of
        MAX_EFFORT_N, a 104 N ceiling, while the drive's maximum possible force
        is its stiffness times the full stroke: 466.56 N/m x 0.05 m = 23 N. The
        clip is 4.5x out of reach, so holding torque and the overload relax
        stay invisible either way until grip force approaches ~100 N.

        It is also not a small change when it does become worth doing: the only
        driver-to-simulator channels are the three JointState command topics, so
        it needs a new bridge subscriber plus drive-write code here and in
        barbot_stage.py.
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
        # _published_m is read and written under the same lock as everything
        # else it is derived from. Today both writers happen to be on the one
        # executor thread this bus is given, but the class does not get to
        # assume that -- its own docstring says the caller chooses the node and
        # the executor -- and an unlocked read-modify-write of the published
        # target against a locked read of the goal is the kind of thing that
        # only starts being wrong once someone changes that choice.
        with self._condition:
            if not self._seeded:
                return
            goal = self._goal_m
            rate = min(self._drive_speed_mps, self.max_velocity_mps)
            if rate <= 0.0:
                self._published_m = goal
            else:
                step = rate * elapsed_s
                delta = goal - self._published_m
                self._published_m += (
                    delta if abs(delta) <= step else (step if delta > 0 else -step)
                )
            published = self._published_m
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name.append(self._joint_name)
        msg.position.append(published)
        self._command_pub.publish(msg)


class IsaacServo(FakeServo):
    """A FakeServo whose position and load come from Isaac instead of the internal model.

    Everything about the *wire* -- register addresses, sign-magnitude position
    encoding, the 6.5 mA/LSB current quantisation, the exactly-zero unloaded
    load, and the rule that static registers never block -- is inherited.
    Only the source of the numbers changes, so the offline tests over the
    base class cover the encoding for this class too.

    Position comes from the measured joint state directly. Load has two
    sources, because the simulator can only see one of the two kinds of
    contact as force -- see ``_contact_load``.
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
    # Below this the joint counts as unloaded. An earlier version of this
    # comment recorded the force limb of _contact_load as UNREACHABLE, because
    # the finger drive was 51.84 N/m and could not beat 1.30 N on this joint
    # against the 2.0 N threshold; a grasp then read -0.835 N and fell through
    # to the stall limb every time. That is no longer true. The drive's
    # stability ceiling was re-measured and the stiffness went up 9x to
    # 466.56 N/m, which is exactly the condition this comment said to re-derive
    # on, so: a grasp on the same 40 mm rigid box now reads 4.76 N on this joint
    # (9.53 N summed across the two contacts), and the force limb is reachable
    # for the first time. Those are the forces on the asset as it now ships; an
    # earlier draft quoted 7.32 N / 14.63 N, which were measured on the fingers'
    # previous convex-hull collider. That collider contacted 11 mm of travel
    # early and so developed more force at more position error -- and gripped
    # worse for it.
    #
    # Left at 2.0 N anyway, for now, because what it changes is still small: the
    # limbs are combined with max(), so a genuine stall still reports the larger
    # stall_load, and 4.76 N is 0.46% of stall torque against homing's contact
    # test at 10% and the overload test at 30%. Nothing acts differently at
    # 0.46% than it did at 0.08%.
    #
    # It is not a noise-rejection threshold -- the measured noise floor is
    # 3.3e-4 N, so this sits 6000x above it, a number inherited from a servo
    # whose stall torque really is on this scale. Now that grasp force reaches
    # the driver, the honest derivation is from that noise floor and the grip
    # forces actually reachable, not from the servo's datasheet; that is
    # recorded in barbot_isaac/FUTURE_WORK.md rather than done here, because
    # changing it changes what every grasp reports and wants its own
    # measurement of what the consumers should see.
    LOAD_DEADBAND_N = 2.0
    # Below this, moving_sign reads 0 (stopped) -- the module-scope constant,
    # which the stall detector is built on too, so nothing here and nothing
    # there can drift apart about what stopped means.
    MOVING_DEADBAND_MPS = MOVING_DEADBAND_MPS

    def __init__(self, bus):
        # mock.FakeServo's internal travel/obstruction model stays off: Isaac
        # is the only source of motion here, and a second model competing
        # with it would fight the real one.
        super().__init__(travel_ticks_per_read=0, obstruction_ticks=None)
        self._bus = bus
        self._tick_offset = 0.0
        self._sample = None
        self._stalled = False

    # -- the one seam: every live read pulls a fresh Isaac sample --------------------
    def _advance(self):
        self._sample = self._bus.wait_for_sample()
        self._stalled = self._bus.stalled
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
        not _load_now()). It is also the single hook the driver's two
        load-shaped questions come through -- ``present_load``, which homing
        reads, and ``present_current``, which the grasp loop reads -- which is
        why the stall below is folded in here rather than at either one of
        them.

        Two sources, because the simulator reports the two kinds of contact
        through different channels. An object squeezed between the fingers is
        an external load and shows up in the measured effort; the finger's own
        travel limit is not, and shows up only as a target the joint is not
        reaching (see ``StallDetector``). Whichever reads higher wins, so
        neither kind of contact can be masked by the other being absent.

        The stall reports the same percentage the offline mock's obstruction
        model does (``FakeServo.stall_load``), so "pressed against a hard stop"
        means one number across both backends rather than two that have to be
        kept in step.
        """
        if self._sample is None:
            return 0.0
        force_n = abs(self._sample[2])
        load = (
            0.0
            if force_n < self.LOAD_DEADBAND_N
            else min(force_n / self.MAX_EFFORT_N * 100.0, 100.0)
        )
        return max(load, self.stall_load) if self._stalled else load

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
