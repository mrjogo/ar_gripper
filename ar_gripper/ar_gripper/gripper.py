import logging
import time
from math import isclose
from threading import Event, Lock, Thread
from time import monotonic as _real_monotonic

from ar_gripper.feetech import FeetechSMSServo

logger = logging.getLogger(__name__)


class CalibrationError(Exception):
    pass


class Deadline:
    """A wait bounded on the driver's clock, with a real-time backstop.

    Every wait in this module is measured with the module-level ``time``, which
    a caller may replace (``mock.py`` does, and so does the Isaac backend).
    Against a simulator that clock is *simulated* time, which is the right unit
    for reasoning about a move: the simulator can run at any real-time factor,
    and a move's budget is a property of the motion, not of how fast the
    machine happens to be simulating it.

    It also means a simulator that STOPS makes the deadline unreachable. Sim
    time never advances, the wait never expires, and the move hangs forever --
    holding the action's lock, so every later goal hangs too and the driver
    needs a restart. Pausing the simulator or reloading its stage is a routine
    thing to do. So each wait also carries a real-time backstop at
    ``WALL_BACKSTOP_FACTOR`` times its budget, which cannot be reached while
    the simulator is merely slow: the worst real-time factor measured on this
    stack is 0.68, and the backstop only trips below 1 / WALL_BACKSTOP_FACTOR.

    ``expired()`` reports *which* deadline tripped, because "timed out" on its
    own does not distinguish a move that failed from a simulator that stopped,
    and those have nothing in common to investigate.
    """

    WALL_BACKSTOP_FACTOR = 3.0

    def __init__(self, seconds):
        self._seconds = seconds
        self._wall_seconds = seconds * self.WALL_BACKSTOP_FACTOR
        self._expiry = time.time() + seconds
        self._wall_expiry = _real_monotonic() + self._wall_seconds

    def elapsed(self):
        """Seconds on the driver's clock since this deadline was created."""
        return time.time() - (self._expiry - self._seconds)

    def expired(self):
        """None while both bounds hold, else a description of the one that tripped."""
        if time.time() >= self._expiry:
            return f"{self._seconds:g} s elapsed on the driver's clock"
        if _real_monotonic() >= self._wall_expiry:
            return (
                f"wall-clock backstop: {self._wall_seconds:g} s of real time passed "
                f"without {self._seconds:g} s passing on the driver's clock, so the "
                "simulator is stopped, paused, or running far below real time"
            )
        return None


class Gripper:
    _POSITION_MAX = 4095  # griper closed
    _POSITION_MIN = 150  # gripper open
    _TOTAL_STEPS = _POSITION_MAX - _POSITION_MIN

    # All torques are absolute percentages of the motor stall torque, regardless of what
    # MAX_TORQUE is set to
    MAX_TORQUE = 100
    OVERLOAD_TORQUE = 30
    HOLDING_TORQUE = 10
    _CALIBRATION_TORQUE = 10

    _WAIT_CHECK_TIME_S = 0.1
    # The inrush wait in _goto_position has nothing to read and nothing to do,
    # so it yields between checks instead of spinning. A bare `continue` loop
    # holds the GIL for whole scheduler quanta at a time, which on the ROS node
    # starves every other Python thread in the process for the length of the
    # window -- measured as the joint-state subscription callback not running
    # within 61 ms of a message that had already arrived on time. Much shorter
    # than _WAIT_CHECK_TIME_S because this one only has to bound the overshoot
    # past INRUSH_TIME.
    _INRUSH_POLL_TIME_S = 0.005

    # -- finger service ------------------------------------------------------
    # Removing and refitting the fingers. Opening is a move: driven against the
    # person pulling, for as long as the work takes, and stopped when told to.
    # Closing is not a move at all -- the two fingers are racks on one pinion,
    # so seating them evenly needs the pinion free to turn while a person
    # pushes each one in by hand, not driven toward closed (which pushes only
    # the finger already engaged and cocks the other against the carriage).
    #
    # The torque is the one calibration already uses. That value was chosen to
    # push the carriage into a hard stop without hurting anything, which is
    # the same requirement the open drive has -- enough to keep the carriage
    # moving while a finger is worked out of it, low enough that a hand can
    # hold it still. Aliased rather than copied so the two cannot drift apart
    # and leave the open drive pushing harder than the one the driver already
    # trusts against a stop.
    FINGER_SERVICE_TORQUE = _CALIBRATION_TORQUE
    # Longest the open drive will push without being told to stop. Long enough
    # for a two-handed job, short enough that a service call forgotten with the
    # operator out of the room does not leave the motor pushing all afternoon.
    FINGER_SERVICE_MAX_S = 60.0
    # Open is the LOW-count side (see _POSITION_MIN / _POSITION_MAX), and 0 is
    # below the calibrated open stop -- which is the point.
    _FINGER_SERVICE_OPEN_GOAL = 0
    # Re-reference and re-command once the encoder gets this close to the goal,
    # so the drive never runs out of travel while the operator is still pulling.
    _FINGER_SERVICE_REARM_TICKS = 100
    _FINGER_SERVICE_JOIN_S = 5.0

    def __init__(self, device, name, servo_id):
        self.name = name
        self.servo = FeetechSMSServo(device, servo_id)
        self._init_servo(self.servo)
        self._calibrated = False
        self._aborted = False
        self._aborted_lock = Lock()
        self._finger_service_lock = Lock()
        self._finger_service_stop = Event()
        self._finger_service_thread = None

    @property
    def calibrated(self):
        return self._calibrated

    def abort(self):
        with self._aborted_lock:
            self._aborted = True

    def _is_aborted(self):
        with self._aborted_lock:
            return self._aborted

    def verify_calibrated(self, previous_position, margin=200):
        """
        Rudimentary way to verify that the gripper is still calibrated by checking that
        the current position is the same as a previously stored position, and assuming
        that if it is, nothing has been moved and the values are still good.

        The margin absorbs the small drift seen when the software is stopped while an
        object is grasped (torque relaxes and the finger creeps a few tens of steps).
        It stays well below the shift caused by removing/reinserting the fingers, so a
        genuine finger swap still triggers a rehome. 200 steps is ~5% of the ~3945-step
        finger stroke.

        :param previous_position: The previous servo position of the gripper
        :param margin: The acceptable margin (+/- inclusive) to still consider the
            position same
        """
        self._calibrated = (
            abs(previous_position - self.servo.present_position) <= margin
        )
        return self._calibrated

    def calibrate(self):
        try:
            self._calibrate()
        except CalibrationError as e:
            logger.error(str(e))
            self.release()
            return False
        else:
            return True

    def _calibrate(self):
        self._calibrated = False
        servo = self.servo
        logger.info(f"calibrating gripper {self.name}")
        servo.torque_limit = self._CALIBRATION_TORQUE
        servo.min_position_limit = 0
        servo.max_position_limit = 4095
        servo.position_correction = 0
        tries = 0
        # move fingers together until they touch
        while servo.present_load < 10:
            if tries >= 3:
                raise CalibrationError("calibration failed: homing failed")
            servo.reset_current_position()
            servo.goal_position = 4095
            if not self._wait_for_stop(servo):
                raise CalibrationError("calibration failed: home move timed out")
            tries += 1

        retreat_count = 0
        # retreat until there is no load anymore
        while servo.present_load > 0:
            if retreat_count >= 5:
                raise CalibrationError(
                    "calibration failed: couldn't retreat from home position"
                )
            servo.goal_position = servo.present_position - 50
            if not self._wait_for_stop(servo):
                raise CalibrationError(
                    "calibration failed: home retreat move timed out"
                )
            retreat_count += 1

        # move to middle position
        servo.reset_current_position()
        self.set_torque(0.5 * self.MAX_TORQUE)
        servo.goal_position = 0
        if not self._wait_for_stop(servo):
            raise CalibrationError(
                "calibration failed: move to middle position timed out"
            )
        # and set it to the servo middle position
        servo.reset_current_position()
        # then completely open the gripper
        servo.goal_position = self._POSITION_MIN
        if not self._wait_for_stop(servo):
            raise CalibrationError(
                "calibration failed: could not move to open position"
            )
        servo.min_position_limit = self._POSITION_MIN
        self._calibrated = True
        logger.info(f"calibrating gripper {self.name} complete")

    # -- finger service -------------------------------------------------------------
    @property
    def finger_service_open_active(self):
        """Whether an open drive is currently pushing."""
        thread = self._finger_service_thread
        return thread is not None and thread.is_alive()

    def finger_service_open(self):
        """Keep driving the fingers open, past the calibrated stop, until stopped.

        For pulling the fingers out of the carriage: the servo has to go on
        pushing the carriage toward open for as long as the operator needs to
        work them free, which is neither a distance nor a duration the driver
        can know in advance. So this starts a background push and returns; the
        caller ends it with :meth:`finger_service_open_stop`, or
        ``FINGER_SERVICE_MAX_S`` ends it on its own.

        "Toward open" is the low-count side -- ``_POSITION_MIN`` (150) is open
        and ``_POSITION_MAX`` (4095) is closed -- so the goal is 0, below the
        calibrated open stop.

        Getting there needs the same two things ``_calibrate`` needs before its
        own hunt for the hard stop, for the same reasons:

        * The calibrated limits come off (see :meth:`_prepare_finger_service`).
          While ``min_position_limit`` is 150 the servo will not accept a goal
          below it, and 150 is exactly where this has to go past.
        * The encoder is re-referenced (``reset_current_position``) before each
          goal is written. The servo counts multi-turn, so after a jam --
          which is how the gripper got here -- ``present_position`` may be a
          long way outside the calibrated stroke, and "0" would then be a goal
          the carriage is already past, or thousands of ticks from. Resetting
          first makes the count 2048 whatever happened, so writing 0 is always
          a request to travel 2048 ticks toward open, and always means the same
          thing. It is what ``_calibrate``'s homing loop does before every
          attempt at 4095.

        Re-referencing is also what makes the push unbounded rather than one
        2048-tick move. When the encoder gets within ``_FINGER_SERVICE_REARM_TICKS``
        of the goal the loop resets and commands again, so there is always more
        travel available. A drive that is stalled against something does not
        re-arm and does not need to -- it is already pushing.

        :returns: ``True`` if a drive was started, ``False`` if one was already
            running (this does not restart or extend it).
        """
        with self._finger_service_lock:
            if self.finger_service_open_active:
                logger.warning("finger service: open drive already running")
                return False
            logger.info(
                f"finger service: driving gripper {self.name} open at "
                f"{self.FINGER_SERVICE_TORQUE:g}% torque for up to "
                f"{self.FINGER_SERVICE_MAX_S:g} s"
            )
            self._prepare_finger_service()
            self._finger_service_stop.clear()
            self._finger_service_thread = Thread(
                target=self._drive_finger_service_open,
                name=f"finger_service_open:{self.name}",
                daemon=True,
            )
            self._finger_service_thread.start()
            return True

    def finger_service_open_stop(self):
        """Stop the open drive and cut torque; report where the servo ended up.

        Safe to call when nothing is running: torque comes off either way. That
        is deliberate rather than defensive -- "stop" is what an operator with
        their hands in the machine reaches for, and it has to mean slack
        fingers whether or not the driver agrees that something was moving.

        :returns: the servo's present position, which is NOT a calibrated
            position: the drive re-references the encoder as it goes, so the
            number says where the count stands, not where the fingers are in a
            stroke that no longer exists. ``None`` if it could not be read --
            the stop itself still succeeded, and never raises.
        """
        with self._finger_service_lock:
            self._finger_service_stop.set()
            thread, self._finger_service_thread = self._finger_service_thread, None
        # Joined outside the lock: the drive loop never takes it, but holding a
        # lock across a join is how that stops being true by accident later.
        if thread is not None:
            thread.join(timeout=self._FINGER_SERVICE_JOIN_S)
            if thread.is_alive():
                logger.error(
                    "finger service: open drive did not stop within "
                    f"{self._FINGER_SERVICE_JOIN_S:g} s; cutting torque anyway"
                )
        # Unconditional, and after the join rather than instead of the loop's
        # own release: the loop cuts torque when it ends on its own bound, and
        # this covers the case where there was no loop at all.
        self._release_after_service("open drive")
        try:
            return self.servo.present_position
        except Exception as exc:
            # Reported, never raised. Torque is already off by this point, so
            # the fingers are safe and the stop SUCCEEDED -- failing the call
            # over a read would tell an operator the opposite of the truth at
            # the moment they most need to believe it.
            logger.exception(
                f"finger service: torque is off, but the servo position could "
                f"not be read back: {exc!r}"
            )
            return None

    def finger_service_close(self):
        """Release torque so the pinion freewheels; report where the servo is.

        Not a move: the two fingers are racks on one pinion, so inserting them
        squarely needs the pinion free to turn while the operator pushes each
        finger in by hand, not driven -- a driven close only pushes the finger
        already engaged and cocks the other one against the carriage. So there
        is nothing to wait for and nothing to command; this just cuts torque
        and returns.

        Callable repeatedly and safe to call whether or not anything is
        currently driving, same as :meth:`finger_service_open_stop` -- "close"
        here means "let the pinion turn", and that is true no matter what came
        before it. Assumes no open drive is running (the caller refuses that
        combination; the two are opposite requests and must not be issued at
        once).

        :returns: the servo's present position, on whatever reference was
            already in force -- this does not re-reference the encoder, so the
            number is only as meaningful as the calibration that was true a
            moment ago and is now gone. ``None`` if it could not be read -- the
            release itself still succeeded, and this never raises.
        """
        logger.info(
            f"finger service: releasing gripper {self.name} so the pinion is "
            "free -- push both fingers in by hand"
        )
        self._release_after_service("release")
        self._calibrated = False
        try:
            return self.servo.present_position
        except Exception as exc:
            # Reported, never raised, for the same reason
            # finger_service_open_stop gives: torque is already off, so the
            # release SUCCEEDED, and failing the call over a read would tell
            # the operator the opposite of the truth.
            logger.exception(
                f"finger service: torque is off, but the servo position could "
                f"not be read back: {exc!r}"
            )
            return None

    def _prepare_finger_service(self):
        """Lift the calibrated limits, set the service torque, drop calibration.

        Exactly what ``_calibrate`` does before it goes looking for the hard
        stop, and for the same reason: the limits describe a stroke that is
        about to stop being true, and while they are in force the servo will
        not accept a goal outside them. ``position_correction`` goes with them
        -- it is the offset that calibrated stroke is expressed in, so leaving
        it set would silently shift every goal written afterwards.
        """
        servo = self.servo
        servo.torque_limit = self.FINGER_SERVICE_TORQUE
        # The three limit registers live in EEPROM, so they are read first and
        # written only on a change -- the same treatment, and for the same
        # reason, that ``_init_servo`` already gives the configuration
        # registers it writes. ``_calibrate`` writes them outright, which is
        # fine there because it runs once per bring-up; an open drive may be
        # stopped and started again more than once in one service session, and
        # each start would otherwise be three EEPROM writes of values that are
        # already correct. ``torque_limit`` above is RAM and stays a plain
        # write.
        self._set_if_different(servo, "min_position_limit", 0)
        self._set_if_different(servo, "max_position_limit", self._POSITION_MAX)
        self._set_if_different(servo, "position_correction", 0)
        # Nothing the driver knows about position survives this: the fingers
        # are about to leave the carriage. Said here rather than inferred
        # later, so ~/gripper_state reports calibrated=False and goto_position
        # refuses until a real rehome has happened.
        self._calibrated = False

    def _release_after_service(self, what):
        """Cut torque, and make a failure to do so impossible to miss.

        Called from the open drive's ``finally`` (so it runs on the failure
        path too -- exactly when the bus is most likely to be the thing that
        failed) and directly from the close release, which has no drive to
        guard and nothing else to do. A release that fails must not replace an
        original error in flight (that one says what actually went wrong) and
        must not pass silently either, because it can mean the motor is still
        driving against a person. So: logged at error, loudly, and swallowed.

        The exception's repr is in the MESSAGE as well, which ruff's TRY401
        calls redundant and which is not redundant here: the handler that
        forwards this module's logging to ROS
        (``helpers.ConnectPythonLoggingToROS``) passes on ``record.msg`` and
        reads nothing else, so ``exc_info`` never reaches the log the operator
        is actually looking at. Without the repr they get "could not cut
        torque" and no way to tell a serial timeout from a bad checksum. The
        same applies to the other two ``logger.exception`` calls below.
        """
        try:
            self.release()
        except Exception as exc:
            logger.exception(
                f"finger service: could not cut torque after the {what}: {exc!r} "
                "-- THE MOTOR MAY STILL BE DRIVING; power the servo down"
            )

    def _drive_finger_service_open(self):
        """The open drive's background loop (see :meth:`finger_service_open`)."""
        servo = self.servo
        deadline = Deadline(self.FINGER_SERVICE_MAX_S)
        armed = False
        try:
            while not self._finger_service_stop.is_set():
                reason = deadline.expired()
                if reason:
                    logger.warning(
                        f"finger service: open drive ended by itself ({reason})"
                    )
                    break
                if (
                    not armed
                    or servo.present_position <= self._FINGER_SERVICE_REARM_TICKS
                ):
                    # Re-checked here rather than only at the top of the loop.
                    # The read just above can block for as long as the bus's
                    # retries take, and a stop landing inside that window would
                    # otherwise be followed by one more reset + goal -- putting
                    # torque back on after finger_service_open_stop() had
                    # already cut it and returned. The window is not closed
                    # outright (nothing between a check and a write can be),
                    # but it shrinks from a retry timeout to a few
                    # instructions, and the release below is the backstop for
                    # what is left.
                    if self._finger_service_stop.is_set():
                        break
                    servo.reset_current_position()
                    servo.goal_position = self._FINGER_SERVICE_OPEN_GOAL
                    armed = True
                time.sleep(self._WAIT_CHECK_TIME_S)
        except Exception as exc:
            # A dying bus, most likely. Caught rather than left to threading's
            # excepthook: without this the thread disappears with torque
            # enabled and goal 0 still standing, the 60 s bound gone with it,
            # and nothing anywhere saying so.
            logger.exception(f"finger service: open drive failed: {exc!r}")
        finally:
            self._release_after_service("open drive")
        # Deliberately not reporting the servo position here. A drive that is
        # ending because the bus failed should not touch the bus again beyond
        # the one write that makes it safe -- and the position is already on
        # ~/gripper_state, and in what finger_service_open_stop() returns.
        logger.info("finger service: open drive stopped, torque off")

    def set_torque(self, torque):
        if torque > self.MAX_TORQUE:
            raise ValueError(f"torque {torque} exceeds max torque {self.MAX_TORQUE}")
        self.servo.torque_limit = torque

    def get_position(self):
        position = self.servo.present_position - self._POSITION_MIN
        return 100.0 - self._down_scale(position, self._TOTAL_STEPS)

    def get_servo_position(self):
        return self.servo.present_position

    def goto_position(self, position, closing_torque, holding_torque=None):
        """
        :param position: 0..100%, 0% - close, 100% - open
        :param closing_torque: 0..100%
        :param holding_torque: 0..100%. Defaults to Gripper.HOLDING_TORQUE
        """
        if holding_torque is None:
            holding_torque = self.HOLDING_TORQUE
        if holding_torque >= self.OVERLOAD_TORQUE:
            logger.error(
                "holding torque exceeds or equals overload torque, aborting move"
            )
            return False
        if closing_torque > self.MAX_TORQUE:
            logger.error("closing torque exceeds max torque, aborting move")
            return False
        if not self._calibrated:
            logger.error("gripper is not calibrated, aborting move")
            return False
        if closing_torque <= 0.0:
            self.release()
            return True

        servo_position = (
            self._scale(100.0 - position, self._TOTAL_STEPS) + self._POSITION_MIN
        )
        logger.info(
            f"goto position {position} (servo: {servo_position}), closing torque: "
            f"{closing_torque}, holding torque: {holding_torque}"
        )

        if not self._goto_position(servo_position, closing_torque, holding_torque):
            if not self._is_aborted():
                logger.error("goto position failed")
            return False

        logger.info("goto position done")
        return True

    def release(self):
        logger.info("Releasing gripper")
        self.servo.torque_enable = False
        return True

    def open(self):
        return self.goto_position(100, 100)

    def halt(self):
        logger.info("Halting gripper")
        self.servo.goal_position = self.servo.present_position

    def get_temperature(self):
        return self.servo.present_temperature

    def get_effort(self):
        return self.servo.present_load

    def _init_servo(self, servo):
        if self.OVERLOAD_TORQUE > self.MAX_TORQUE:
            raise Exception(
                f"Overload torque {self.OVERLOAD_TORQUE} exceeds max torque "
                f"{self.MAX_TORQUE}"
            )

        # Don't do unecessary writes to EPROM
        self._set_if_different(servo, "minimum_startup_force", 5)
        self._set_if_different(servo, "max_torque", self.MAX_TORQUE)
        self._set_if_different(servo, "overload_torque", self.OVERLOAD_TORQUE)
        self._set_if_different(servo, "protection_torque", 20)
        self._set_if_different(servo, "protection_time", 300)
        self._set_if_different(servo, "drive_mode", 0)

        servo.drive_speed = 122500
        servo.torque_limit = self.MAX_TORQUE

    def _wait_for_stop(self, servo, timeout=20.0, stop_delay=3):
        with self._aborted_lock:
            self._aborted = False
        deadline = Deadline(timeout)
        last_position = 5000
        stop_count = 0
        while not self._is_aborted():
            current_position = servo.present_position
            if current_position == last_position and not servo.moving_sign:
                stop_count += 1
                if stop_count >= stop_delay:
                    return True
            else:
                stop_count = 0
            last_position = current_position
            time.sleep(self._WAIT_CHECK_TIME_S)
            reason = deadline.expired()
            if reason:
                logger.warning(f"wait for stop timed out ({reason})")
                return False
        # Stop movement if the move was aborted
        self.halt()
        return False

    def _wait_for_no_load(self, servo, timeout=5.0):
        with self._aborted_lock:
            self._aborted = False
        deadline = Deadline(timeout)
        last_load = 1000
        while not self._is_aborted():
            current_load = servo.present_load
            if current_load == last_load and current_load == 0:
                return True
            last_load = current_load
            time.sleep(self._WAIT_CHECK_TIME_S)
            reason = deadline.expired()
            if reason:
                logger.warning(f"wait for no load timed out ({reason})")
                return False
        return False

    def _goto_position(self, position, closing_torque, holding_torque):
        TIMEOUT = 20.0  # seconds
        INRUSH_TIME = 0.3  # seconds
        BASELINE_SAMPLES = 4
        CURRENT_THRESHOLD = 1.4  # multiplier
        STALL_SAMPLES = 2

        with self._aborted_lock:
            self._aborted = False
        # Turn off the torque momentarily to prevent jerking and not apply new torque to
        # previous position
        self.servo.torque_enable = False
        # essentially sets velocity of movement,
        # but also sets max_effort for initial moments of grasp (until stall is detected
        # and torque drops down to holding_torque)
        self.set_torque(closing_torque)

        holding_torque_applied = False
        num_samples = 0
        current_baseline = 0
        current_stall_count = 0
        # Start move
        self.servo.goal_position = position
        deadline = Deadline(TIMEOUT)
        while not self._is_aborted():
            # Check for timeout
            reason = deadline.expired()
            if reason:
                logger.error(f"goto position timed out ({reason})")
                break
            # Wait for initial current spike to subside
            if deadline.elapsed() < INRUSH_TIME:
                time.sleep(self._INRUSH_POLL_TIME_S)
                continue
            current = self.servo.present_current
            # Accumulate samples to average for baseline
            if num_samples <= BASELINE_SAMPLES:
                num_samples += 1
                current_baseline = (
                    current_baseline * (num_samples - 1) + current
                ) / num_samples
                continue
            # Once baseline is set, count consecutive samples above threshold
            if current > current_baseline * CURRENT_THRESHOLD:
                current_stall_count += 1
            else:
                current_stall_count = 0
            # Apply holding torque if stall is detected or if motion has stopped
            stopped = not self.servo.moving_sign
            if not holding_torque_applied and (
                current_stall_count >= STALL_SAMPLES or stopped
            ):
                self.set_torque(holding_torque)
                holding_torque_applied = True

            if stopped:
                return True
        # Stop movement if the move failed or was aborted
        self.halt()
        return False

    @staticmethod
    def _scale(n, to_max):
        # Scale from 0..100 to 0..to_max
        result = int(n * to_max / 100)
        result = min(result, to_max)
        result = max(result, 0)
        return result

    @staticmethod
    def _down_scale(n, to_max):
        # Scale from 0..to_max to 0..100
        result = int(round(n * 100.0 / to_max))
        result = min(result, 100)
        result = max(result, 0)
        return result

    @staticmethod
    def _set_if_different(servo, attribute, value):
        if not isclose(getattr(servo, attribute), value):
            setattr(servo, attribute, value)
            return True
        return False
