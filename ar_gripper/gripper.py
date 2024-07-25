import logging
import time
from math import isclose
from threading import Lock

from ar_gripper.feetech import FeetechSMSServo

logger = logging.getLogger(__name__)


class CalibrationError(Exception):
    pass


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

    def __init__(self, device, name, servo_id):
        self.name = name
        self.servo = FeetechSMSServo(device, servo_id)
        self._init_servo(self.servo)
        self._calibrated = False
        self._aborted = False
        self._aborted_lock = Lock()

    @property
    def calibrated(self):
        return self._calibrated

    def abort(self):
        with self._aborted_lock:
            self._aborted = True

    def _is_aborted(self):
        with self._aborted_lock:
            return self._aborted

    def verify_calibrated(self, previous_position, margin=1):
        """
        Rudimentary way to verify that the gripper is still calibrated by checking that
        the current position is the same as a previously stored position, and assuming
        that if it is, nothing has been moved and the values are still good.

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
        wait_start = time.time()
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
            if time.time() - wait_start > timeout:
                logger.warning("wait for stop timed out")
                return False
        # Stop movement if the move was aborted
        self.halt()
        return False

    def _wait_for_no_load(self, servo, timeout=5.0):
        with self._aborted_lock:
            self._aborted = False
        wait_start = time.time()
        last_load = 1000
        while not self._is_aborted():
            current_load = servo.present_load
            if current_load == last_load and current_load == 0:
                return True
            last_load = current_load
            time.sleep(self._WAIT_CHECK_TIME_S)
            if time.time() - wait_start > timeout:
                logger.warning("wait for no load timed out")
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
        move_start = time.time()
        while not self._is_aborted():
            # Check for timeout
            if time.time() - move_start > TIMEOUT:
                logger.error("goto position timed out")
                break
            # Wait for initial current spike to subside
            if time.time() - move_start < INRUSH_TIME:
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
