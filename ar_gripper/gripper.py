import logging
import time
from threading import Lock

from ar_gripper.feetech import FeetechSMSServo

logger = logging.getLogger(__name__)


class CalibrationError(Exception):
    pass


class Gripper:
    _POSITION_MAX = 4095  # griper closed
    _POSITION_MIN = 150  # gripper open
    _TOTAL_STEPS = _POSITION_MAX - _POSITION_MIN

    _TORQUE_MAX = 50  # maximum gripper torque in percent
    _TORQUE_HOLD = 20  # percentage of torque max

    _WAIT_CHECK_TIME_S = 0.1

    _CALIBRATION_TORQUE = 35

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
        servo.max_torque = self._CALIBRATION_TORQUE
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
        servo.max_torque = 100
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

    def set_max_effort(self, max_effort):
        self.servo.max_torque = self._scale(max_effort, self._TORQUE_MAX)

    def get_position(self):
        position = self.servo.present_position - self._POSITION_MIN
        return 100.0 - self._down_scale(position, self._TOTAL_STEPS)

    def goto_position(self, position, closing_torque):
        """
        :param position: 0..100%, 0% - close, 100% - open
        :param closing_torque: 0..100%
        """
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
            "goto position {} {}: servo position {}".format(
                position, closing_torque, servo_position
            )
        )
        # essentially sets velocity of movement,
        # but also sets max_effort for initial half second of grasp.
        self.set_max_effort(closing_torque)

        if not self._goto_position(servo_position):
            if not self._is_aborted():
                logger.error("goto position failed")
            return False

        # Sets torque to keep gripper in position,
        # but does not apply torque if there is no load.
        # This does not provide continuous grasping torque.
        holding_torque = min(self._TORQUE_HOLD, closing_torque)
        self.set_max_effort(holding_torque)
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
        servo.max_torque = self._TORQUE_MAX
        servo.minimum_startup_force = (
            5  # minimum force which needs to be applied, before the servo starts to act
        )
        servo.torque_limit = 100
        servo.overload_torque = 80
        servo.protection_torque = 20
        servo.protection_time = 0
        servo.drive_mode = 0
        servo.drive_speed = 100000

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

    def _goto_position(self, position):
        self.servo.torque_enable = False
        self.servo.goal_position = position
        return self._wait_for_stop(self.servo)

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
