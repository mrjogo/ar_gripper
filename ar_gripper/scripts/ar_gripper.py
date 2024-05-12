#!/usr/bin/env python
import json
import logging
import os
import sys
from math import isclose
from threading import Lock

import rclpy
from control_msgs.action import GripperCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from ar_gripper.feetech import USB2FeetechDevice
from ar_gripper.gripper import Gripper
from ar_gripper.helpers import ConnectPythonLoggingToROS


class ARGripper:
    SAVED_POSITION_KEY = "position"
    MIN_PERCENT = 0.0
    MAX_PERCENT = 100.0
    FINGER_CLOSED_POS = 0.0
    FINGER_OPEN_POS = 0.05
    MIN_EFFORT = 0.0
    # Not exact, but pretty close
    MAX_EFFORT = 1000.0

    def __init__(self, device, gripper_name, servo_id, servo_position_path, node):
        self.gripper = Gripper(device, gripper_name, servo_id)
        self._node = node
        self._servo_position_path = servo_position_path

        self._calibrate_srv = self._node.create_service(
            Empty, f"~/{gripper_name}/calibrate", self._calibrate_srv
        )

        self._goal_lock = Lock()
        self._commanding_lock = Lock()
        self._goal_handle = None
        self._action_server = ActionServer(
            self._node,
            GripperCommand,
            f"~/{gripper_name}/gripper_cmd",
            execute_callback=self._gripper_action_execute,
            handle_accepted_callback=self._handle_accepted_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        os.makedirs(os.path.dirname(self._servo_position_path), exist_ok=True)
        previous_position = None
        try:
            with open(self._servo_position_path, "r") as f:
                previous_position = (json.load(f))[self.SAVED_POSITION_KEY]
        except (FileNotFoundError, json.decoder.JSONDecodeError, KeyError):
            pass

        if previous_position is None or not self.gripper.verify_calibrated(
            previous_position
        ):
            if not self.gripper.calibrate():
                sys.exit("Gripper calibration failed")
            self._save_servo_position()
        else:
            self._node.get_logger().info("Using previous gripper calibration")

    def _save_servo_position(self):
        with open(self._servo_position_path, "w") as f:
            json.dump({self.SAVED_POSITION_KEY: self.gripper.get_servo_position()}, f)

    def _calibrate_srv(self, _request, response):
        self._node.get_logger().info("Calibrate service: request received")
        if self.gripper.calibrate():
            self._node.get_logger().info(
                "Calibrate service: request successfully completed"
            )
            self._save_servo_position()
        else:
            self._node.get_logger().info("Calibrate service: calibration failed")
        return response

    def _handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._node.get_logger().info("Aborting previous goal")
                # If the gripper is in the servoing block, call abort until it finishes
                rate = self._node.create_rate(50)
                while not self._commanding_lock.acquire(blocking=False):
                    self.gripper.abort()
                    rate.sleep()
                try:
                    self._goal_handle.abort()
                finally:
                    self._commanding_lock.release()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def _goal_callback(self, _goal_request):
        self._node.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal):
        self._node.get_logger().info("Received cancel request")
        # This may still have a race condition if _gripper_action_execute acquires
        # _goal_lock and checks goal_handle.is_cancel_requested BEFORE the goal state
        # machine transitions the goal to CANCELLING, but the result should just be that
        # goal_handle.success() will be called on a canceled goal.
        with self._goal_lock:
            # If the gripper is in the servoing block, call abort until it finishes
            rate = self._node.create_rate(50)
            while not self._commanding_lock.acquire(blocking=False):
                self.gripper.abort()
                rate.sleep()
            self._commanding_lock.release()
            return CancelResponse.ACCEPT

    def _gripper_action_execute(self, goal_handle):
        self._node.get_logger().info(
            f"Execute goal: position={goal_handle.request.command.position:.1f}, "
            f"max_effort={goal_handle.request.command.max_effort:.1f}"
        )

        with self._commanding_lock:
            # Needed to prevent an aborted goal from being executed
            if not goal_handle.is_active:
                self.get_logger().info("Gripper goal aborted")
                return GripperCommand.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._node.get_logger().info("Goal canceled")
                return GripperCommand.Result()

            if goal_handle.request.command.max_effort == self.MIN_EFFORT:
                command_msg = "Release torque"
                self._node.get_logger().info(f"{command_msg}: start")
                succeeded = self.gripper.release()
                self._node.get_logger().info("Release torque: done")
            else:
                command_msg = "Go to position"
                self._node.get_logger().info(f"{command_msg}: start")
                request_position_percent = self.stroke_to_percent(
                    goal_handle.request.command.position
                )
                max_effort_percent = self.effort_to_percent(
                    goal_handle.request.command.max_effort
                )
                succeeded = self.gripper.goto_position(
                    request_position_percent,
                    max_effort_percent,
                )

        with self._goal_lock:
            if not goal_handle.is_active:
                self.get_logger().info("Gripper goal aborted")
                return GripperCommand.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._node.get_logger().info("Goal canceled")
                return GripperCommand.Result()

            self._node.get_logger().info(
                f"{command_msg}: {'done' if succeeded else 'failed'}"
            )

            if not succeeded:
                self.gripper.halt()

            result = GripperCommand.Result()
            # not necessarily the current position of the gripper
            # if the gripper did not reach its goal position.
            result_position_percent = self.gripper.get_position()
            result.position = self.percent_to_stroke(result_position_percent)
            result.effort = self.percent_to_effort(self.gripper.get_effort())
            result.reached_goal = isclose(
                result.position, goal_handle.request.command.position, abs_tol=0.001
            )
            result.stalled = not result.reached_goal and result.effort > 0
            goal_handle.succeed()
            self._goal_handle = None

            # Persist new encoder position
            self._save_servo_position()
            return result

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


class ARGripperNode(Node):
    DIAG_UPDATE_INTERVAL_S = 1.0
    STATUS_UPDATE_INTERVAL_S = 0.2
    SERVO_OVERLOAD_CHECK_INTERVAL_S = 0.05

    def __init__(self):
        super().__init__("ar_gripper")

        for module in ("ar_gripper.gripper", "ar_gripper.feetech"):
            # reconnect logging calls which are children of this to the ros log system
            logging.getLogger(module).addHandler(
                ConnectPythonLoggingToROS(self.get_logger())
            )
            # logs sent to children of trigger with a level >= this will be redirected
            # to ROS
            logging.getLogger(module).setLevel(logging.INFO)

        self.get_logger().info("ARGripper driver starting")

        port_name = self.declare_parameter(
            "port",
            "/dev/ttyUSB0",
            ParameterDescriptor(
                description=(
                    "The port to open communication with the AR Gripper(s) RS-485 bus"
                ),
                read_only=True,
            ),
        )
        baudrate = self.declare_parameter(
            "baud",
            "115200",
            ParameterDescriptor(
                description="The baud rate used to communicate with the AR Gripper(s)",
                read_only=True,
            ),
        )
        gripper_params_json = self.declare_parameter(
            "grippers",
            "{}",
            ParameterDescriptor(
                description=(
                    "The parameters for each gripper. See additional_constraints for "
                    "format"
                ),
                additional_constraints=(
                    'Must be a JSON string in the format {"gripper_1_name_string": '
                    '[1], "gripper_2_name": [5]}, where the value is the servo RS-485 '
                    "ID. Currently only one servo ID per gripper is supported"
                ),
                read_only=True,
            ),
        )
        gripper_params = json.loads(gripper_params_json.value)
        servo_position_path = self.declare_parameter(
            "servo_position_path",
            "~/.ros/ar_gripper/servo_position.json",
            ParameterDescriptor(
                description=(
                    "The JSON file to store the previous servo position in for "
                    "checking calibration at startup"
                ),
                read_only=True,
            ),
        )

        self.all_servos = []
        self._grippers = []

        device = USB2FeetechDevice(port_name.value, baudrate=baudrate.value)
        for gripper_name, servo_ids in gripper_params.items():
            if len(servo_ids) != 1:
                raise ValueError(
                    "Only one servo ID per gripper is supported, but gripper "
                    f"'{gripper_name}' has {len(servo_ids)}"
                )
            gripper = ARGripper(
                device,
                gripper_name,
                servo_ids[0],
                os.path.expanduser(servo_position_path.value),
                self,
            )
            self.all_servos.append(gripper.gripper.servo)
            self._grippers.append(gripper)

        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 1
        )
        self._joint_state_pub = self.create_publisher(JointState, "joint_states", 5)

        self.create_timer(self.DIAG_UPDATE_INTERVAL_S, self._send_diagnostics)
        self.create_timer(self.STATUS_UPDATE_INTERVAL_S, self._send_status)
        self.create_timer(
            self.SERVO_OVERLOAD_CHECK_INTERVAL_S, self._check_servo_overload
        )

    def _send_diagnostics(self):
        try:
            # See diagnostics with: rosrun rqt_runtime_monitor rqt_runtime_monitor
            msg = DiagnosticArray()
            msg.status = []
            msg.header.stamp = self.get_clock().now().to_msg()

            for gripper in (g.gripper for g in self._grippers):
                for servo in [gripper.servo]:
                    status = DiagnosticStatus()
                    status.name = f"Gripper '{gripper.name}' servo {servo.servo_id}"
                    status.hardware_id = f"{servo.servo_id}"
                    temperature = servo.present_temperature
                    status.values.append(
                        KeyValue(key="Temperature", value=str(temperature))
                    )
                    status.values.append(
                        KeyValue(key="Voltage", value=str(servo.present_voltage))
                    )

                    if temperature >= 70:
                        status.level = DiagnosticStatus.ERROR
                        status.message = "OVERHEATING"
                    elif temperature >= 65:
                        status.level = DiagnosticStatus.WARN
                        status.message = "HOT"
                    else:
                        status.level = DiagnosticStatus.OK
                        status.message = "OK"

                    msg.status.append(status)

            self._diagnostics_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(
                f"Exception while reading diagnostics: {e}", throttle_duration_sec=5.0
            )

    def _send_status(self):
        try:
            state_msg = JointState()
            state_msg.header.stamp = self.get_clock().now().to_msg()
            for gripper in self._grippers:
                pos_percent = gripper.gripper.get_position()
                joint_pos = gripper.percent_to_stroke(pos_percent)
                state_msg.name.append(f"{gripper.gripper.name}_ar_gripper_body_finger1")
                state_msg.position.append(joint_pos)

            self._joint_state_pub.publish(state_msg)
        except Exception as e:
            self.get_logger().error(
                f"Exception while publishing status {e}", throttle_duration_sec=5.0
            )

    def _check_servo_overload(self):
        for servo in self.all_servos:
            try:
                servo.check_overload_and_recover()
            except Exception as e:
                self.get_logger().error(
                    f"Exception while checking overload: {e}", throttle_duration_sec=5.0
                )
                servo.flush_all()


def main():
    rclpy.init(args=sys.argv)

    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        ar_gripper_node = ARGripperNode()
        executor.add_node(ar_gripper_node)

        try:
            executor.spin()
        finally:
            ar_gripper_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
