#!/usr/bin/env python
import json
import logging
import os
import sys
from math import isclose
from threading import Lock, Thread

import rclpy
from ar_gripper_interfaces.srv import SetHoldingTorque
from control_msgs.action import GripperCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from ar_gripper.feetech import USB2FeetechDevice
from ar_gripper.gripper import CalibrationError
from ar_gripper.helpers import ConnectPythonLoggingToROS
from ar_gripper.standalone import ARGripperStandalone


class ARGripper:
    """ROS action/service/diagnostics shim over ARGripperStandalone.

    All gripper logic (serial device, grasp state machine, unit maps, calibration
    persistence) lives in ARGripperStandalone; this class only bridges it to the
    GripperCommand action, the calibrate / set_holding_torque services, and the
    node's JointState / diagnostics timers. Behaviour is unchanged vs the
    pre-shim node: the unit maps and persistence format are shared, not
    duplicated.
    """

    def __init__(self, device, gripper_name, servo_id, servo_position_path, node):
        self._node = node
        # Build the standalone WITHOUT calibrating yet, so the action/service
        # endpoints are advertised before the (potentially blocking) startup
        # calibration runs -- matching the pre-shim node's ordering.
        self._standalone = ARGripperStandalone(
            servo_id=servo_id,
            name=gripper_name,
            device=device,
            servo_position_path=servo_position_path,
            calibrate_on_init=False,
        )
        # The underlying Gripper is driven directly by the action/diagnostics code.
        self.gripper = self._standalone.gripper
        self._holding_torque = self.gripper.HOLDING_TORQUE

        self._calibrate_srv = self._node.create_service(
            Empty, f"~/{gripper_name}/calibrate", self._handle_calibrate_srv
        )
        self._set_holding_torque = self._node.create_service(
            SetHoldingTorque,
            f"~/{gripper_name}/set_holding_torque",
            self._handle_set_holding_torque,
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

        try:
            self._standalone.run_startup_calibration()
        except CalibrationError:
            sys.exit("Gripper calibration failed")

    def _handle_calibrate_srv(self, _request, response):
        self._node.get_logger().info("Calibrate service: request received")
        if self.gripper.calibrate():
            self._node.get_logger().info(
                "Calibrate service: request successfully completed"
            )
            self._standalone.save_position()
        else:
            self._node.get_logger().info("Calibrate service: calibration failed")
        return response

    def _handle_set_holding_torque(self, request, response):
        if request.torque < 0 or request.torque >= self.gripper.OVERLOAD_TORQUE:
            response.success = False
            response.msg = (
                f"Max holding torque {request.torque} must be between 0 and "
                f"{self.gripper.OVERLOAD_TORQUE}"
            )
            self._node.get_logger().error(response.msg)
            return response

        self._holding_torque = request.torque
        self._node.get_logger().info(f"Set holding torque to {request.torque}")
        response.success = True
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
                self._node.get_logger().info("Gripper goal aborted")
                return GripperCommand.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._node.get_logger().info("Goal canceled")
                return GripperCommand.Result()

            if goal_handle.request.command.max_effort == ARGripperStandalone.MIN_EFFORT:
                command_msg = "Release torque"
                self._node.get_logger().info(f"{command_msg}: start")
                succeeded = self.gripper.release()
                self._node.get_logger().info("Release torque: done")
            else:
                command_msg = "Go to position"
                self._node.get_logger().info(f"{command_msg}: start")
                request_position_percent = ARGripperStandalone.stroke_to_percent(
                    goal_handle.request.command.position
                )
                max_effort_percent = ARGripperStandalone.effort_to_percent(
                    goal_handle.request.command.max_effort
                )
                # Always close "full bore"
                succeeded = self.gripper.goto_position(
                    request_position_percent,
                    max_effort_percent,
                    holding_torque=self._holding_torque,
                )

        with self._goal_lock:
            if not goal_handle.is_active:
                self._node.get_logger().info("Gripper goal aborted")
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
            result.position = ARGripperStandalone.percent_to_stroke(
                result_position_percent
            )
            result.effort = ARGripperStandalone.percent_to_effort(
                self.gripper.get_effort()
            )
            result.reached_goal = isclose(
                result.position, goal_handle.request.command.position, abs_tol=0.001
            )
            result.stalled = not result.reached_goal and result.effort > 0
            goal_handle.succeed()
            self._goal_handle = None

            # Persist new encoder position
            self._standalone.save_position()
            return result


class ARGripperNode(Node):
    DIAG_UPDATE_INTERVAL_S = 1.0
    STATUS_UPDATE_INTERVAL_S = 0.2
    SERVO_OVERLOAD_CHECK_INTERVAL_S = 0.05

    def __init__(self):
        super().__init__("ar_gripper")

        for module in (
            "ar_gripper.gripper",
            "ar_gripper.feetech",
            "ar_gripper.standalone",
        ):
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
            ARGripperStandalone.DEFAULT_SERVO_POSITION_PATH,
            ParameterDescriptor(
                description=(
                    "The JSON file to store the previous servo position in for "
                    "checking calibration at startup"
                ),
                read_only=True,
            ),
        )

        mock = self.declare_parameter(
            "mock",
            False,
            ParameterDescriptor(
                description=(
                    "Run against an in-process fake Feetech bus instead of real "
                    "serial hardware, for hardware-free bring-up (simulation / mock "
                    "e2e). The real driver, action, calibration and diagnostics code "
                    "all run unchanged -- only the serial servo bus is faked."
                ),
                read_only=True,
            ),
        ).value
        isaac = self.declare_parameter(
            "isaac",
            False,
            ParameterDescriptor(
                description=(
                    "Run against Isaac Sim's simulated finger joint instead of "
                    "real serial hardware or the offline mock. The real driver, "
                    "action, calibration and diagnostics code all run unchanged "
                    "-- only the serial servo bus is backed by Isaac's measured "
                    "joint state instead of an internal travel model."
                ),
                read_only=True,
            ),
        ).value
        if mock and isaac:
            raise ValueError(
                "mock:=true and isaac:=true are mutually exclusive backends; pick one."
            )

        self.all_servos = []
        self._grippers = []

        self._mock_bus = None
        if mock:
            # Route every serial device onto an in-process FakeServo bus. Done
            # before the device opens (USB2FeetechDevice opens serial in __init__).
            from ar_gripper.mock import install_ros_node_loopback

            self.get_logger().warn(
                "ARGripper running in MOCK mode: in-process fake Feetech bus "
                "(no serial hardware)"
            )
            self._mock_bus, _ = install_ros_node_loopback()

        self._isaac_bus = None
        if isaac:
            from ar_gripper.mock import install_fake_serial
            from ar_gripper.scripts.isaac_servo import IsaacJointBus, IsaacServo

            isaac_joint_states_topic = self.declare_parameter(
                "isaac_joint_states_topic",
                "/isaac/joint_states",
                ParameterDescriptor(
                    description="Isaac joint-state topic the servo backend reads.",
                    read_only=True,
                ),
            ).value
            isaac_command_topic = self.declare_parameter(
                "isaac_command_topic",
                "/isaac/gripper/joint_commands",
                ParameterDescriptor(
                    description=(
                        "Isaac joint-command topic the servo backend publishes."
                    ),
                    read_only=True,
                ),
            ).value

            self.get_logger().warn(
                "ARGripper running in ISAAC mode: servo bus backed by Isaac Sim's "
                "measured joint state (no serial hardware)"
            )
            # Route every serial device onto an in-process FakeServo bus, same
            # hook the mock path uses. Done before the device opens
            # (USB2FeetechDevice opens serial in __init__).
            self._isaac_bus, _ = install_fake_serial(None)

            # Startup calibration below (inside the ARGripper() constructor)
            # runs synchronously, still inside this Node's own __init__ --
            # which returns to main() *before* its executor ever spins this
            # node. Without help, IsaacJointBus's joint_states subscription
            # and its command ramp timer would never run, and homing would
            # starve waiting for a sample that never arrives (every read
            # would silently time out and see the same frozen (0, 0, 0)
            # sample forever). Spin this node on a private, temporary
            # executor for the rest of construction; torn down again below
            # before __init__ returns, so main()'s own executor is the only
            # one ever spinning it once construction is done.
            self._isaac_bootstrap_executor = rclpy.executors.SingleThreadedExecutor()
            self._isaac_bootstrap_executor.add_node(self)
            self._isaac_bootstrap_thread = Thread(
                target=self._isaac_bootstrap_executor.spin, daemon=True
            )
            self._isaac_bootstrap_thread.start()

        device = USB2FeetechDevice(port_name.value, baudrate=baudrate.value)
        try:
            for gripper_name, servo_ids in gripper_params.items():
                if len(servo_ids) != 1:
                    raise ValueError(
                        "Only one servo ID per gripper is supported, but gripper "
                        f"'{gripper_name}' has {len(servo_ids)}"
                    )
                if mock:
                    # Seed the fake servo so the real startup homing
                    # (Gripper.calibrate) finds contact then retreats, and skip
                    # position persistence (path None) so every mock run rehomes
                    # deterministically instead of trusting a saved real position.
                    from ar_gripper.mock import HOMING_LOAD_SEQUENCE

                    self._mock_bus[-1].servo(servo_ids[0]).load_sequence = list(
                        HOMING_LOAD_SEQUENCE
                    )
                if isaac:
                    # Replace the default in-memory register file with one backed
                    # by Isaac before the Gripper constructor probes the bus.
                    # FakeSerial.servo() uses setdefault, so pre-seeding here is
                    # enough -- no change to mock.py is required. The joint name
                    # is spelled out because tf_prefix is empty in this
                    # deployment; if that ever changes, this is one of three
                    # places (with TurntableNode.JOINT_NAME and the xacro) that
                    # must change together.
                    joint_bus = IsaacJointBus(
                        self,
                        joint_states_topic=isaac_joint_states_topic,
                        command_topic=isaac_command_topic,
                        joint_name="primary_ar_gripper_body_finger1",
                    )
                    self._isaac_bus[-1].servos[servo_ids[0]] = IsaacServo(joint_bus)
                gripper = ARGripper(
                    device,
                    gripper_name,
                    servo_ids[0],
                    (
                        None
                        if (mock or isaac)
                        else os.path.expanduser(servo_position_path.value)
                    ),
                    self,
                )
                self.all_servos.append(gripper.gripper.servo)
                self._grippers.append(gripper)
        finally:
            # Runs on the success path below and also on a calibration
            # failure, which sys.exit()s out of the loop above: either way
            # the bootstrap executor must stop spinning this node in its own
            # thread before __init__ unwinds, or the daemon thread is still
            # running when the interpreter starts finalizing on the way out
            # of sys.exit() and crashes on shutdown.
            if isaac:
                self._isaac_bootstrap_executor.remove_node(self)
                self._isaac_bootstrap_executor.shutdown()
                self._isaac_bootstrap_thread.join(timeout=2.0)

        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 1
        )
        # Latch (TRANSIENT_LOCAL) the joint state so late-joining subscribers get
        # the current gripper angle on connect instead of waiting for the next
        # update. /joint_states has multiple publishers; rosbridge only latches its
        # shared subscription when *every* publisher is TRANSIENT_LOCAL, so a
        # VOLATILE publisher here would silently defeat latching for the whole
        # topic (e.g. the browser 3D view showing stale turntable/arm poses on
        # connect). Matches the joint_state_broadcaster and turntable publishers.
        joint_state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._joint_state_pub = self.create_publisher(
            JointState, "joint_states", joint_state_qos
        )

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
                joint_pos = ARGripperStandalone.percent_to_stroke(pos_percent)
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
        except KeyboardInterrupt:
            pass
        finally:
            ar_gripper_node.destroy_node()
    finally:
        # A SIGINT (e.g. launch teardown) may have already shut the context down via
        # rclpy's signal handling; guard so we don't raise on a double shutdown.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
