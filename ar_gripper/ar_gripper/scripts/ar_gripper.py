#!/usr/bin/env python
import json
import logging
import os
import sys
from math import isclose
from threading import Lock, Thread
from time import monotonic, sleep

import rclpy
from ar_gripper_interfaces.srv import SetHoldingTorque
from control_msgs.action import GripperCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from ar_gripper import tracing
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
        # A grasp occupies its execute callback for as long as the move takes,
        # up to Gripper._goto_position's whole timeout. In the node's default
        # MutuallyExclusiveCallbackGroup that would also hold off the status,
        # diagnostics and overload timers that share it, so joint_states and
        # diagnostics would go silent for the duration of every grasp --
        # exactly when a consumer most wants to see them. Its own group lets
        # the action run concurrently with them.
        self._action_callback_group = MutuallyExclusiveCallbackGroup()
        self._action_server = ActionServer(
            self._node,
            GripperCommand,
            f"~/{gripper_name}/gripper_cmd",
            execute_callback=self._gripper_action_execute,
            handle_accepted_callback=self._handle_accepted_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_callback_group,
        )

        # Real homing, on every backend. Nothing here is allowed to shortcut it
        # for the simulator: _calibrate() is also what establishes the position
        # limits, the position correction and the calibration torque limit that
        # the rest of the driver reads, so a backend that marked itself
        # calibrated instead would have to hand-set all four -- a second
        # implementation of the driver's own state setup, free to drift away
        # from the real one. What the simulator needed to make this possible
        # was a way to feel a hard stop that reports no force; see
        # isaac_servo.StallDetector.
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

    def __init__(self, **node_kwargs):
        # node_kwargs is forwarded straight to rclpy's Node so a caller can
        # supply parameter_overrides. main() passes none; the tests that drive
        # this constructor end to end use it to configure the backend without
        # having to smuggle YAML through the global command line.
        super().__init__("ar_gripper", **node_kwargs)

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
        bus_trace_path = self.declare_parameter(
            "bus_trace_path",
            "",
            ParameterDescriptor(
                description=(
                    "Write a timestamped trace of every Feetech bus transaction "
                    "here on shutdown. Empty disables tracing entirely, which is "
                    "the default and costs nothing on the bus path. Use it to "
                    "profile motion: /joint_states at 5 Hz is far too coarse to "
                    "time a one-second stroke."
                ),
                read_only=True,
            ),
        ).value
        bus_trace_sample_hz = self.declare_parameter(
            "bus_trace_sample_hz",
            0.0,
            ParameterDescriptor(
                description=(
                    "With bus_trace_path set, additionally read present_position "
                    "at this rate on a background thread. 0 (default) does not. "
                    "This ADDS bus traffic rather than observing it -- each read "
                    "occupies the bus for a round trip and competes with the "
                    "control loop -- so raise resolution with it only when you "
                    "need to, and compare against an unsampled run."
                ),
                read_only=True,
            ),
        ).value

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
            # sys.exit(message) prints the message and exits cleanly, no
            # traceback -- matching the "Gripper calibration failed" startup
            # failure below, and what a misconfigured launch argument should
            # look like in the log instead of a raised exception's stack.
            sys.exit(
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

        # Enabled BEFORE the first servo is built, so the startup homing --
        # the longest and least observable motion the driver makes -- is in the
        # trace rather than missing from the front of it.
        self._bus_trace = None
        self._bus_trace_path = bus_trace_path
        self._position_sampler = None
        self._bus_trace_sample_hz = bus_trace_sample_hz
        if bus_trace_path:
            self._bus_trace = tracing.enable()
            self.get_logger().warn(
                f"Feetech bus tracing ON, writing to {bus_trace_path} on shutdown"
            )

        self._isaac_bus = None
        self._isaac_node = None
        self._isaac_executor = None
        self._isaac_spin_thread = None
        self._stamp_clock = None
        if isaac:
            from ar_gripper import gripper as gripper_module
            from ar_gripper.mock import install_fake_serial
            from ar_gripper.scripts.isaac_servo import (
                IsaacJointBus,
                IsaacServo,
                RosClock,
            )

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
            # hook the mock path uses -- but via install_fake_serial(None)
            # directly rather than install_ros_node_loopback(), and done
            # before the device opens (USB2FeetechDevice opens serial in
            # __init__). install_ros_node_loopback() would ALSO swap
            # gripper.py's `time` for FakeTime, which is an INSTANT clock:
            # right for the mock's in-memory model, where the waits exist only
            # to terminate, and wrong here, where the waiting is the point.
            # The seam itself is the right one though, so it is reused just
            # below with a clock that measures simulated time instead.
            self._isaac_bus, _ = install_fake_serial(None)

            # Everything that talks to the simulator lives on its own node,
            # spun by its own SingleThreadedExecutor, for two reasons.
            #
            # Sim time. Isaac runs at a real-time factor well below 1.0, so the
            # driver's inrush window, stall baseline and move timeout, and the
            # ramp timer that paces the commanded target, all mean something
            # different measured on the wall clock than measured against the
            # motion they describe. A ramp ticking on wall time in particular
            # advances the target by the joint's full speed per WALL second
            # while the joint only manages that per SIM second, so the target
            # runs ahead of the joint -- the failure the ramp's own clamp
            # exists to prevent. This node subscribes /clock and runs on it.
            #
            # Its own executor, because rclpy's MultiThreadedExecutor cannot
            # carry a subscription at Isaac's rate. Measured in this workspace:
            # one 120-500 Hz subscription on a MultiThreadedExecutor pins a
            # core at ~0.98 and delivers a fraction of the messages, while the
            # same load on a SingleThreadedExecutor costs ~0.5 of a core and
            # delivers all of them; a MultiThreadedExecutor carrying only
            # timers costs ~0.15. The executor loop spins on an entity that is
            # ready but whose message has not been taken yet -- the take
            # happens on a pool thread -- and that spin burns the GIL, which
            # delays the very pool thread it is waiting for. The visible
            # symptom is a subscription callback that runs tens of
            # milliseconds after a message that arrived on time, which is what
            # made blocking reads here time out. Keeping /clock and
            # /isaac/joint_states, both 120 Hz, off the driver node's
            # MultiThreadedExecutor is what avoids it.
            #
            # It also removes the need for a temporary bootstrap executor:
            # startup calibration runs synchronously inside this constructor,
            # before main() ever spins the driver node, and this executor is
            # already running by then.
            self._isaac_node = rclpy.create_node(
                "ar_gripper_isaac_bus",
                parameter_overrides=[
                    Parameter("use_sim_time", Parameter.Type.BOOL, True)
                ],
            )
            self._isaac_executor = rclpy.executors.SingleThreadedExecutor()
            self._isaac_executor.add_node(self._isaac_node)
            self._isaac_spin_thread = Thread(
                target=self._isaac_executor.spin, daemon=True
            )
            self._isaac_spin_thread.start()
            # Same module-level `time` seam mock.install_ros_node_loopback()
            # uses, with the simulator node's sim-time clock behind it.
            # Installed before anything constructs a Gripper, since gripper.py
            # reads the module attribute at call time.
            gripper_module.time = RosClock(self._isaac_node)
            # This node itself stays on wall time -- adding /clock to its
            # MultiThreadedExecutor is the thing above says not to do -- so its
            # periodic publishers tick in real time, which is what a
            # diagnostics consumer wants anyway. Their message stamps still
            # have to be on the simulator's timeline, or every consumer sees
            # gripper joint states dated differently from the arm's.
            self._stamp_clock = self._isaac_node.get_clock()

        try:
            # Inside the try (not above it): if this raises, the isaac
            # executor above has already started spinning its node on its own
            # thread and must still be torn down by the handler below, or that
            # thread is left running after __init__ gave up.
            device = USB2FeetechDevice(port_name.value, baudrate=baudrate.value)
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
                    # enough -- no change to mock.py is required.
                    #
                    # The joint name is spelled out because tf_prefix is empty in
                    # this deployment. If that ever changes, every site that
                    # hardcodes the UNPREFIXED name has to change with it, and
                    # they are not all in this repository: here, and then in
                    # whatever integrates this driver -- its simulator stage and
                    # asset import both key on the finger's exact name, as do
                    # the controller and joint-limit configs. Do not trust a
                    # count; grep for the bare name across every tree that
                    # consumes this. The xacro is the source they all have to
                    # agree with, and a stage that identifies its articulation
                    # by exact DOF set fails loudly on a mismatch.
                    joint_bus = IsaacJointBus(
                        self._isaac_node,
                        joint_states_topic=isaac_joint_states_topic,
                        command_topic=isaac_command_topic,
                        joint_name="primary_ar_gripper_body_finger1",
                    )
                    self._isaac_bus[-1].servos[servo_ids[0]] = IsaacServo(joint_bus)
                    self._wait_for_simulator(joint_bus)
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
        except BaseException:
            # BaseException, because a calibration failure leaves here as the
            # SystemExit that sys.exit() raises. On any failed construction the
            # simulator executor is already spinning its node on a daemon
            # thread; leaving that running while the interpreter finalizes
            # crashes on the way out. On the success path it keeps running for
            # the life of the process and destroy_node() stops it.
            if isaac:
                self._shutdown_isaac_executor()
            raise

        if self._bus_trace is not None and self._bus_trace_sample_hz > 0.0:
            self._position_sampler = tracing.PositionSampler(
                self._grippers[0].gripper.servo, self._bus_trace_sample_hz
            ).start()
            self.get_logger().warn(
                f"position sampler ON at {self._bus_trace_sample_hz:g} Hz; this "
                "adds bus traffic and will slow the control loop"
            )

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

    # How long startup waits for the simulator, in REAL seconds: this is the
    # one wait that cannot be measured in simulated time, because what it is
    # waiting for is the thing that makes simulated time advance. Generous,
    # since the usual reason for waiting at all is that the driver was started
    # while the stage was still coming up.
    SIMULATOR_WAIT_S = 30.0
    _SIMULATOR_POLL_S = 0.05

    def _wait_for_simulator(self, joint_bus):
        """Block until the simulator is publishing, before homing measures anything.

        Two things have to have arrived, and neither of them is optional.

        A joint state, because the first thing homing does is read the finger's
        position, and a read with nothing on the far end can only time out.

        And a ``/clock`` message, because until one arrives this node's sim-time
        clock reads exactly zero -- and every wait in ``gripper.py`` is a
        deadline on that clock. A deadline taken at zero against a simulator
        that is already some way into its run expires the instant the real time
        arrives, which surfaces as a 20 s homing move "timing out" three
        seconds after it started, blaming the move for the clock.

        The two get a budget each rather than sharing one. Sharing it means a
        slow first sample leaves nothing for the clock, so the clock wait
        expires without ever having waited and reports a simulator "publishing
        joint states but not /clock" that was in fact only late.
        """
        if not joint_bus.wait_for_first_sample(self.SIMULATOR_WAIT_S):
            sys.exit(
                "No /isaac/joint_states within "
                f"{self.SIMULATOR_WAIT_S:g} s: no simulator to drive"
            )
        deadline = monotonic() + self.SIMULATOR_WAIT_S
        while self._isaac_node.get_clock().now().nanoseconds == 0:
            if monotonic() >= deadline:
                sys.exit(
                    f"Simulated time still reads zero after "
                    f"{self.SIMULATOR_WAIT_S:g} s: the simulator is publishing "
                    "joint states but not /clock, so nothing the driver waits "
                    "on can ever expire correctly"
                )
            sleep(self._SIMULATOR_POLL_S)

    def _shutdown_isaac_executor(self):
        """Stop the simulator node's private executor and its spin thread."""
        if getattr(self, "_isaac_executor", None) is None:
            return
        self._isaac_executor.remove_node(self._isaac_node)
        self._isaac_executor.shutdown()
        # Guarded, rather than joined outright: this runs on the failure path
        # too, and joining a thread that was never started is itself an error,
        # which would replace whatever failure got us here with a confusing
        # one. The attribute is pre-initialised to None alongside the node and
        # the executor so that the window between creating the thread and
        # starting it is covered as well.
        if self._isaac_spin_thread is not None and self._isaac_spin_thread.is_alive():
            self._isaac_spin_thread.join(timeout=2.0)
        if self._isaac_spin_thread is not None and self._isaac_spin_thread.is_alive():
            self.get_logger().error(
                "Isaac executor thread did not stop within 2s; it may still be "
                "spinning its node."
            )
        self._isaac_node.destroy_node()
        self._isaac_executor = None
        self._isaac_node = None
        self._isaac_spin_thread = None
        self._stamp_clock = None

    def _write_bus_trace(self):
        """Stop tracing and write the CSV. Never raises: this is diagnostics."""
        if self._position_sampler is not None:
            self._position_sampler.stop()
            self._position_sampler = None
        if self._bus_trace is None:
            return
        trace, self._bus_trace = self._bus_trace, None
        tracing.disable()
        try:
            tracing.write_csv(self._bus_trace_path, trace)
            self.get_logger().info(
                f"wrote {len(trace)} bus transactions to {self._bus_trace_path}"
            )
        except (OSError, ValueError) as exc:
            # An unwritable path or an empty trace. Reported, never raised:
            # losing diagnostics must not turn a clean shutdown into a crash.
            self.get_logger().error(f"could not write bus trace: {exc}")

    def destroy_node(self):
        self._write_bus_trace()
        self._shutdown_isaac_executor()
        return super().destroy_node()

    def _stamp_now(self):
        """Timestamp for outgoing messages, on the simulator's clock if there is one."""
        clock = self._stamp_clock if self._stamp_clock is not None else self.get_clock()
        return clock.now().to_msg()

    def _send_diagnostics(self):
        try:
            # See diagnostics with: rosrun rqt_runtime_monitor rqt_runtime_monitor
            msg = DiagnosticArray()
            msg.status = []
            msg.header.stamp = self._stamp_now()

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
            state_msg.header.stamp = self._stamp_now()
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
