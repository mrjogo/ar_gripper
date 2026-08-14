"""Whole-node tests for the isaac backend's two concurrency guarantees.

Both are invisible offline, both were found by driving the real driver at a
running simulator, and both are properties of how ``ARGripperNode`` wires
itself up -- so both are tested here by building the real node against a stand
-in for the simulator's side of the bridge, rather than by re-creating the
pattern in the test.

* Startup calibration runs synchronously inside ``ARGripperNode.__init__``,
  before ``main()``'s executor ever spins anything. The isaac branch starts a
  private executor for its simulator node; without it the joint-state
  subscription, the ramp timer and the ``/clock`` subscription the driver's
  sim-time clock waits on never run, and homing starves on a sample that can
  never arrive.
* Blocking reads of the simulated servo happen inside callbacks on the driver
  node, which runs on a ``MultiThreadedExecutor``. The simulator's 120 Hz
  traffic must not be on that executor: measured here, one 120 Hz subscription
  on a ``MultiThreadedExecutor`` delivers 53 Hz for a whole core of CPU with
  gaps up to 700 ms, while the same load on a ``SingleThreadedExecutor``
  delivers 118 Hz for 0.13 of a core with a worst gap of 9 ms. Reads that
  block on a sample therefore have to be able to time out for the executor
  layout to be wrong, which is what these assert.

Needs a real rclpy node and real executors; skipped automatically where rclpy
is unavailable.
"""

# ruff: noqa: I001 -- rclpy has to be importorskip'd before anything imports it,
# which puts a statement between the import blocks that the sorter cannot move.

import json
import threading
import time
from contextlib import contextmanager

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

JOINT_NAME = "primary_ar_gripper_body_finger1"
# Isaac's own publish rate for both topics (barbot_stage.py ticks the bridge
# graph at the physics rate), so the load here is the load in the field.
SIM_PERIOD_S = 1.0 / 120.0
# Enough external load for Gripper._calibrate's `present_load < 10` contact
# test to read as contact: IsaacServo maps newtons to percent of stall torque
# against MAX_EFFORT_N (1041.25 N), so 200 N is ~19%.
CONTACT_EFFORT_N = 200.0
# Bound on ARGripperNode's constructor. Generous against the ~1-3 s a healthy
# build takes, but it also has to sit above the longest legitimate failure:
# a wait whose simulated clock has stopped gives up on Deadline's real-time
# backstop, which for the 20 s homing move is 60 s.
CONSTRUCTION_TIMEOUT_S = 90.0


class _SimPublisher:
    """A minimal stand-in for the simulator's side of the bridge.

    Publishes ``/clock`` (the backend's simulator node runs on sim time, so
    without this nothing in the driver's clock ever advances) and
    ``/isaac/joint_states``, from a plain thread rather than a ROS timer so it
    keeps running while the node under test is blocked.
    """

    def __init__(self, node, contact_effort_for_s=0.0):
        self._clock_pub = node.create_publisher(Clock, "/clock", 10)
        self._state_pub = node.create_publisher(JointState, "/isaac/joint_states", 10)
        self._contact_effort_for_s = contact_effort_for_s
        self.position_m = 0.019
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=5.0)

    def _run(self):
        started = time.monotonic()
        while not self._stop.is_set():
            elapsed = time.monotonic() - started
            clock_msg = Clock()
            clock_msg.clock.sec = int(elapsed)
            clock_msg.clock.nanosec = int((elapsed % 1.0) * 1e9)
            self._clock_pub.publish(clock_msg)

            state_msg = JointState()
            state_msg.name = [JOINT_NAME]
            state_msg.position = [self.position_m]
            state_msg.velocity = [0.0]
            state_msg.effort = [
                CONTACT_EFFORT_N if elapsed < self._contact_effort_for_s else 0.0
            ]
            self._state_pub.publish(state_msg)
            time.sleep(SIM_PERIOD_S)


@contextmanager
def simulator(contact_effort_for_s=0.0):
    """Run the stand-in simulator on its own node and executor."""
    node = rclpy.create_node("test_isaac_sim")
    publisher = _SimPublisher(node, contact_effort_for_s=contact_effort_for_s)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    publisher.start()
    try:
        yield publisher
    finally:
        publisher.stop()
        executor.remove_node(node)
        executor.shutdown()
        spin_thread.join(timeout=5.0)
        node.destroy_node()


@contextmanager
def isaac_driver_node(skip_calibration):
    """Build the real ``ARGripperNode`` on the isaac backend and tear it down.

    The isaac branch deliberately patches process-global state (the serial
    factory and gripper.py's clock) and keeps it patched for the process's
    whole life, so an in-process test has to put both back or every later test
    inherits a clock backed by a destroyed node.
    """
    from ar_gripper import feetech
    from ar_gripper import gripper as gripper_module
    from ar_gripper.scripts.ar_gripper import ARGripperNode

    original_serial_for_url = feetech.serial.serial_for_url
    original_time = gripper_module.time
    node = None
    try:
        # Built on a worker thread with a bounded join. Every failure mode this
        # file is about presents as construction not returning -- a starved
        # read, a clock that never advances, an executor that was never spun --
        # and an unbounded constructor turns each of those into a CI job
        # timeout with no failing test to point at, instead of a red test.
        built = {}

        def _build():
            try:
                built["node"] = ARGripperNode(
                    parameter_overrides=[
                        Parameter("isaac", Parameter.Type.BOOL, True),
                        Parameter(
                            "isaac_skip_calibration",
                            Parameter.Type.BOOL,
                            skip_calibration,
                        ),
                        Parameter(
                            "grippers",
                            Parameter.Type.STRING,
                            json.dumps({"primary": [0]}),
                        ),
                    ]
                )
            except BaseException as exc:  # noqa: BLE001 - re-raised on the main thread
                built["error"] = exc

        builder = threading.Thread(target=_build, daemon=True)
        builder.start()
        builder.join(timeout=CONSTRUCTION_TIMEOUT_S)
        if builder.is_alive():
            pytest.fail(
                f"ARGripperNode(isaac) did not finish constructing within "
                f"{CONSTRUCTION_TIMEOUT_S:g} s. Startup calibration runs inside the "
                "constructor, so this means a blocking read or a sleep never "
                "returned -- check that the isaac branch's executor is spinning "
                "its node and that /clock is being serviced."
            )
        if "error" in built:
            raise built["error"]
        node = built["node"]
        yield node
    finally:
        if node is not None:
            node.destroy_node()
        feetech.serial.serial_for_url = original_serial_for_url
        gripper_module.time = original_time


def test_startup_calibration_completes_inside_the_node_constructor():
    """Lets the real startup homing run to completion, with nothing but the
    private executor the isaac branch starts to service the joint-state
    subscription and the ``/clock`` subscription the sim-time driver clock
    waits on.

    Remove that executor and this hangs on the first blocking read and then
    fails calibration; the driver's ``sys.exit`` on that path surfaces here as
    ``SystemExit``.
    """
    from ar_gripper import gripper as gripper_module

    rclpy.init()
    try:
        # Hold contact load long enough for the homing loop's first read to see
        # it, then drop to zero so the retreat loop can converge.
        with (
            simulator(contact_effort_for_s=1.0),
            isaac_driver_node(skip_calibration=False) as node,
        ):
            assert node._grippers[0].gripper.calibrated is True
            # Sim time, not wall time: the driver's clock is the simulator
            # node's clock, and it only advances because /clock is published
            # and serviced. Its epoch is the simulator's, so it reads as a
            # handful of seconds rather than as a unix timestamp.
            assert node._isaac_node.get_parameter("use_sim_time").value is True
            assert 0.0 < gripper_module.time.time() < time.time() / 2.0
    finally:
        rclpy.shutdown()


def test_blocking_reads_from_a_driver_callback_are_never_starved():
    """The starvation harness: 40 blocking servo reads issued from inside a
    callback in the driver node's DEFAULT callback group -- where its status,
    diagnostics and overload timers live and do exactly this -- while the
    simulator publishes throughout.

    All 40 must be satisfied by a real sample (``timeout_count`` 0). Put the
    simulator's 120 Hz traffic on the driver node's ``MultiThreadedExecutor``
    instead of on its own executor and this goes red: that executor spins on a
    subscription whose message has not been taken yet, burning the GIL that the
    pool thread doing the taking needs, so samples that arrived on time are
    delivered tens to hundreds of milliseconds late.

    The behavioural half of that is timing-sensitive by nature -- measured at
    around a third of runs catching the regression when the whole suite runs,
    because whatever a preceding test warms up masks it -- so the arrangement
    that produces the behaviour is asserted structurally first. That half
    cannot be timing-dependent, and it is what actually pins the design.
    """
    reads = 40
    rclpy.init()
    try:
        with simulator(), isaac_driver_node(skip_calibration=True) as node:
            servo = node._grippers[0].gripper.servo
            # The IsaacServo behind the driver's FeetechSMSServo, and the bus
            # whose timeout counter is the measurement.
            bus = node._isaac_bus[-1].servos[0]._bus

            # Structural: the simulator's traffic is on a node of its own, spun
            # by a SingleThreadedExecutor of its own, and the driver node --
            # the one with the MultiThreadedExecutor and the blocking
            # callbacks -- subscribes to none of it.
            assert node._isaac_node is not node
            assert isinstance(node._isaac_executor, SingleThreadedExecutor)
            assert node._isaac_node.executor is node._isaac_executor
            driver_topics = {sub.topic_name for sub in node.subscriptions}
            assert "/isaac/joint_states" not in driver_topics
            assert "/clock" not in driver_topics

            ready = threading.Event()
            done = threading.Event()
            positions = []

            def _read_burst():
                # Once only, and cancelled up front: a timer that keeps coming
                # ready while its own group is occupied makes rclpy's executor
                # busy-wait on it, which would measure that instead.
                if not ready.is_set() or done.is_set():
                    return
                reader_timer.cancel()
                for _ in range(reads):
                    positions.append(servo.present_position)
                done.set()

            # No callback_group: deliberately the node's default group.
            reader_timer = node.create_timer(0.01, _read_burst)

            executor = MultiThreadedExecutor()
            executor.add_node(node)
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()
            try:
                # Let discovery settle: a read issued before the simulator's
                # publisher is matched times out for a reason that has nothing
                # to do with executor layout.
                time.sleep(1.0)
                bus.timeout_count = 0
                ready.set()
                assert done.wait(timeout=30.0), "the read burst never finished"
            finally:
                executor.remove_node(node)
                executor.shutdown()
                spin_thread.join(timeout=5.0)

            assert len(positions) == reads
            assert bus.timeout_count == 0
    finally:
        rclpy.shutdown()
