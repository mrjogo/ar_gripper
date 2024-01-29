#!/usr/bin/env python
import logging

import rospy

from ar_gripper.feetech import USB2FeetechDevice
from ar_gripper.gripper import Gripper
from std_srvs.srv import Empty, EmptyResponse
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandResult
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState

from ar_gripper.helpers import ConnectPythonLoggingToROS

DIAG_UPDATE_INTERVAL_S = 1.0
STATUS_UPDATE_INTERVAL_S = 0.2
UPDATE_RATE_HZ = 20


class GripperActionServer:
    def __init__(self, action_name, gripper):
        self.gripper = gripper
        self._action_server = actionlib.SimpleActionServer(
            action_name,
            GripperCommandAction,
            self._gripper_action_execute,
            False,
        )
        self._action_server.register_preempt_callback(
            self._gripper_action_preempt
        )
        self._action_server.start()

    def _gripper_action_execute(self, goal):
        self._cancelled = False
        rospy.loginfo(
            f"Execute goal: position={goal.command.position:.1f}, "
            f"max_effort={goal.command.max_effort:.1f}"
        )

        if goal.command.max_effort == 0.0:
            rospy.loginfo("Release torque: start")
            succeeded = self.gripper.release()
            rospy.loginfo("Release torque: done")
        else:
            rospy.loginfo("Go to position: start")
            succeeded = self.gripper.goto_position(
                goal.command.position, goal.command.max_effort
            )
            rospy.loginfo(
                f"Go to position: {'done' if succeeded else 'failed'}"
            )

        if not succeeded:
            self.gripper.halt()

        result = GripperCommandResult()
        # not necessarily the current position of the gripper
        # if the gripper did not reach its goal position.
        result.position = self.gripper.get_position()
        result.effort = goal.command.max_effort
        result.stalled = False
        result.reached_goal = succeeded
        self._action_server.set_succeeded(result)

    def _gripper_action_preempt(self):
        rospy.loginfo("Aborting gripper goal")
        self.gripper.abort()


class GripperDiagnostics:
    def __init__(self, grippers):
        self._grippers = grippers

        self._pub = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=1
        )

    def send_diags(self):
        # See diagnostics with: rosrun rqt_runtime_monitor rqt_runtime_monitor
        msg = DiagnosticArray()
        msg.status = []
        msg.header.stamp = rospy.Time.now()

        for gripper in (g.gripper for g in self._grippers):
            for servo in [gripper.servo]:
                status = DiagnosticStatus()
                status.name = f"Gripper '{gripper.name}' servo {servo.servo_id}"
                status.hardware_id = f'{servo.servo_id}'
                temperature = servo.present_temperature
                status.values.append(KeyValue('Temperature', str(temperature)))
                status.values.append(
                    KeyValue('Voltage', str(servo.present_voltage))
                )

                if temperature >= 70:
                    status.level = DiagnosticStatus.ERROR
                    status.message = 'OVERHEATING'
                elif temperature >= 65:
                    status.level = DiagnosticStatus.WARN
                    status.message = 'HOT'
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = 'OK'

                msg.status.append(status)

        self._pub.publish(msg)


class GripperStatus:
    FINGER_OPEN_POS = -0.05
    FINGER_CLOSED_POS = 0.0

    def __init__(self, grippers):
        self._grippers = grippers
        self._joint_state_pub = rospy.Publisher(
            'joint_states', JointState, queue_size=5
        )
        self._pos_fb_pubs = {}
        for gripper in (g.gripper for g in self._grippers):
            self._pos_fb_pubs[gripper.name] = rospy.Publisher(
                f'~{gripper.name}/position_feedback', JointState, queue_size=5
            )

    def publish_status(self):
        state_msg = JointState()
        state_msg.header.stamp = rospy.Time.now()
        for gripper in (g.gripper for g in self._grippers):
            pos = gripper.get_position()
            joint_pos = self.FINGER_OPEN_POS + (100.0 - pos) / 100.0 * (
                self.FINGER_CLOSED_POS - self.FINGER_OPEN_POS
            )
            state_msg.name.append(f'{gripper.name}_ar_gripper_body_finger1')
            state_msg.position.append(joint_pos)

            pos_msg = JointState()
            pos_msg.header.stamp = state_msg.header.stamp
            pos_msg.name.append('finger1')
            pos_msg.position.append(pos)
            self._pos_fb_pubs[gripper.name].publish(pos_msg)

        self._joint_state_pub.publish(state_msg)


class ARGripper:
    def __init__(self, device, gripper_name, servo_id):
        self.gripper = Gripper(device, gripper_name, servo_id)

        self._action_srv = GripperActionServer('~' + gripper_name, self.gripper)
        self._calibrate_srv = rospy.Service(
            '~' + gripper_name + '/calibrate', Empty, self._calibrate_srv
        )

        if not self.gripper.calibrate():
            rospy.signal_shutdown("Gripper calibration failed")

    def _calibrate_srv(self, _msg):
        rospy.loginfo("Calibrate service: request received")
        if self.gripper.calibrate():
            rospy.loginfo("Calibrate service: request successfully completed")
        else:
            rospy.loginfo("Calibrate service: calibration failed")
        return EmptyResponse()


def main():
    for module in ('ar_gripper.gripper', 'ar_gripper.feetech'):
        # reconnect logging calls which are children of this to the ros log system
        logging.getLogger(module).addHandler(ConnectPythonLoggingToROS())
        # logs sent to children of trigger with a level >= this will be redirected to ROS
        logging.getLogger(module).setLevel(logging.INFO)

    rospy.init_node('ar_gripper')
    rospy.loginfo("ARGripper driver starting")

    port_name = rospy.get_param('~port', '/dev/ttyUSB0')
    baudrate = int(rospy.get_param('~baud', '115200'))
    gripper_params = rospy.get_param('~grippers')

    all_servos = []
    grippers = []

    device = USB2FeetechDevice(port_name, baudrate=baudrate)
    for gripper_name, servo_ids in gripper_params.items():
        gripper = ARGripper(device, gripper_name, servo_ids[0])
        all_servos.append(gripper.gripper.servo)
        grippers.append(gripper)

    diagnostics = GripperDiagnostics(grippers)
    status = GripperStatus(grippers)

    # Main Loop
    r = rospy.Rate(UPDATE_RATE_HZ)  # hz
    diags_last_sent = 0
    status_last_sent = 0
    while not rospy.is_shutdown():
        now = rospy.get_time()
        if now - diags_last_sent > DIAG_UPDATE_INTERVAL_S:
            try:
                diagnostics.send_diags()
                diags_last_sent = now
            except Exception as e:
                rospy.logerr_throttle(
                    5.0, "Exception while reading diagnostics: %s" % e
                )

        if now - status_last_sent > STATUS_UPDATE_INTERVAL_S:
            try:
                status.publish_status()
                status_last_sent = now
            except Exception as e:
                rospy.logerr_throttle(
                    5.0, "Exception while publishing status %s" % e
                )

        for servo in all_servos:
            try:
                servo.check_overload_and_recover()
            except Exception as e:
                rospy.logerr_throttle(
                    5.0, "Exception while checking overload: %s" % e
                )
                servo.flush_all()

        r.sleep()

    rospy.loginfo("Exiting")


if __name__ == '__main__':
    main()
