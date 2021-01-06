#!/usr/bin/env python
from collections import defaultdict

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Empty, EmptyResponse

STATUS_UPDATE_INTERVAL_S = 0.2
UPDATE_RATE_HZ = 20


class Gripper:
    STATUS_UPDATE_RATE_HZ = 50
    ACTION_TIMEOUT_S = 10.0

    def __init__(self, action_name, gripper_name):
        self.name = gripper_name
        self._position = 100.0
        self._at_pos = True
        self._connected = False
        self._action_server = actionlib.SimpleActionServer(
            action_name,
            GripperCommandAction,
            self._gripper_action_execute,
            False,
        )
        self._action_server.start()
        self._sub_listener = CountingSubscribeListener()
        self._sub_listener.connected_cbs.append(self._on_connected)
        self._sub_listener.disconnected_cbs.append(self._on_disconnected)

        cmd_pos_topic = f'~{gripper_name}/cmd_pos'
        self._cmd_pos_pub = rospy.Publisher(
            cmd_pos_topic,
            Float32,
            queue_size=1,
            latch=True,
            subscriber_listener=self._sub_listener,
        )
        cmd_effort_topic = f'~{gripper_name}/cmd_effort'
        self._cmd_effort_pub = rospy.Publisher(
            cmd_effort_topic,
            Float32,
            queue_size=1,
            latch=True,
            subscriber_listener=self._sub_listener,
        )

        self._subs = []
        fb_pos_topic = f'~{gripper_name}/fb_pos'
        self._subs.append(
            rospy.Subscriber(fb_pos_topic, Float32, self._on_fb_pos_received)
        )
        fb_closed_topic = f'~{gripper_name}/fb_at_pos'
        self._subs.append(
            rospy.Subscriber(fb_closed_topic, Bool, self._on_fb_at_pos_received)
        )

    @property
    def position(self):
        return self._position

    @property
    def at_pos(self):
        return self._at_pos

    def _gripper_action_execute(self, goal):
        rospy.loginfo(
            "Execute goal: position=%.1f, max_effort=%.1f"
            % (goal.command.position, goal.command.max_effort)
        )
        self._at_pos = self._position == goal.command.position
        self._cmd_pos_pub.publish(Float32(data=goal.command.position))
        self._cmd_effort_pub.publish(Float32(data=goal.command.max_effort))

        r = rospy.Rate(self.STATUS_UPDATE_RATE_HZ)
        start_time = rospy.Time.now()
        reached = False
        while self._connected:
            if self._at_pos:
                reached = True
                break
            if rospy.Time.now() - start_time >= rospy.Duration.from_sec(
                self.ACTION_TIMEOUT_S
            ):
                break
            r.sleep()

        if not self._connected:
            self._position = goal.command.position
            reached = True

        result = GripperCommandResult()
        # not necessarily the current position of the gripper
        # if the gripper did not reach its goal position.
        result.position = goal.command.position
        result.effort = goal.command.max_effort
        result.stalled = False
        result.reached_goal = reached
        self._action_server.set_succeeded(result)

    def _on_fb_pos_received(self, msg):
        self._position = msg.data

    def _on_fb_at_pos_received(self, msg):
        self._at_pos = msg.data

    def _on_connected(self):
        rospy.loginfo('Simulation connected')
        self._connected = True

    def _on_disconnected(self):
        rospy.loginfo('Simulation disconnected')
        self._connected = False


class CountingSubscribeListener(rospy.SubscribeListener):
    def __init__(self):
        super().__init__()
        self.connected_cbs = []
        self.disconnected_cbs = []
        self._connected = False
        self._peers = defaultdict(int)

    @property
    def connected(self):
        return self._connected

    def peer_subscribe(self, topic_name, _topic_publish, _peer_publish):
        self._peers[topic_name] += 1
        self._update_status()

    def peer_unsubscribe(self, topic_name, num_peers):
        self._peers[topic_name] = num_peers
        self._update_status()

    def _update_status(self):
        connected = sum(self._peers.values()) > 0
        if connected is not self._connected:
            self._connected = connected
            if connected:
                for cb in self.connected_cbs:
                    cb()
            else:
                for cb in self.disconnected_cbs:
                    cb()


class GripperStatus:
    FINGER_OPEN_POS = -0.05
    FINGER_CLOSED_POS = 0.0

    def __init__(self, grippers):
        self._grippers = grippers
        self._pub = rospy.Publisher('joint_states', JointState, queue_size=5)
        self._pos_fb_pubs = {}
        for gripper in (g.gripper for g in self._grippers):
            self._pos_fb_pubs[gripper.name] = rospy.Publisher(
                f'~{gripper.name}/position_feedback', JointState, queue_size=5
            )

    def publish_status(self):
        state_msg = JointState()
        state_msg.header.stamp = rospy.Time.now()
        for gripper in (g.gripper for g in self._grippers):
            pos = gripper.position
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

        self._pub.publish(state_msg)


class ARGripper:
    def __init__(self, gripper_name):
        self.gripper = Gripper('~' + gripper_name, gripper_name)
        self._calibrate_srv = rospy.Service(
            '~' + gripper_name + '/calibrate', Empty, self._calibrate_srv
        )

    @staticmethod
    def _calibrate_srv(_msg):
        rospy.loginfo("Calibrate service class")
        return EmptyResponse()


def main():
    rospy.init_node('ar_gripper')
    rospy.loginfo("ARGripper sim driver starting")
    gripper_params = rospy.get_param('~grippers')

    grippers = []
    for gripper_name, _servo_ids in gripper_params.items():
        gripper = ARGripper(gripper_name)
        grippers.append(gripper)

    status = GripperStatus(grippers)

    # Main Loop
    r = rospy.Rate(UPDATE_RATE_HZ)  # hz
    status_last_sent = 0

    while not rospy.is_shutdown():
        now = rospy.get_time()

        if now - status_last_sent > STATUS_UPDATE_INTERVAL_S:
            try:
                status.publish_status()
                status_last_sent = now
            except Exception as e:
                rospy.logerr("Exception while publishing status %s" % e)

        r.sleep()

    rospy.loginfo("Exiting")


if __name__ == '__main__':
    main()
