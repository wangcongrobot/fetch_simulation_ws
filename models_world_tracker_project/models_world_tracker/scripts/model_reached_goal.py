#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, UInt8


class ModelReachedGoal:

    def __init__(self, vehicle_name, goal_name, rate_hz, pose_epsilon=0.1):

        rospy.logdebug("Starting Init ModelReachedGoal...")
        self._vehicle_name = vehicle_name
        self._goal_name = goal_name
        self._rate = rospy.Rate(rate_hz)
        self._pose_epsilon = pose_epsilon

        self._vehicle_pose_topic = "/gazebo/" + vehicle_name
        self._goal_pose_topic = "/gazebo/" + goal_name

        self._vehicle_in_goal_topic = "/" + vehicle_name + "/" + goal_name + "/active"
        self._vehicle_in_goal_counter_topic = "/" + \
            vehicle_name + "/" + goal_name + "/counter"

        self.check_all_systems_ready()

        # Subscribers
        self.vehicle_sub = rospy.Subscriber(
            self._vehicle_pose_topic, Pose, self.vehicle_callback)

        self.goal_sub = rospy.Subscriber(
            self._goal_pose_topic, Pose, self.goal_callback)

        # Publishers
        self._goal_active = Bool()
        self._goal_active.data = False
        self.goal_count = UInt8()
        self.goal_count.data = 0

        self.active_history = [False, False]

        self.vehicle_in_goal_pub = rospy.Publisher(
            self._vehicle_in_goal_topic, Bool, queue_size=1)

        self.vehicle_in_goal_counter_pub = rospy.Publisher(
            self._vehicle_in_goal_counter_topic, UInt8, queue_size=1)

        rospy.logdebug("Starting Init ModelReachedGoal...DONE")

    def check_all_systems_ready(self):
        self._check_vehicle_pose_ready()
        self._check_goal_pose_ready()

    def _check_vehicle_pose_ready(self):
        self.vehicle_pose = None
        rospy.logdebug(
            "Waiting for "+self._vehicle_pose_topic+" to be READY...")
        while self.vehicle_pose is None and not rospy.is_shutdown():
            try:
                self.vehicle_pose = rospy.wait_for_message(
                    self._vehicle_pose_topic, Pose, timeout=1.0)
                rospy.logdebug("Current "+self._vehicle_pose_topic+" READY=>")

            except:
                rospy.logerr(self._vehicle_pose_topic+" not ready yet, retry")
        return self.vehicle_pose

    def _check_goal_pose_ready(self):
        self.goal_pose = None
        rospy.logdebug(
            "Waiting for "+self._goal_pose_topic+" to be READY...")
        while self.goal_pose is None and not rospy.is_shutdown():
            try:
                self.goal_pose = rospy.wait_for_message(
                    self._goal_pose_topic, Pose, timeout=1.0)
                rospy.logdebug("Current "+self._goal_pose_topic+" READY=>")

            except:
                rospy.logerr(self._vehicle_pose_topic+" not ready yet, retry")
        return self.goal_pose

    def vehicle_callback(self, data):
        self.vehicle_pose = data

    def goal_callback(self, data):
        self.goal_pose = data

    def position_is_similar(self, pos1, pos2, error):
        """
        Returns True if positions are similar
        """
        deltas = [abs(pos1.x - pos2.x),
                  abs(pos1.y - pos2.y),
                  abs(pos1.z - pos2.z)]

        x_similar = deltas[0] <= error
        y_similar = deltas[1] <= error
        z_similar = deltas[2] <= error
        similar = x_similar and y_similar and z_similar

        rospy.logdebug(str(deltas)+">>>"+str(similar))

        return similar

    def publish_goal_active(self):
        # We calculate

        pos_sim = self.position_is_similar(self.vehicle_pose.position,
                                           self.goal_pose.position,
                                           self._pose_epsilon)

        # We remove the second previous, and move the previous
        self.active_history.pop(0)
        self.active_history.append(pos_sim)

        self._goal_active.data = pos_sim

        self.vehicle_in_goal_pub.publish(self._goal_active)

    def publish_goal_count(self):

        previous_active = self.active_history[0]
        now_active = self.active_history[1]

        if not previous_active and now_active:
            self.goal_count.data += 1

        rospy.logdebug("Count >>>"+str(self.goal_count.data))
        self.vehicle_in_goal_counter_pub.publish(self.goal_count)

    def loop(self):

        while not rospy.is_shutdown():
            self.publish_goal_active()
            self.publish_goal_count()
            self._rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('model_reached_goal', anonymous=True,
                        log_level=rospy.DEBUG)
        vehicle_name = rospy.get_param('~vehicle_name')
        goal_name = rospy.get_param('~goal_name')
        rate_hz = float(rospy.get_param('~publish_rate', 10.0))
        pose_epsilon = float(rospy.get_param('~pose_epsilon', 0.1))

        mrg = ModelReachedGoal(vehicle_name, goal_name, rate_hz, pose_epsilon)
        mrg.loop()
    except rospy.ROSInterruptException:
        pass
