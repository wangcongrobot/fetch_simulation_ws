#!/usr/bin/env python
import os
import rospy
import sys
import time
import actionlib
from geometry_msgs.msg import Pose
import copy
from rviz_markers import MarkerBasicsArray
from rviz_markers import MarkerBasics
import yaml
import rospkg
import numpy as np
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from sensor_msgs.msg import JointState
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal



class FetchTest(object):

    def __init__(self):
        rospy.logdebug("========= In Fetch Env")

        # this object contains all object's positions!!
        self.obj_positions = Obj_Pos()

        # We Start all the ROS related Subscribers and publishers

        self.JOINT_STATES_SUBSCRIBER = '/joint_states'
        self.joint_names = ["joint0",
                           "joint1",
                           "joint2",
                           "joint3",
                           "joint4",
                           "joint5",
                           "joint6"]

        self._check_all_systems_ready()

        self.joint_states_sub = rospy.Subscriber(
            self.JOINT_STATES_SUBSCRIBER, JointState, self.joints_callback)
        self.joints = JointState()

        # Start Services
        self.move_fetch_object = MoveFetch()

        # Wait until it has reached its Startup Position
        self.wait_fetch_ready()

        rospy.logdebug("========= Out Fetch Env")

    # RobotGazeboEnv virtual methods
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True

    # FetchEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        self._check_joint_states_ready()

        rospy.logdebug("ALL SENSORS READY")

    def _check_joint_states_ready(self):
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message(
                    self.JOINT_STATES_SUBSCRIBER, JointState, timeout=1.0)
                rospy.logdebug(
                    "Current "+str(self.JOINT_STATES_SUBSCRIBER)+" READY=>" + str(self.joints))

            except:
                rospy.logerr(
                    "Current "+str(self.JOINT_STATES_SUBSCRIBER)+" not ready yet, retrying....")
        return self.joints

    def joints_callback(self, data):
        self.joints = data

    def get_joints(self):
        return self.joints

    def get_joint_names(self):
        return self.joints.name

    def set_trajectory_ee(self, action, orientation):
        """
        Sets the Pose of the EndEffector based on the action variable.
        The action variable contains the position and orientation of the EndEffector.
        """
        # Set up a trajectory message to publish.
        ee_target = geometry_msgs.msg.Pose()
        ee_target.orientation.x = orientation[0]
        ee_target.orientation.y = orientation[1]
        ee_target.orientation.z = orientation[2]
        ee_target.orientation.w = orientation[3]
        ee_target.position.x = action[0]
        ee_target.position.y = action[1]
        ee_target.position.z = action[2]

        result = self.move_fetch_object.ee_traj(ee_target)
        return result

    def set_trajectory_joints(self, initial_qpos):

        positions_array = [None] * 7
        positions_array[0] = initial_qpos["joint0"]
        positions_array[1] = initial_qpos["joint1"]
        positions_array[2] = initial_qpos["joint2"]
        positions_array[3] = initial_qpos["joint3"]
        positions_array[4] = initial_qpos["joint4"]
        positions_array[5] = initial_qpos["joint5"]
        positions_array[6] = initial_qpos["joint6"]

        self.move_fetch_object.joint_traj(positions_array)

        return True

    def create_joints_dict(self, joints_positions):
        """
        Based on the Order of the positions, they will be assigned to its joint name
        names_in_order:
          joint0: 0.0
          joint1: 0.0
          joint2: 0.0
          joint3: -1.5
          joint4: 0.0
          joint5: 1.5
          joint6: 0.0
        """

        assert len(joints_positions) == len(
            self.joint_names), "Wrong number of joints, there should be "+str(len(self.joint_names))
        joints_dict = dict(zip(self.joint_names, joints_positions))

        return joints_dict

    def get_ee_pose(self):
        """
        Returns geometry_msgs/PoseStamped
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            geometry_msgs/Pose pose
              geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
              geometry_msgs/Quaternion orientation
                float64 x
                float64 y
                float64 z
                float64 w
        """
        gripper_pose = self.move_fetch_object.ee_pose()
        self.gazebo.pauseSim()
        return gripper_pose

    def get_ee_rpy(self):
        gripper_rpy = self.move_fetch_object.ee_rpy()
        return gripper_rpy

    def wait_fetch_ready(self):
        """
        # TODO: Make it wait for this position
        Desired Position to wait for
        """

        for i in range(20):
            print("WAITING..."+str(i))
            sys.stdout.flush()
            time.sleep(1.0)

        print("WAITING...DONE")

    # ParticularEnv methods
    # ----------------------------

    def test_rubishbin_disposal(self):

        arm_joint_positions = [0.0, -0.9, 0, 0.9, 0.0, 1.57, 0.0]
        arm_joint_positions_dict = self.create_joints_dict(arm_joint_positions)
        self.set_trajectory_joints(arm_joint_positions_dict)

        # Go to Init Safe pose
        print ("Going to Sage Grasp Init Pose...")
        init_joints_config  = [-1.57, -0.7, 0, 0.7, 0.0, 1.57, 0.0]
        init_joints_config_dict = self.create_joints_dict(init_joints_config)
        self.set_trajectory_joints(init_joints_config_dict)

    def test_range_world_movements(self):
        """
        Here we test the range above the table that fetch can move the arm.
        """
        marker_size = 0.1
        marker_array_obj = MarkerBasicsArray(namespace="fetch_test_range", marker_size=marker_size)

        marker_type = "cube"
        namespace = "demo_table"
        mesh_package_path = ""
        demo_table_position = MarkerBasics(type=marker_type,
                                        namespace=namespace,
                                        index=0,
                                        red=0.0,
                                        green=0.0,
                                        blue=1.0,
                                        alfa=1.0,
                                        scale=[0.6,0.6,0.6],
                                        mesh_package_path=mesh_package_path)

        demo_table_pose = Pose()
        demo_table_pose.position.x = 0.3
        demo_table_pose.position.y = 0.3
        demo_table_pose.position.z = 0.3
        demo_table_position.publish_marker(demo_table_pose)


        base_table_height = 0.6
        height_delta = 0.25
        position_XYZ = [0.3, 0.3, base_table_height + height_delta ]
        orientation_XYZW = [-0.707, 0.000, 0.707, 0.001]

        result = self.set_trajectory_ee(position_XYZ, orientation_XYZW)

        # Range of accepcted positions XYZ in world frame, Ok range is if they were ok or not
        range_list = []
        range_list_dicts = []
        ok_range_list = []

        delta_increment = marker_size
        max_value = 0.6

        min_val_X = 0.0
        position_XYZ[0] = min_val_X

        min_val_Y = 0.0
        position_XYZ[1] = min_val_Y

        while position_XYZ[0] <= max_value:

            position_XYZ[1] = min_val_Y
            while position_XYZ[1] <= max_value:
                rospy.logwarn(str(position_XYZ))

                result = self.set_trajectory_ee(position_XYZ, orientation_XYZW)
                ok_range_list.append(result)
                range_list.append(copy.deepcopy(position_XYZ))

                pose_dict = dict(zip(["OK","X","Y","Z"], [ok_range_list[-1],position_XYZ[0],position_XYZ[1],position_XYZ[2]]))
                range_list_dicts.append(pose_dict)

                marker_array_obj.publish_pose_array(range_list,ok_range_list)
                demo_table_position.publish_marker(demo_table_pose)
                position_XYZ[1] += delta_increment

            rospy.logerr("Max Y Value==>"+str(position_XYZ[1]))
            position_XYZ[0] += delta_increment

        rospy.logerr("Max X Value==>"+str(position_XYZ[0]))

        print (str(range_list_dicts))

        rospack = rospkg.RosPack()
        path_to_package = rospack.get_path('fetch_tutorials_pkg')
        pose_files_dir = os.path.join(path_to_package, "fetch_limits_files")

        if not os.path.exists(pose_files_dir):
            os.makedirs(pose_files_dir)

        pose_file_name = "fetch_move_limits.yaml"
        file_to_store = os.path.join(pose_files_dir, pose_file_name)

        self.store_dict_in_yaml(file_to_store, range_list_dicts)


    def test_ee_world_pose(self):
        """
        Test A certain Position and orientation through input interactive

        """

        while not rospy.is_shutdown():
            rospy.logwarn("--------INPUT EE POSE----------")
            ee_x = float(raw_input("End Effector X="))
            ee_y = float(raw_input("End Effector Y="))
            ee_z = float(raw_input("End Effector Z="))
            #ee_x_orientation = float(raw_input("End Effector X Orientation="))
            #ee_y_orientation = float(raw_input("End Effector Y Orientation="))
            #ee_z_orientation = float(raw_input("End Effector Z Orientation="))
            #ee_w_orientation = float(raw_input("End Effector W Orientation="))
            rospy.logwarn("--------XXXXXXXXXXXXXX----------")
            ee_x_orientation = -0.707
            ee_y_orientation = 0.0
            ee_z_orientation = 0.707
            ee_w_orientation = 0.001

            position_XYZ = [ee_x, ee_y, ee_z ]
            orientation_XYZW = [ee_x_orientation, ee_y_orientation, ee_z_orientation, ee_w_orientation]

            result = self.set_trajectory_ee(position_XYZ, orientation_XYZW)


    def store_dict_in_yaml(self, file_to_store, dictionary_to_save):

        with open(file_to_store, 'w') as outfile:
            yaml.dump(dictionary_to_save, outfile, default_flow_style=False)
        rospy.logdebug("Data Saved in=>"+str(file_to_store))


class Obj_Pos(object):
    """
    This object maintains the pose and rotation of the cube in a simulation through Gazebo Service

    """

    def __init__(self):
        world_specs = rospy.ServiceProxy(
            '/gazebo/get_world_properties', GetWorldProperties)()
        self.time = 0
        self.model_names = world_specs.model_names
        self.get_model_state = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)

    def get_states(self):
        """
        Returns the ndarray of pose&rotation of the cube
        """
        for model_name in self.model_names:
            if model_name == "cube":
                data = self.get_model_state(
                    model_name, "world")  # gazebo service client
                return np.array([
                    data.pose.position.x,
                    data.pose.position.y,
                    data.pose.position.z,
                    data.pose.orientation.x,
                    data.pose.orientation.y,
                    data.pose.orientation.z
                ])


# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

class MoveFetch(object):
    def __init__(self):
        rospy.logdebug("===== In MoveFetch")
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")

        # Init Torso Action
        self.torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
        # Gripper Action
        self.gripper_action = GripperActionClient()
        # Point Head action
        self.head_action = PointHeadClient()

        rospy.logdebug("===== Out MoveFetch")

    def ee_traj(self, pose):
        self.group.set_pose_target(pose)
        result = self.execute_trajectory()
        return result

    def joint_traj(self, positions_array):

        self.group_variable_values = self.group.get_current_joint_values()
        self.group_variable_values[0] = positions_array[0]
        self.group_variable_values[1] = positions_array[1]
        self.group_variable_values[2] = positions_array[2]
        self.group_variable_values[3] = positions_array[3]
        self.group_variable_values[4] = positions_array[4]
        self.group_variable_values[5] = positions_array[5]
        self.group_variable_values[6] = positions_array[6]
        self.group.set_joint_value_target(self.group_variable_values)
        result = self.execute_trajectory()

        return result

    def move_torso(self, torso_height):
        """
        Changes the Fetch Torso height based on the height given.
        """
        result = self.torso_action.move_to([torso_height, ])

        return result


    def move_gripper(self, gripper_x, max_effort):
        """
        Moves the gripper to given pose
        """
        result = self.gripper_action.move_gripper(gripper_x, max_effort)

        return result

    def move_head_point(self, XYZ, frame="world"):
        """
        Changes the Fetch Torso height based on the height given.
        """
        result = self.head_action.look_at(XYZ[0], XYZ[1], XYZ[2], frame)

        return result


    def execute_trajectory(self):
        """
        Assuming that the trajecties has been set to the self objects appropriately
        Make a plan to the destination in Homogeneous Space(x,y,z,yaw,pitch,roll)
        and returns the result of execution
        """
        self.plan = self.group.plan()
        result = self.group.go(wait=True)
        return result

    def ee_pose(self):
        gripper_pose = self.group.get_current_pose()
        return gripper_pose

    def ee_rpy(self, request):
        gripper_rpy = self.group.get_current_rpy()
        return gripper_rpy


class GripperActionClient(object):
    def __init__(self):

        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected.")

    def move_gripper(self, gripper_x, max_effort, timeout=5.0):

        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = max_effort
        gripper_goal.command.position = gripper_x

        self.gripper_client.send_goal(gripper_goal)
        result = self.gripper_client.wait_for_result(rospy.Duration(timeout))

        return result

class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        result = self.client.wait_for_result()

        return result





if __name__ == '__main__':
    rospy.init_node('fetch_move_client_node', anonymous=True, log_level=rospy.INFO)

    fetch_test_object = FetchTest()
    #fetch_test_object.test_rubishbin_disposal()
    #fetch_test_object.test_range_world_movements()
    fetch_test_object.test_ee_world_pose()
