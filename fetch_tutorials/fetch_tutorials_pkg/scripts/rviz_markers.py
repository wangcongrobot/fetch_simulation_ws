#!/usr/bin/env python
import os
import sys
sys.path.append('/usr/lib/python2.7/dist-packages')
import rospy
import rospkg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
import random
from jsk_rviz_plugins.msg import OverlayMenu

class MarkerBasics(object):
    def __init__(self, type, namespace="rviz_marker", index=0, red=0.0, green=0.0, blue=0.0, alfa=0.0, scale=None, mesh_package_path=""):

        topic_name = "/"+namespace+"/rviz_marker"
        self.marker_objectlisher = rospy.Publisher(topic_name, Marker, queue_size=1)
        self.rate = rospy.Rate(1)
        self.init_marker(type, namespace, index, red, green, blue, alfa, scale, mesh_package_path)

    def init_marker(self, type, namespace="rviz_marker", index=0, red=0.0, green=0.0, blue=0.0, alfa=0.0, scale=None, mesh_package_path=""):
        """

        :param type:
        :param namespace:
        :param index:
        :param z_val:
        :param red:
        :param green:
        :param blue:
        :param alfa:
        :param mesh_package_path: Path to the mesh file inside a package ex: haro_description/meshes/turtle_real_textures.dae
                where 'haro_description' is the package
        :return:
        """

        self.marker_object = Marker()
        self.marker_object.header.frame_id = "/world"
        self.marker_object.header.stamp = rospy.get_rostime()
        self.marker_object.ns = namespace
        self.marker_object.id = index

        if type == "mesh":
            self.marker_object.type = Marker.MESH_RESOURCE
        elif type == "sphere":
            self.marker_object.type = Marker.SPHERE
        elif type == "cube":
            self.marker_object.type = Marker.CUBE

        self.marker_object.action = Marker.ADD

        my_point = Point()
        self.marker_object.pose.position = my_point

        self.marker_object.pose.orientation.x = 0
        self.marker_object.pose.orientation.y = 0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0

        if scale:
            self.marker_object.scale.x = scale[0]
            self.marker_object.scale.y = scale[1]
            self.marker_object.scale.z = scale[2]

        if type == "mesh":
            extension = os.path.splitext(mesh_package_path)[1]
            if extension == ".dae":
                self.marker_object.color.r = 0.0
                self.marker_object.color.g = 0.0
                self.marker_object.color.b = 0.0
                # This has to be otherwise it will be transparent
                self.marker_object.color.a = 0.0

                package_mesh_file_path = "package://"+mesh_package_path
                self.marker_object.mesh_resource = package_mesh_file_path
                self.marker_object.mesh_use_embedded_materials = True
            else:
                self.marker_object.color.r = red
                self.marker_object.color.g = green
                self.marker_object.color.b = blue
                # This has to be otherwise it will be transparent
                self.marker_object.color.a = alfa

                package_mesh_file_path = "package://" + mesh_package_path
                self.marker_object.mesh_resource = package_mesh_file_path
                self.marker_object.mesh_use_embedded_materials = False

        else:
            self.marker_object.color.r = red
            self.marker_object.color.g = green
            self.marker_object.color.b = blue
            # This has to be otherwise it will be transparent
            self.marker_object.color.a = alfa

        # If we want it forever, 0; otherwise, seconds before disappearing
        self.marker_object.lifetime = rospy.Duration(0)

    def publish_marker(self, new_pose):

        self.marker_object.pose.position.x = new_pose.position.x
        self.marker_object.pose.position.y = new_pose.position.y
        self.marker_object.pose.position.z = new_pose.position.z

        self.marker_object.pose.orientation.x = new_pose.orientation.x
        self.marker_object.pose.orientation.y = new_pose.orientation.y
        self.marker_object.pose.orientation.z = new_pose.orientation.z
        self.marker_object.pose.orientation.w = new_pose.orientation.w

        self.marker_objectlisher.publish(self.marker_object)


class MarkerBasicsArray(object):
    def __init__(self, namespace="rviz_marker_array", marker_size=0.05):

        self._marker_size = marker_size

        topic_name = "/"+namespace+"/rviz_marker"
        self.markerarray_object_publisher = rospy.Publisher(topic_name, MarkerArray, queue_size=1)


    def create_new_marker(self,new_pose, type, namespace="rviz_marker", index=0, red=0.0, green=0.0, blue=0.0, alfa=0.0, scale=None, mesh_package_path=""):

        marker_object = Marker()
        marker_object.header.frame_id = "/world"
        marker_object.header.stamp = rospy.get_rostime()
        marker_object.ns = namespace
        marker_object.id = index

        if type == "mesh":
            marker_object.type = Marker.MESH_RESOURCE
        elif type == "sphere":
            marker_object.type = Marker.SPHERE
        elif type == "cube":
            marker_object.type = Marker.CUBE

        marker_object.action = Marker.ADD

        marker_object.pose.position.x = new_pose.position.x
        marker_object.pose.position.y = new_pose.position.y
        marker_object.pose.position.z = new_pose.position.z

        marker_object.pose.orientation.x = new_pose.orientation.x
        marker_object.pose.orientation.y = new_pose.orientation.y
        marker_object.pose.orientation.z = new_pose.orientation.z
        marker_object.pose.orientation.w = new_pose.orientation.w


        if scale:
            marker_object.scale.x = scale[0]
            marker_object.scale.y = scale[1]
            marker_object.scale.z = scale[2]

        if type == "mesh":
            extension = os.path.splitext(mesh_package_path)[1]
            if extension == ".dae":
                marker_object.color.r = 0.0
                marker_object.color.g = 0.0
                marker_object.color.b = 0.0
                # This has to be otherwise it will be transparent
                marker_object.color.a = 0.0

                package_mesh_file_path = "package://"+mesh_package_path
                marker_object.mesh_resource = package_mesh_file_path
                marker_object.mesh_use_embedded_materials = True
            else:
                marker_object.color.r = red
                marker_object.color.g = green
                marker_object.color.b = blue
                # This has to be otherwise it will be transparent
                marker_object.color.a = alfa

                package_mesh_file_path = "package://" + mesh_package_path
                marker_object.mesh_resource = package_mesh_file_path
                marker_object.mesh_use_embedded_materials = False

        else:
            marker_object.color.r = red
            marker_object.color.g = green
            marker_object.color.b = blue
            # This has to be otherwise it will be transparent
            marker_object.color.a = alfa

        # If we want it forever, 0; otherwise, seconds before disappearing
        marker_object.lifetime = rospy.Duration(0)

        return marker_object


    def add_marker_to_array(self, index, new_pose, rgba):

        new_marker = self.create_new_marker(new_pose,
                                            type="cube",
                                            namespace="rviz_marker",
                                            index=index,
                                            red=rgba[0],
                                            green=rgba[1],
                                            blue=rgba[2],
                                            alfa=rgba[3],
                                            scale=[self._marker_size,self._marker_size,self._marker_size],
                                            mesh_package_path="")

        self.marker_array_object.markers.append(new_marker)


    def publish_marker_array(self):

        self.markerarray_object_publisher.publish(self.marker_array_object)

    def publish_pose_array(self, range_list, ok_list):

        self.marker_array_object = MarkerArray()

        i = 0
        for elementXYZ in range_list:

            new_pose = Pose()
            new_pose.position.x = elementXYZ[0]
            new_pose.position.y = elementXYZ[1]
            new_pose.position.z = elementXYZ[2]

            new_pose.orientation.x = 0.0
            new_pose.orientation.y = 0.0
            new_pose.orientation.z = 0.0
            new_pose.orientation.w = 1.0

            if ok_list[i] == True:
                rgba = [0.0, 1.0, 0.0, 1.0]
            else:
                rgba = [1.0, 0.0, 0.0, 1.0]

            self.add_marker_to_array(i, new_pose, rgba)
            i += 1

        self.publish_marker_array()

class PickObjectMenu(object):
    def __init__(self, namespace="rviz_graps_menu"):

        topic_name = "/"+ namespace + "/test_menu"
        self.menu_publisher = rospy.Publisher(topic_name, OverlayMenu, queue_size=1)
        self.menu = OverlayMenu()
        self.menu.title = "FetchGraspValidator"
        self.menu.menus = ["Nothing Grasped","Grasped Object", "Grasp Fail"]

        # Index 0 is Nothing Grasped
        self.menu.current_index = 0
        self.menu_publisher.publish(self.menu)

    def update_menu(self, index):
        """
        Index = 0 --> Nothing Grapsed
        Index = 1 --> Grasp Success
        Index = 2 --> Grasp Fail
        """

        print ("Index Menu publish==>"+str(index))
        self.menu.current_index = index
        self.menu_publisher.publish(self.menu)



if __name__ == '__main__':
    rospy.init_node('marker_basic_node', anonymous=True)

    marker_type = "mesh"
    mesh_package_path = "dynamic_objects/models/suzanne/meshes/suzanne.stl"

    markerbasics_object1 = MarkerBasics(type=marker_type,
                                       namespace="suzanne",
                                       index=0,
                                       red=1.0,
                                       green=0.0,
                                       blue=0.0,
                                       alfa=1.0,
                                       scale=[1.0,1.0,1.0],
                                       mesh_package_path=mesh_package_path)

    marker_type = "sphere"
    mesh_package_path = ""
    namespace = "centre_of_mass"
    markerbasics_object2 = MarkerBasics(type=marker_type,
                                        namespace=namespace,
                                        index=0,
                                        red=0.0,
                                        green=1.0,
                                        blue=0.0,
                                        alfa=1.0,
                                        scale=[0.2, 0.2, 0.2],
                                        mesh_package_path=mesh_package_path)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        new_pose1 = Pose()
        new_pose1.position.x = random.random()
        new_pose1.position.y = random.random()
        new_pose1.position.z = random.random()

        new_pose2 = Pose()
        new_pose2.position.x = random.random()
        new_pose2.position.y = random.random()
        new_pose2.position.z = random.random()

        markerbasics_object1.publish_marker(new_pose1)
        markerbasics_object2.publish_marker(new_pose2)
        rate.sleep()