#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>

#include <iri_wam_reproduce_trajectory/ExecTraj.h>

#include <string>



using namespace std;

const std::string trajFiles[2]={"get_food.txt","release_food.txt"};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "iri_wam_aff_demo");

  // Start the service
  ros::NodeHandle nh;

  ros::ServiceClient traj_client_ = nh.serviceClient<iri_wam_reproduce_trajectory::ExecTraj>("/execute_trajectory");
  iri_wam_reproduce_trajectory::ExecTraj trajectory;

  int i = 0;

  while (ros::ok())
  {
      ROS_INFO("Executing trajectory %d", i);
      trajectory.request.file = ros::package::getPath("iri_wam_reproduce_trajectory") + "/config/" + trajFiles[i];
      if (!traj_client_.call(trajectory))
      {
          ROS_ERROR ("Failed to execute [%s] trajectory", trajFiles[i].c_str());
          break;
      }

      i = (i + 1) % 2;

      ros::spinOnce();
      usleep(100000);
  }

  return 0;
}
