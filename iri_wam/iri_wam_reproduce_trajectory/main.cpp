#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <std_srvs/Empty.h>
#include "iri_wam_reproduce_trajectory/ExecTraj.h"

using namespace std;

typedef  vector < vector <double> > Trajectory;

class Trajectories
{

public:

    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;
    Client* _ac;

    Trajectories()
    {
        // creates the action client
        // true causes the client to spin its own thread
        _ac = new Client("/iri_wam/iri_wam_controller/follow_joint_trajectory", true);

        // wait for the action server to start
        _ac->waitForServer(); //will wait for infinite time
    }

    ~Trajectories()
    {
        delete _ac;
    }

    std::string trajExec(std::string file)
    {
        ROS_INFO("Action server started, sending goal.");
        Trajectory trajectory = readFile (file);
        if (trajectory.size() == 0) return "trajectory file not found";

        // send a goal to the action;
        _ac->sendGoal( buildGoal (trajectory) );

        //wait for the action to return
        bool finished_before_timeout = _ac->waitForResult(ros::Duration(50.0));

        if (finished_before_timeout)
        {
          actionlib::SimpleClientGoalState state = _ac->getState();
          ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
        {
          ROS_INFO("Action did not finish before the time out.");
          return "timeout";
        }

        return "ok";
    }

 private:

    Trajectory readFile (std::string fileName)
    {
      Trajectory listOfPoints;
      string line;
      ifstream myfile (fileName.c_str());
      if (myfile.is_open())
      {
        while ( getline (myfile,line) )
        {
          std::string delimiter = ",";

          size_t pos = 0;
          std::string token;
          vector<double> point;
          while ((pos = line.find(delimiter)) != std::string::npos)
          {
            token = (line.substr(0, pos));
            point.push_back (strtod(token.c_str(),NULL));
            line.erase(0, pos + delimiter.length());
          }
          listOfPoints.push_back(point);
        }
        myfile.close();
      }
      else
        ROS_ERROR( "Unable to open file [%s]", fileName.c_str());

      return listOfPoints;
    }

    control_msgs::FollowJointTrajectoryGoal buildGoal ( Trajectory trajectory)
    {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.header.stamp = ros::Time::now()  + ros::Duration(1.0);
        goal.trajectory.header.frame_id = "iri_wam_link_base";

        // First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("iri_wam_joint_1");
        goal.trajectory.joint_names.push_back("iri_wam_joint_2");
        goal.trajectory.joint_names.push_back("iri_wam_joint_3");
        goal.trajectory.joint_names.push_back("iri_wam_joint_4");
        goal.trajectory.joint_names.push_back("iri_wam_joint_5");
        goal.trajectory.joint_names.push_back("iri_wam_joint_6");
        goal.trajectory.joint_names.push_back("iri_wam_joint_7");

        // setting the number of points in the trajectory
        goal.trajectory.points.resize( trajectory.size());

        // First trajectory point
        // Positions
        for (int i=0 ; i < trajectory.size(); i++)
        {

            std::vector<double> point = trajectory.at(i);
            goal.trajectory.points[i].positions.resize(7);
            goal.trajectory.points[i].velocities.resize(7);
            goal.trajectory.points[i].accelerations.resize(7);

            goal.trajectory.points[i].positions[0] = point.at(0);
            //goal.trajectory.points[i].velocities[0] = point.at(7);

            goal.trajectory.points[i].positions[1] = point.at(1);
            //goal.trajectory.points[i].velocities[1] = point.at(8);

            goal.trajectory.points[i].positions[2] = point.at(2);
            //goal.trajectory.points[i].velocities[2] = point.at(9);

            goal.trajectory.points[i].positions[3] = point.at(3);
            //goal.trajectory.points[i].velocities[3] = point.at(10);

            goal.trajectory.points[i].positions[4] = point.at(4);
            //goal.trajectory.points[i].velocities[4] = point.at(11);

            goal.trajectory.points[i].positions[5] = point.at(5);
            //goal.trajectory.points[i].velocities[5] = point.at(12);

            goal.trajectory.points[i].positions[6] = point.at(6);
            //goal.trajectory.points[i].velocities[6] = point.at(13);

            // To be reached 1 second after starting along the trajectory
            goal.trajectory.points[i].time_from_start = ros::Duration(1.0 + i * 0.02);
        }

        return goal;
    }

};


bool trajectory_execution_callback(iri_wam_reproduce_trajectory::ExecTraj::Request &req,
                                   iri_wam_reproduce_trajectory::ExecTraj::Response &res)
{
    Trajectories traj;
    if (traj.trajExec(req.file) == "ok")
        return true;

    return false;
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "reproduce_trajectory");

  ROS_INFO("Waiting for action server to start.");



  if (argc > 1) /// If the program was started with a trajectory parameter to set in an initial position the robot
  {
    Trajectories traj;
    std::string file(argv[1]);
    traj.trajExec(file);
  }


  // Start the service
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("execute_trajectory", &trajectory_execution_callback);
  ROS_INFO("Ready execute trajectories");
  ros::spin();

  //exit
  return 0;
}
