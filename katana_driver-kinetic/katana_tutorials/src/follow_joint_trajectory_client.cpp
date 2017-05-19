/*
 * follow_joint_trajectory_client.cpp
 *
 *  Created on: 06.11.2011
 *      Author: martin
 */

#include <katana_tutorials/follow_joint_trajectory_client.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace katana_tutorials
{

FollowJointTrajectoryClient::FollowJointTrajectoryClient(std::string robot_ns) :
    traj_client_(robot_ns+"/katana_arm_controller/follow_joint_trajectory", true), got_joint_state_(false), spinner_(1)
{

  joint_names_.push_back("katana_motor1_pan_joint");
  joint_names_.push_back("katana_motor2_lift_joint");
  joint_names_.push_back("katana_motor3_lift_joint");
  joint_names_.push_back("katana_motor4_lift_joint");
  joint_names_.push_back("katana_motor5_wrist_roll_joint");

  joint_state_sub_ = nh_.subscribe(robot_ns+"/joint_states", 1, &FollowJointTrajectoryClient::jointStateCB, this);
  spinner_.start();

  // wait for action server to come up
  while (!traj_client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the follow_joint_trajectory server");
  }
}

FollowJointTrajectoryClient::~FollowJointTrajectoryClient()
{
}

void FollowJointTrajectoryClient::jointStateCB(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> ordered_js;

  ordered_js.resize(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    bool found = false;
    for (size_t j = 0; j < msg->name.size(); ++j)
    {
      if (joint_names_[i] == msg->name[j])
      {
        ordered_js[i] = msg->position[j];
        found = true;
        break;
      }
    }
    if (!found)
      return;
  }

  ROS_INFO_ONCE("Got joint state!");
  current_joint_state_ = ordered_js;
  got_joint_state_ = true;
}

//! Sends the command to start a given trajectory
void FollowJointTrajectoryClient::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
  // When to start the trajectory: 1s from now
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  traj_client_.sendGoal(goal);
}


control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryClient::changePosition()
{
  const size_t NUM_TRAJ_POINTS = 4;
  const size_t NUM_JOINTS = 5;


  std::vector<double> pick(NUM_JOINTS);
  pick[0] = 0.0303;
  pick[1] = 2.1684;
  pick[2] = 2.0541;
  pick[3] = 0.415;
  pick[4] = 0;

	std::vector<double> place(NUM_JOINTS);
  place[0] = 0.0303;
  place[1] = 0.4091;
  place[2] = -1.376;
  place[3] = -0.9593;
  place[4] = 0;

  // arm pointing straight up
  std::vector<double> straight_up_positions(NUM_JOINTS);
  straight_up_positions[0] = 0.0;
  straight_up_positions[1] = 1.57;
  straight_up_positions[2] = 0.0;
  straight_up_positions[3] = 0.0;
  straight_up_positions[4] = 0.0;

  trajectory_msgs::JointTrajectory trajectory;

  for (ros::Rate r = ros::Rate(10); !got_joint_state_; r.sleep())
  {
    ROS_DEBUG("waiting for joint state...");

    if (!ros::ok())
      exit(-1);
  }

  // First, the joint names, which apply to all waypoints
  trajectory.joint_names = joint_names_;

  trajectory.points.resize(5);

  // trajectory point:
  int ind = 0;
  trajectory.points[ind].time_from_start = ros::Duration(5 * ind);
  trajectory.points[ind].positions = current_joint_state_;

  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(5 * ind);
  trajectory.points[ind].positions.resize(NUM_JOINTS);
  trajectory.points[ind].positions = straight_up_positions;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(6 * ind);
  trajectory.points[ind].positions = pick;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(6 * ind);
  trajectory.points[ind].positions.resize(NUM_JOINTS);
  trajectory.points[ind].positions = straight_up_positions;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(6 * ind);
  trajectory.points[ind].positions = place;


  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  return goal;
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState FollowJointTrajectoryClient::getState()
{
  return traj_client_.getState();
}

} /* namespace katana_tutorials */

std::string topicMsg = "";

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("%s", msg->data.c_str());
  topicMsg = msg->data.c_str();
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "follow_joint_trajectory_client");
  std::string robot_ns;
  ros::NodeHandle n;

  n.param("robotname", robot_ns, robot_ns); //get robot namespace

	katana_tutorials::FollowJointTrajectoryClient arm(robot_ns);
	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);


	while(1){
	 
		if(topicMsg == "1"){
			 arm.startTrajectory(arm.changePosition());

			 while (!arm.getState().isDone() && ros::ok())
				{
					usleep(50000);
				}
				topicMsg = "";
		}
	}

}
