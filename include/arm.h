#pragma once

#include <array>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"

#include "sensor_msgs/JointState.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

#include "control_msgs/FollowJointTrajectoryAction.h"

using ArmJointState = std::array<double, 7>;

class Arm {
public:
  Arm(const std::string &name);
  ~Arm() = default;

  bool go_to_joint_state(ArmJointState joint_state,
                         ros::Duration duration = ros::Duration(1.0));

  bool move_linear_actuator(double position);
  bool move_arm(ArmJointState joint_state);

  bool go_to_local_pose(geometry_msgs::Point point);

private:
  void joint_state_callback(
      const boost::shared_ptr<sensor_msgs::JointState const> &msg);

  geometry_msgs::Pose joint_state_to_pose(ArmJointState joint_state);

private:
  std::string m_name;

  ros::NodeHandle m_nh;

  // state orderings
  std::vector<std::string> m_urk_ordering;

  // current state
  ArmJointState m_current_joint_state;
  geometry_msgs::PoseStamped m_current_pose_local;

  // subscribers
  ros::Subscriber m_joint_state_sub;
  ros::Subscriber m_gripper_sub;

  // publishers
  ros::Publisher m_trajectory_pub;

  // action server
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      m_trajectory_as;
};
