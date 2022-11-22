#include "arm.h"

#include "sensor_msgs/JointState.h"

#include "ur_kinematics/ur_kin.h"

Arm::Arm(const std::string &name)
    : m_nh(), m_name(name),
      m_trajectory_as("/ariac/" + name + "/follow_joint_trajectory", true) {
  std::string base_topic("/ariac/" + name);
  std::string joint_state_topic = base_topic + "/joint_states";

  m_urk_ordering.reserve(7);
  m_urk_ordering.emplace_back("linear_arm_actuator_joint");
  m_urk_ordering.emplace_back("shoulder_pan_joint");
  m_urk_ordering.emplace_back("shoulder_lift_joint");
  m_urk_ordering.emplace_back("elbow_joint");
  m_urk_ordering.emplace_back("wrist_1_joint");
  m_urk_ordering.emplace_back("wrist_2_joint");
  m_urk_ordering.emplace_back("wrist_3_joint");

  m_joint_state_sub = m_nh.subscribe<sensor_msgs::JointState>(
      joint_state_topic, 32, &Arm::joint_state_callback, this);

  ROS_INFO("Waiting for trajectory action server");
  m_trajectory_as.waitForServer();
  ROS_INFO("Trajectory server running");

  // m_trajectory_as = Actionlib
  // m_trajectory_as = {"/ariac/" + name + "/follow_joint_trajectory", true};
}

void Arm::joint_state_callback(
    const boost::shared_ptr<sensor_msgs::JointState const> &msg) {

  ArmJointState new_joint_state;

  int count = 0;

  for (int i = 0; i < m_urk_ordering.size(); ++i) {
    for (int j = 0; j < msg->name.size(); ++j) {
      if (m_urk_ordering[i] == msg->name[j]) {
        new_joint_state[i] = msg->position[j];
        count++;
      }
    }
  }

  if (count < m_urk_ordering.size()) {
    return;
  }

  m_current_joint_state = new_joint_state;

  // ROS_INFO_STREAM_THROTTLE(10, "Joint state: " << );

  double T[16];

  ur_kinematics::forward(m_current_joint_state.data() + 1, (double *)&T);
}

bool Arm::go_to_joint_state(ArmJointState joint_state) {
  static uint64_t count = 0;

  trajectory_msgs::JointTrajectory joint_trajectory;

  joint_trajectory.header.seq = count++;
  joint_trajectory.header.stamp = ros::Time::now();
  joint_trajectory.header.frame_id = "/world";

  joint_trajectory.joint_names = m_urk_ordering;

  joint_trajectory.points.resize(2);

  joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
  joint_trajectory.points[1].time_from_start = ros::Duration(3.0);

  for (int i = 0; i < m_urk_ordering.size(); ++i) {
    joint_trajectory.points[0].positions[i] = m_current_joint_state[i];
    joint_trajectory.points[1].positions[i] = joint_state[i];
  }

  // TODO: set tolerance?

  control_msgs::FollowJointTrajectoryActionGoal action_goal;

  action_goal.goal.trajectory = joint_trajectory;

  actionlib::SimpleClientGoalState state = m_trajectory_as.sendGoalAndWait(
      action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
  // m_trajectory_as

  // actionlib::SimpleClientGoalState state =
  return true;
}

bool Arm::move_linear_actuator(ArmJointState joint_state) {
  for (int i = 1; i < joint_state.size(); ++i) {
    joint_state[i] = m_current_joint_state[i];
  }
  return go_to_joint_state(joint_state);
}

bool Arm::move_arm(ArmJointState joint_state) {
  joint_state[0] = m_current_joint_state[0];
  return go_to_joint_state(joint_state);
}
