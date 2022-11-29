#include "arm.h"

#include "sensor_msgs/JointState.h"

#include "ur_kinematics/ur_kin.h"

Arm::Arm(const std::string &name)
    : m_nh(), m_name(name),
      m_trajectory_as("/ariac/" + name + "/arm/follow_joint_trajectory", true) {
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

  ROS_INFO_THROTTLE(10.0, "Received joint state update");

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

  // ur_kinematics::forward(m_current_joint_state.data() + 1, (double *)&T);
}

bool Arm::go_to_joint_state(ArmJointState joint_state) {
  static uint64_t count = 0;
  static uint64_t as_count = 0;
  // ROS_INFO("Reach go to joint state");

  ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
  spinner.start();

  trajectory_msgs::JointTrajectory joint_trajectory;

  joint_trajectory.header.seq = count++;
  joint_trajectory.header.stamp = ros::Time::now();
  joint_trajectory.header.frame_id = "/world";

  joint_trajectory.joint_names = m_urk_ordering;

  // ROS_INFO("Reached here 1");

  joint_trajectory.points.resize(2);

  joint_trajectory.points[0].time_from_start = ros::Duration(0.1);
  joint_trajectory.points[1].time_from_start = ros::Duration(3.0);

  for (int i = 0; i < m_urk_ordering.size(); ++i) {
    joint_trajectory.points[0].positions.push_back(m_current_joint_state[i]);
    joint_trajectory.points[1].positions.push_back(joint_state[i]);
  }

  // ROS_INFO("Reached here 2");
  // TODO: set toleranceo

  control_msgs::FollowJointTrajectoryAction joint_trajectory_as;

  joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
  joint_trajectory_as.action_goal.header.seq = as_count++;
  joint_trajectory_as.action_goal.header.stamp = ros::Time::now();
  joint_trajectory_as.action_goal.header.frame_id = "/world";

  joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
  joint_trajectory_as.action_goal.goal_id.id = std::to_string(as_count - 1);

  actionlib::SimpleClientGoalState state =
      m_trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal,
                                      ros::Duration(30.0), ros::Duration(30.0));
  ROS_INFO("Action Server returned with status: [%i] %s", state.state_,
           state.toString().c_str());
  // ROS_INFO("Reached here 3");
  // actionlib::SimpleClientGoalState state =
  return true;
}

bool Arm::move_linear_actuator(double position) {
  ArmJointState joint_state;
  for (int i = 0; i < joint_state.size(); ++i) {
    joint_state[i] = m_current_joint_state[i];
  }
  joint_state[0] = position;
  return go_to_joint_state(joint_state);
}

bool Arm::move_arm(ArmJointState joint_state) {
  joint_state[0] = m_current_joint_state[0];
  return go_to_joint_state(joint_state);
}

bool Arm::go_to_local_pose(geometry_msgs::Point point) {

  double T_des[4][4];
  double q_des[8][6];

  T_des[0][0] = 0.0;
  T_des[0][1] = -1.0;
  T_des[0][2] = 0.0;
  T_des[1][0] = 0.0;
  T_des[1][1] = 0.0;
  T_des[1][2] = 1.0;
  T_des[2][0] = -1.0;
  T_des[2][1] = 0.0;
  T_des[2][2] = 0.0;
  T_des[3][0] = 0.0;
  T_des[3][1] = 0.0;
  T_des[3][2] = 0.0;

  T_des[0][3] = point.x;
  T_des[1][3] = point.y;
  T_des[2][3] = point.z;
  T_des[3][3] = 1.0;

  const int num_sols =
      ur_kinematics::inverse((double *)&T_des, (double *)&q_des);

  if (num_sols < 1) {
    ROS_ERROR("Not enough IK solutions");
    return false;
  }

  ArmJointState joint_state;
  joint_state[0] = 0.0;
  for (int i = 0; i < 6; ++i) {
    joint_state[i + 1] = q_des[0][i];
  }

  return move_arm(joint_state);
}
