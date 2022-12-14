#include "arm.h"

#include "sensor_msgs/JointState.h"

#include "ur_kinematics/ur_kin.h"

#include <cmath>
#include <sstream>

std::string pose_to_string(const geometry_msgs::Pose &pose) {
  std::stringstream ss;

  ss << "(";
  ss << pose.position.x;
  ss << ", ";
  ss << pose.position.y;
  ss << ", ";
  ss << pose.position.z;
  ss << ")";

  return ss.str();
}

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

  m_trajectory_pub = m_nh.advertise<trajectory_msgs::JointTrajectory>(
      base_topic + "/arm/command", 2);

  m_joint_state_sub = m_nh.subscribe<sensor_msgs::JointState>(
      joint_state_topic, 32, &Arm::joint_state_callback, this);

  m_gripper_sub = m_nh.subscribe<osrf_gear::VacuumGripperState>(
      base_topic + "/gripper/state", 32, &Arm::gripper_state_callback, this);

  m_gripper_client = m_nh.serviceClient<osrf_gear::VacuumGripperControl>(
      base_topic + "/gripper/control");

  m_gripper_client.waitForExistence(ros::Duration(-1.0));

  ROS_INFO("Waiting for trajectory action server");
  m_trajectory_as.waitForServer();
  ROS_INFO("Trajectory server running");

  // m_trajectory_as = Actionlib
  // m_trajectory_as = {"/ariac/" + name + "/follow_joint_trajectory", true};
}

bool Arm::set_vacuum_enable(bool enable) {
  osrf_gear::VacuumGripperControl srv;
  srv.request.enable = enable;

  m_gripper_client.call(srv);

  return srv.response.success;
}

void Arm::gripper_state_callback(
    const boost::shared_ptr<osrf_gear::VacuumGripperState const> &msg) {
  ROS_INFO_STREAM_THROTTLE(10.0, "Recieved grippper state. Enabled="
                                     << msg->enabled
                                     << ", Attached=" << msg->attached);
  m_current_gripper_state = *msg;
}

void Arm::joint_state_callback(
    const boost::shared_ptr<sensor_msgs::JointState const> &msg) {

  ArmJointState new_joint_state;

  // ROS_INFO_THROTTLE(10.0, "Received joint state update");

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

  {
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < m_urk_ordering.size(); ++i) {
      if (i != 0)
        ss << ", ";
      ss << m_current_joint_state[i];
    }
    ss << "]";

    // ROS_INFO_THROTTLE_STREAM(10.0, "Joint state: " << ss.str());
    ROS_INFO_STREAM_THROTTLE(10.0, "Current joint state: " << ss.str());
  }

  // ROS_INFO_STREAM_THROTTLE(10, "Joint state: " << );

  double T[4][4];

  ur_kinematics::forward(m_current_joint_state.data() + 1, (double *)&T);

  m_current_pose_local.pose = joint_state_to_pose(m_current_joint_state);
  m_current_pose_local.header.stamp = ros::Time::now();

  ROS_INFO_STREAM_THROTTLE(10.0, "Current position (XYZ): " << pose_to_string(
                                     m_current_pose_local.pose));
}

bool Arm::go_to_joint_state(ArmJointState joint_state, ros::Duration duration) {
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

  const ros::Duration dur_offset(0.1);

  joint_trajectory.points[0].time_from_start = dur_offset;
  joint_trajectory.points[1].time_from_start = duration + dur_offset;

  for (int i = 0; i < m_urk_ordering.size(); ++i) {
    joint_trajectory.points[0].positions.push_back(m_current_joint_state[i]);
    // joint_trajectory.points[0].velocities.push_back(0.0);
    joint_trajectory.points[1].positions.push_back(joint_state[i]);
    // joint_trajectory.points[1].velocities.push_back(0.0);
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

  // actionlib::SimpleClientGoalState state =
  //     m_trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal,
  //                                     ros::Duration(30.0),
  //                                     ros::Duration(30.0));
  // actionlib::SimpleClient
  // if()
  m_trajectory_as.sendGoal(joint_trajectory_as.action_goal.goal);
  bool finished_before_timeout =
      m_trajectory_as.waitForResult(ros::Duration(120.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = m_trajectory_as.getState();
    ROS_INFO("Trajectory finished with state %s with text: %s",
             state.toString().c_str(), state.getText().c_str());
  } else {
    ROS_WARN("Trajectory did not finish before timeout");
  }

  // ROS_INFO("Action Server returned with status: %s. Text: %s",
  //          state.toString().c_str(), state.getText().c_str());

  // m_trajectory_pub.publish(joint_trajectory);
  // ros::Duration wait_dur = duration + duration;
  // wait_dur.sleep();

  return true;
}

bool Arm::move_linear_actuator(double position) {
  ArmJointState joint_state;
  for (int i = 0; i < joint_state.size(); ++i) {
    joint_state[i] = m_current_joint_state[i];
  }
  joint_state[0] = position;

  const double duration_per_meter = 1.0;
  double dist = std::abs(position - m_current_joint_state[0]);

  if (dist < 0.01) {
    dist = 0.01;
  }

  ROS_INFO("Moving linear actuator to position %f (distance of %f meters) in "
           "%f seconds",
           position, dist, dist * duration_per_meter);

  ros::Duration duration(dist * duration_per_meter);

  return go_to_joint_state(joint_state, duration);
}

bool Arm::move_arm(ArmJointState joint_state) {
  // set linear actuator to what it was before
  joint_state[0] = m_current_joint_state[0];
  // set rotation of end effector to same as it was before
  joint_state[6] = m_current_joint_state[6];

  const auto current_pose = m_current_pose_local.pose;
  const auto goal_pose = joint_state_to_pose(joint_state);

  const double x = current_pose.position.x - goal_pose.position.x;
  const double y = current_pose.position.y - goal_pose.position.y;
  const double z = current_pose.position.z - goal_pose.position.z;

  double dist = std::sqrt(x * x + y * y + z * z);

  if (dist < 0.01) {
    dist = 0.5;
  }

  const double duration_per_distance = 1.5;

  ROS_INFO("Move arm %f meters in %f seconds.", dist,
           dist * duration_per_distance);

  ros::Duration duration(dist * duration_per_distance);

  return go_to_joint_state(joint_state, duration);
}

bool Arm::pickup_part(geometry_msgs::Point point,
                      geometry_msgs::Point camera_point, double rotation,
                      bool left, bool agv, bool pickup) {
  point.z += 0.02;

  geometry_msgs::Point hover, waypoint;
  hover = point;
  hover.z += 0.1;

  if (!agv) {
    waypoint.x = camera_point.x;
    if (left) {
      waypoint.y = camera_point.y - 0.4;
    } else {
      waypoint.y = camera_point.y + 0.4;
    }
  } else {
    waypoint.x = camera_point.x - 0.4;
    waypoint.y = camera_point.y;
  }
  waypoint.z = camera_point.z - 0.3;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  go_to_home_pose();
  ros::Duration(0.1).sleep();
  go_to_local_pose(waypoint);
  ros::Duration(0.1).sleep();
  go_to_local_pose(hover);

  rotate_end_effector(rotation);

  ros::Duration(0.1).sleep();

  if (pickup) {
    set_vacuum_enable(true);
  }
  ros::Duration(0.1).sleep();

  go_to_local_pose(point);

  if (!pickup) {
    // if dropping off part, disable vacuum
    set_vacuum_enable(false);
  }
  // wait for correct vacuum state
  ros::Duration(0.3).sleep();
  while (ros::ok() && m_current_gripper_state.attached != pickup)
    ros::Duration(0.1).sleep();

  go_to_local_pose(hover);
  ros::Duration(0.1).sleep();
  rotate_end_effector(0.0);
  go_to_local_pose(waypoint);
  ros::Duration(0.1).sleep();
  go_to_home_pose();
  ros::Duration(0.1).sleep();

  return true;
}

bool Arm::rotate_end_effector(double rotation) {
  ArmJointState joint_state;

  for (int i = 0; i < 6; ++i) {
    joint_state[i] = m_current_joint_state[i];
  }

  while (rotation > 2 * M_PI)
    rotation -= 2 * M_PI;
  while (rotation < 0)
    rotation += 2 * M_PI;

  ROS_INFO("Rotating end effector to angle: %0f radians", rotation);

  joint_state[6] = rotation;
  return go_to_joint_state(joint_state, ros::Duration(1.5));
}

bool Arm::move_linear_actuator_relative(double position) {
  // ArmJointState joint_state;
  const double abs_pos = m_current_joint_state[0] + position;
  return move_linear_actuator(abs_pos);
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

  std::stringstream ss;
  ss << "\n";
  for (int i = 0; i < num_sols; ++i) {
    ss << "Solution #" << i << ": ";
    for (int j = 0; j < 6; ++j) {
      ss << q_des[i][j] << " ";
    }
    ss << "\n";
  }

  int sol_idx = 0;
  for (int i = 0; i < num_sols; ++i) {
    if (q_des[i][3] > M_PI && q_des[i][1] > M_PI && q_des[i][2] < M_PI) {
      sol_idx = i;
      break;
    }
  }

  ROS_INFO_STREAM("Choosing IK solution #" << sol_idx);

  ROS_INFO_STREAM_THROTTLE(60.0, "IK Joint state array: " << ss.str());

  ArmJointState joint_state;
  joint_state[0] = 0.0;
  for (int i = 0; i < 6; ++i) {
    joint_state[i + 1] = q_des[sol_idx][i];
  }

  return move_arm(joint_state);
}

geometry_msgs::Pose Arm::joint_state_to_pose(ArmJointState joint_state) {
  double T[4][4];

  ur_kinematics::forward(joint_state.data() + 1, (double *)&T);

  geometry_msgs::Pose pose;
  pose.position.x = T[0][3];
  pose.position.y = T[1][3];
  pose.position.z = T[2][3];

  return pose;
}

bool Arm::go_to_home_pose() {
  geometry_msgs::Point goal;
  goal.x = -0.3;
  goal.y = 0.2;
  goal.z = 0.3;
  return go_to_local_pose(goal);
}
