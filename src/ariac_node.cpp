#include <ros/console.h>
#include <ros/ros.h>

#include "osrf_gear/AGVControl.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/SubmitShipment.h"

#include "sensor_msgs/JointState.h"

#include "trajectory_msgs/JointTrajectory.h"

#include "ur_kinematics/ur_kin.h"

#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/Pose.h"

#include <std_msgs/String.h>

#include <std_srvs/Trigger.h>

#include <array>
#include <iostream>
#include <map>
#include <queue>
#include <string>

#include <boost/thread.hpp>

#include "arm.h"

using osrf_gear::LogicalCameraImage;

template <size_t N>
using LogicalCameraArray = std::array<LogicalCameraImage, N>;

using LogicalCameraPtr = const boost::shared_ptr<
    const osrf_gear::LogicalCameraImage_<std::allocator<void>>>;

using ImageCallback = boost::function<void(LogicalCameraPtr)>;

// constants
constexpr int k_buffer_size = 16;

// global transform listener
tf2_ros::Buffer g_tf_buf;

// global data
std::queue<osrf_gear::Shipment> g_shipment_queue;
std::string g_agv1_state;
std::string g_agv2_state;

LogicalCameraArray<6> g_bin_images;
LogicalCameraArray<2> g_agv_images;
LogicalCameraArray<2> g_quality_images;

void adjust_pose(geometry_msgs::PoseStamped &pose) {
  pose.pose.position.z += 0.1;

  pose.pose.orientation.w = 0.707;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.707;
  pose.pose.orientation.z = 0.0;
}

void print_pose(const geometry_msgs::Pose &pose) {
  ROS_DEBUG("%0f %0f %0f", pose.position.x, pose.position.y, pose.position.z);
}

geometry_msgs::TransformStamped
get_robot_to_frame(const std::string &to_frame) {
  geometry_msgs::TransformStamped tf;
  try {
    tf = g_tf_buf.lookupTransform("arm1_base_link", to_frame, ros::Time(0.0),
                                  ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Transform fetch error: %s", ex.what());
  }
  return tf;
}

double yaw_from_pose(const geometry_msgs::Pose &pose) {
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);

  tf2::Matrix3x3 m(q);
  double yaw, pitch, roll;

  m.getRPY(roll, pitch, yaw);

  ROS_WARN("RPY for pose: %f %f %f", roll, pitch, yaw);

  // Yes, I know this looks wrong
  return yaw;
}

void order_callback(const osrf_gear::Order &order) {
  static int n = 0;
  ROS_INFO("Recieved order %d", n++);

  for (const auto &s : order.shipments) {
    // loop over every product in order
    g_shipment_queue.push(s);
    ROS_INFO("Adding shipment to queue");
  }
}

void agv1_callback(const std_msgs::StringConstPtr &msg) {
  g_agv1_state = msg->data;
}

void agv2_callback(const std_msgs::StringConstPtr &msg) {
  g_agv2_state = msg->data;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ariac_node");

  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
  spinner.start();

  tf2_ros::TransformListener tfListener(g_tf_buf);

  auto material_location_client =
      nh.serviceClient<osrf_gear::GetMaterialLocations>(
          "/ariac/material_locations");

  material_location_client.waitForExistence(ros::Duration(-1.0));

  auto agv1_client = nh.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
  auto agv2_client = nh.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
  agv1_client.waitForExistence(ros::Duration(-1.0));
  agv2_client.waitForExistence(ros::Duration(-1.0));

  auto shipment_submision_client =
      nh.serviceClient<osrf_gear::SubmitShipment>("/ariac/submit_shipment");
  shipment_submision_client.waitForExistence();

  Arm arm("arm1");

  auto order_sub = nh.subscribe("/ariac/orders", 2, order_callback);

  auto agv1_state_sub = nh.subscribe("/ariac/agv1/state", 32, agv1_callback);
  auto agv2_state_sub = nh.subscribe("/ariac/agv2/state", 32, agv2_callback);

  // arrays of subsribers
  std::array<ros::Subscriber, 6> bin_camera_subs;
  std::array<ros::Subscriber, 2> agv_camera_subs;
  std::array<ros::Subscriber, 2> quality_camera_subs;

  ROS_INFO("Subscribing to bin cameras");
  // generate subsribers to bin camera
  for (int i = 1; i <= 6; ++i) {
    const std::string topic = "/ariac/logical_camera_bin" + std::to_string(i);

    const ImageCallback callback = [&, i](LogicalCameraPtr img) {
      g_bin_images[i - 1] = *img;
    };

    bin_camera_subs[i - 1] =
        nh.subscribe<osrf_gear::LogicalCameraImage>(topic, 16, callback);
  }

  ROS_INFO("Subscribing to agv and  quality cameras");
  // generate subscribers for agv and quality camera
  for (int i = 1; i <= 2; ++i) {
    const std::string agv_topic =
        "/ariac/logical_camera_agv" + std::to_string(i);
    const std::string quality_topic =
        "/ariac/quality_control_sensor_" + std::to_string(i);

    const ImageCallback agv_callback = [&, i](LogicalCameraPtr img) {
      g_agv_images[i - 1] = *img;
    };

    const ImageCallback quality_callback = [&, i](LogicalCameraPtr img) {
      g_quality_images[i - 1] = *img;
    };

    agv_camera_subs[i - 1] = nh.subscribe<osrf_gear::LogicalCameraImage>(
        agv_topic, 16, agv_callback);

    quality_camera_subs[i - 1] = nh.subscribe<osrf_gear::LogicalCameraImage>(
        quality_topic, 16, quality_callback);
  }

  ros::ServiceClient begin_client =
      nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  begin_client.waitForExistence();

  std_srvs::Trigger begin_comp;

  const bool send_success = begin_client.call(begin_comp);

  if (!send_success) {
    ROS_ERROR("Competition service call failed!  Goodness Gracious!!");
    return 1;
  }

  const bool start_success = begin_comp.response.success;

  if (!start_success) {
    ROS_WARN("Competition service returned failure: %s",
             begin_comp.response.message.c_str());
    // TODO: Should this exit?
  } else {
    ROS_INFO("Competition service called successfully: %s",
             begin_comp.response.message.c_str());
  }

  ROS_INFO("Before loop");

  arm.go_to_home_pose();
  ros::Duration(1.0).sleep();
  arm.rotate_end_effector(0.0);
  ros::Duration(1.0).sleep();

  ros::Rate r(10);
  while (ros::ok()) {

    if (g_shipment_queue.empty()) {
      r.sleep();
      continue;
    }

    ROS_INFO("Putting together shipment");

    auto current_shipment = g_shipment_queue.front();
    g_shipment_queue.pop();

    const std::string shipment_type = current_shipment.shipment_type;
    const std::string agv_id = current_shipment.agv_id;

    double agv_lin;
    int agv_num;
    std::string agv_camera_frame;

    if (agv_id == "agv1") {
      agv_lin = 2.25;
      agv_num = 1;
      agv_camera_frame = "logical_camera_agv1_frame";
    } else {
      agv_lin = -2.25;
      agv_num = 2;
      agv_camera_frame = "logical_camera_agv2_frame";
    }

    for (const auto &part : current_shipment.products) {
      std::string product_type = part.type;

      osrf_gear::GetMaterialLocations material_loc;
      material_loc.request.material_type = product_type;
      if (!material_location_client.call(material_loc)) {
        ROS_ERROR("Could not call material location service");
        continue;
      }

      int bin_num;
      std::string unit_id;

      for (const osrf_gear::StorageUnit &su :
           material_loc.response.storage_units) {
        if (!std::strstr(su.unit_id.c_str(), "bin")) {
          continue;
        }
        char c = su.unit_id[3];
        bin_num = (int(c) - int('0')) - 1;
        unit_id = su.unit_id;
        break;
      }
      const LogicalCameraImage &img = g_bin_images[bin_num];
      const std::string camera_frame = "logical_camera_" + unit_id + "_frame";

      geometry_msgs::Pose camera_pose, part_pose, blank_pose;
      geometry_msgs::Pose offset_pose, goal_pose;

      goal_pose.position.x = 0;
      goal_pose.position.y = 0;
      goal_pose.position.z = 0;

      camera_pose = img.pose;
      part_pose = img.models.front().pose;

      // yaw_from_pose(part_pose);

      auto tf = get_robot_to_frame(camera_frame);
      tf2::doTransform(blank_pose, offset_pose, tf);

      bool left = bin_num > 2;

      const double linear_offset = 0.6;
      if (left) {
        arm.move_linear_actuator_relative(offset_pose.position.y -
                                          linear_offset);
      } else {
        arm.move_linear_actuator_relative(offset_pose.position.y +
                                          linear_offset);
      }

      ros::Duration(1.0).sleep();

      tf = get_robot_to_frame(camera_frame);

      tf2::doTransform(part_pose, goal_pose, tf);
      tf2::doTransform(blank_pose, camera_pose, tf);

      double begin_rotation = yaw_from_pose(part_pose);

      ROS_INFO("Picking up part with rotation: %f", begin_rotation);

      // pickup part
      arm.pickup_part(goal_pose.position, camera_pose.position, begin_rotation,
                      left, false, true);
      ros::Duration(1.0).sleep();

      // move to agv
      arm.move_linear_actuator(agv_lin);
      ros::Duration(1.0).sleep();

      if (agv_id == "agv1") {
        while (ros::ok() && g_agv1_state != "ready_to_deliver")
          ros::Duration(0.1).sleep();
      } else {
        while (ros::ok() && g_agv2_state != "ready_to_deliver")
          ros::Duration(0.1).sleep();
      }

      geometry_msgs::Pose agv_camera_pose, agv_part_pose;
      tf = get_robot_to_frame(agv_camera_frame);

      std::string tray_frame;
      {
        std::stringstream ss;
        ss << "kit_tray_";
        ss << agv_num;
        tray_frame = ss.str();
      }
      auto tray_tf = get_robot_to_frame(tray_frame);

      tf2::doTransform(blank_pose, agv_camera_pose, tf);
      tf2::doTransform(part.pose, agv_part_pose, tray_tf);

      // ROS_INFO("Attempting to place part at ");

      // agv_part_pose.position.z = agv_camera_pose.position.z - 0.5;
      // agv_part_pose.position.z = -agv_camera_pose.position.z + 0.075;
      agv_part_pose.position.z += 0.05;
      // https://bitbucket.org/osrf/ariac/wiki/2019/frame_specifications
      double end_rotation = yaw_from_pose(agv_part_pose);

      // place part on agv
      arm.pickup_part(agv_part_pose.position, agv_camera_pose.position,
                      end_rotation, agv_id != "agv1", true, false);

      ros::Duration(0.5).sleep();
    }
    ROS_INFO("Done with shipment");

    osrf_gear::AGVControl submit;
    submit.request.shipment_type = shipment_type;

    if (agv_id == "agv1") {
      if (!agv1_client.call(submit)) {
        ROS_ERROR("AGV1 client could not be callled");
      } else if (!submit.response.success) {
        ROS_ERROR("Submission from AGV1 unsuccessful with message: %s",
                  submit.response.message.c_str());
      }
    } else {
      if (!agv2_client.call(submit)) {
        ROS_ERROR("AGV2 client could not be callled");
      } else if (!submit.response.success) {
        ROS_ERROR("Submission from AGV2 unsuccessful with message: %s",
                  submit.response.message.c_str());
      }
    }
  }

  return 0;
}
