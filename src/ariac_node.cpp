#include <ros/console.h>
#include <ros/ros.h>

#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/Shipment.h"

#include "sensor_msgs/JointState.h"

#include "trajectory_msgs/JointTrajectory.h"

#include "ur_kinematics/ur_kin.h"

#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/Pose.h"

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

// global services
ros::ServiceClient g_material_location_client;

// global transform listener
tf2_ros::Buffer g_tf_buf;
// tf2_ros::TransformListener g_tf_listener(g_tf_buf);You will find it
// challenging to perform as well as those attending which could have a
// detrimental effect on your grade.

// global data
std::queue<osrf_gear::Order> g_orders;
std::queue<geometry_msgs::Pose> g_transform_queue;

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

// TODO: figure out where to calls this
void process_order(const osrf_gear::Order &order) {
  osrf_gear::GetMaterialLocations get_loc;
  // loop over all shipments in order
  for (const auto &s : order.shipments) {
    // loop over every product in order
    for (const auto &p : s.products) {
      get_loc.request.material_type = p.type;
      const bool send = g_material_location_client.call(get_loc);
      if (!send) {
        ROS_ERROR("Could not call material location client");
        continue;
      }

      for (const osrf_gear::StorageUnit &su : get_loc.response.storage_units) {
        ROS_INFO("Product %s is in %s", p.type.c_str(), su.unit_id.c_str());
        if (su.unit_id == "belt") {

        } else {
          const char c = su.unit_id[3];
          const int bin_num = (int(c) - int('0')) - 1;
          const LogicalCameraImage &img = g_bin_images[bin_num];

          const std::string camera_frame =
              "logical_camera_" + su.unit_id + "_frame";
          ROS_INFO("Getting transform to frame: %s", camera_frame.c_str());
          const auto tf = get_robot_to_frame(camera_frame);

          // for(const )

          geometry_msgs::PoseStamped part_pose, goal_pose, camera_pose;

          camera_pose.pose = img.pose;
          // part_pose.pose = img.models[0].pose;

          for (const auto &model : img.models) {
            part_pose.pose = model.pose;
            tf2::doTransform(part_pose, goal_pose, tf);
            adjust_pose(goal_pose);
            geometry_msgs::Point pos = goal_pose.pose.position;

            if (su.unit_id == "bin4") {
              ROS_INFO("Adding transform to queue: %0f %0f %0f",
                       goal_pose.pose.position.x, goal_pose.pose.position.y,
                       goal_pose.pose.position.z);
              g_transform_queue.push(part_pose.pose);
            }

            ROS_WARN_ONCE("Position of %s: (%f, %f, %f)", p.type.c_str(), pos.x,
                          pos.y, pos.z);
          }
        }
      }
    }
  }
}

void start_competition() {}

void order_callback(const osrf_gear::Order &order) {
  static int n = 0;
  ROS_INFO("Recieved order %d", n++);
  g_orders.push(order);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ariac_node");

  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
  spinner.start();

  tf2_ros::TransformListener tfListener(g_tf_buf);

  g_material_location_client =
      nh.serviceClient<osrf_gear::GetMaterialLocations>(
          "/ariac/material_locations");

  g_material_location_client.waitForExistence(ros::Duration(-1.0));

  Arm arm("arm1");

  auto order_sub = nh.subscribe("/ariac/orders", 2, order_callback);
  // while (ros::ok() &&
  //        !g_tf_buf.canTransform("arm1_base_link",
  //        "logical_camera_bin4_frame",
  //                               ros::Time(0, 0), ros::Duration(1.0)))
  //   ;

  // subscribe to order topic

  // material location service client

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

  {
    ros::Duration d(5.0);
    d.sleep();
  }

  arm.go_to_home_pose();

  ros::Rate r(10);
  while (ros::ok()) {
    if (g_orders.size() > 0) {
      ROS_INFO("Getting order");
      const auto order = g_orders.front();
      process_order(order);
      g_orders.pop();
    }

    if (!g_transform_queue.empty()) {
      auto current_goal = g_transform_queue.front();
      g_transform_queue.pop();

      // current_goal.position.z -= 0.20;

      geometry_msgs::Pose goal_pose;
      goal_pose.position.x = 0;
      goal_pose.position.y = 0;
      goal_pose.position.z = 0;

      arm.move_linear_actuator(-0.2);

      // hardcode this for now
      const auto tf = get_robot_to_frame("logical_camera_bin4_frame");

      tf2::doTransform(current_goal, goal_pose, tf);

      goal_pose.position.z += 0.1;

      arm.go_to_local_pose(goal_pose.position);
      ros::Duration slep(1.0);
      slep.sleep();
    } else {
      arm.go_to_home_pose();
      ros::Duration slep(5.0);
      slep.sleep();
    }

    r.sleep();
  }

  return 0;
}
