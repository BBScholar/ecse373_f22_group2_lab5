#include <ros/console.h>
#include <ros/ros.h>

#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/Shipment.h"

#include <std_srvs/Trigger.h>

#include <array>
#include <iostream>
#include <map>
#include <queue>
#include <string>

std::queue<osrf_gear::Order> g_orders;

void order_callback(const osrf_gear::Order &order) { g_orders.push(order); }

bool start_competition() {
  ros::NodeHandle nh;

  ros::ServiceClient begin_client =
      nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

  std_srvs::Trigger begin_comp;

  bool send_success = begin_client.call(begin_comp);

  if (send_success) {
    ROS_INFO("Competition service called successfully: %s",
             begin_comp.response.message.c_str());
  } else {
    ROS_ERROR("Could not start competition");
  }

  return send_success;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ariac_node");

  ros::NodeHandle nh;

  if (!start_competition()) {
    return 1;
  }

  // subscribe to order topic
  auto order_sub = nh.subscribe("/ariac/orders", 2, order_callback);

  // material location service client
  auto material_location_client =
      nh.serviceClient<osrf_gear::GetMaterialLocations>(
          "/ariac/material_locations");

  const std::array<const char *, 10> logical_camera_names = {
      "logical_camera_bin1",      "logical_camera_bin2",
      "logical_camera_bin3",      "logical_camera_bin4",
      "logic_camera_bin5",        "logical_camera_bin6",
      "logical_camera_agv1",      "logical_camera_agv2",
      "quality_control_sensor_1", "quality_control_sensor_2"};

  ros::spin();

  return 0;
}
