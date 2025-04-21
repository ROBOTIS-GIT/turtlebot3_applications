// Copyright 2013 Yujin Robot.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Description:
//   This header defines the PanoApp class used in the TurtleBot3 panorama application.
//   The class implements a ROS2 node that handles camera image acquisition, robot rotation,
//   and panoramic image stitching using OpenCV. It provides service interfaces for
//   triggering panorama capture and manages all motion and data processing required
//   for panorama generation.
//
// Authors: Younghun Ju, Jihoon Lee, Marcus Liebhardt, YeonSoo Noh

#ifndef TURTLEBOT3_PANORAMA__PANORAMA_HPP_
#define TURTLEBOT3_PANORAMA__PANORAMA_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#include "turtlebot3_applications_msgs/srv/take_panorama.hpp"
#include "turtlebot3_panorama/geometry.hpp"


namespace turtlebot3_panorama
{

class PanoApp : public rclcpp::Node
{
public:
  explicit PanoApp(const rclcpp::NodeOptions & options);
  virtual ~PanoApp();

  void setup();

private:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
  rclcpp::Service<turtlebot3_applications_msgs::srv::TakePanorama>::SharedPtr srv_start_pano_;
  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::Publisher pub_stitched_;
  std::unique_ptr<image_transport::ImageTransport> it_;

  std::map<std::string, std::string> params_;
  std::vector<cv::Mat> images_;

  geometry_msgs::msg::Twist cmd_vel_;
  geometry_msgs::msg::Twist zero_cmd_vel_;
  double angle_;
  double given_angle_;
  double ang_vel_cur_;
  double heading_start_ = 0.0;
  double previous_heading_ = 0.0;
  double snap_interval_ = 0.0;

  bool is_active_ = false;
  bool go_active_ = false;
  bool heading_initialized_ = false;
  bool continuous_ = false;
  bool take_snapshot_ = false;
  bool store_image_ = false;

  int snap_count_ = 0;
  int snapshot_index_ = 1;
  int default_mode_;

  void run();

  bool take_pano_service_cb(
    const std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Request> request,
    const std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Response> response);

  void snap();

  void rotate();

  bool has_reached_angle();

  void odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

  void camera_image_cb(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
};

}  // namespace turtlebot3_panorama

#endif  // TURTLEBOT3_PANORAMA__PANORAMA_HPP_
