// Copyright (c) 2013, Yujin Robot, Rohan Agrawal.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.

//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.

//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file panorama.hpp
 * @brief Panorama app class definition.
 * @date 2013-08-01
 * @author Younghun Ju, Jihoon Lee, Marcus Liebhardt, YeonSoo Noh
 */

#ifndef TURTLEBOT3_PANORAMA__PANORAMA_HPP_
#define TURTLEBOT3_PANORAMA__PANORAMA_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <image_transport/image_transport.hpp>
#ifdef ROS2_HUMBLE
  #include <cv_bridge/cv_bridge.h>
#elif defined(ROS2_LATEST)
  #include <cv_bridge/cv_bridge.hpp>
#endif

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
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_camera_;
  rclcpp::Service<turtlebot3_applications_msgs::srv::TakePanorama>::SharedPtr srv_start_pano_;
  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::Publisher pub_stitched_;
  std::unique_ptr<image_transport::ImageTransport> image_transport_;

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

  void run();

  bool take_pano_service_cb(
    const std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Request> request,
    const std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Response> response);

  void snap();

  void rotate();

  bool has_reached_angle();

  void odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

  void camera_image_cb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);
};

}  // namespace turtlebot3_panorama

#endif  // TURTLEBOT3_PANORAMA__PANORAMA_HPP_
