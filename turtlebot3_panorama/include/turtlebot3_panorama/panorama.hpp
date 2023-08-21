
/*
 * Copyright (c) 2013, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /include/turtlebot3_panorama/panorama.h
 *
 * @brief Panorama app class definition
 *
 * @date 08/01/2013
 *
 * @author Younghun Ju, Jihoon Lee and Marcus Liebhardt
 **/

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/
#ifndef TURTLEBOT3_PANORAMA__PANORAMA_HPP_
#define TURTLEBOT3_PANORAMA__PANORAMA_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
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

  /**
   * Additionally sends out logging information on a ROS topic
   * @param msg logging information
   */
  void setup();

private:
  rclcpp::Node::SharedPtr nh;
  std::map<std::string, std::string> params;

  geometry_msgs::msg::Twist cmd_vel, zero_cmd_vel;
  double snap_interval;
  double angle, last_angle, given_angle, ang_vel_cur;

  bool continuous;

  image_transport::Publisher pub_stitched;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_camera;
  rclcpp::Service<turtlebot3_applications_msgs::srv::TakePanorama>::SharedPtr srv_start_pano;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<image_transport::ImageTransport> it_;
  std::vector<cv::Mat> images_;

  /**
   * turns true, when the pano_ros action goal goes active
   */
  bool is_active;
  /**
   * Tells the pano_ros feedback callback to set is_active to true (starts rotating the robot)
   * This is necessary in order to capture the first picture at the start,
   * since it takes a while to get the first pciture from the Kinect.
   */
  bool go_active;
  /**
   * Default panorama mode used for interaction via rostopic
   */
  int default_mode;
  /**
   * Default panorama angle used for interaction via rostopic
   */
  double default_pano_angle;
  /**
   * Default snap interval used for interaction via rostopic
   */
  double default_snap_interval;
  /**
   * Default rotation velocity used for interaction via rostopic
   */
  double default_rotation_velocity;

  bool store_image;
  /**
   * Starts the creation of a panorama picture via a ROS service
   * @param request specify the details for panorama creation
   * @param response the current state of the app (started, in progress, stopped)
   * @return true, if service call was successful
   */

  void run();

  bool takePanoServiceCb(
    const std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Request> request,
    const std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Response> response);

  void snap();

  void rotate();

  bool hasReachedAngle();

  void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  void cameraImageCb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);
};
}  //namespace turtlebot3_panorama
#endif  // TURTLEBOT3_PANORAMA__PANORAMA_HPP_
