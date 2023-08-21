/*
 * Copyright (c) 2016, Yujin Robot, Rohan Agrawal
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
 * @file /include/turtlebot_panorama/panorama.cpp
 *
 * @brief Panorama app class and ROS node implementation
 *
 * @date 08/01/2013
 *
 * @author Younghun Ju, Jihoon Lee, Marcus Liebhardt and Rohan Agrawal
 **/

#include <cmath>
#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "turtlebot3_panorama/panorama.hpp"


turtlebot3_panorama::PanoApp::PanoApp(const rclcpp::NodeOptions & options)
: rclcpp::Node("turtlebot3_panorama", options)
{

}

turtlebot3_panorama::PanoApp::~PanoApp()
{

}

void turtlebot3_panorama::PanoApp::setup()
{
  //***************************
  // public API for the app
  //***************************
  srv_start_pano = this->create_service<turtlebot3_applications_msgs::srv::TakePanorama>(
    "take_pano",
    [this](
      const std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Request> request,
      std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Response> response) -> void
    {
      takePanoServiceCb(request, response);
    });

  it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());

  pub_stitched = it_->advertise("/panorama", 1);
  // sub_camera = it_->subscribe("/image_raw/compressed", 1, &PanoApp::cameraImageCb, this);

  sub_camera = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    "/image_raw/compressed",
    1,
    [this](
      const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) -> void
    {
      cameraImageCb(msg);
    });

  //***************************
  // Robot control
  //***************************
  pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100);
  sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    100,
    [this](
      const nav_msgs::msg::Odometry::ConstSharedPtr msg) -> void
    {
      odomCb(msg);
    });

    cmd_vel.linear.x = 0.0f;
    cmd_vel.linear.y = 0.0f;
    cmd_vel.linear.z = 0.0f;
    cmd_vel.angular.x = 0.0f;
    cmd_vel.angular.y = 0.0f;
    cmd_vel.angular.z = 0.0f;
    zero_cmd_vel = cmd_vel;
    is_active = false;
    continuous = false;
    ang_vel_cur = 0.0;
    given_angle = 0.0;
    angle = 0.0;
    last_angle = 0.0;

    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() -> void
    {
      run();
    }
  );
}

void turtlebot3_panorama::PanoApp::run()
{
  // ros::Rate loop_rate(10);
  double start_time;
  start_time = 0.0;
  bool take_snapshot = false;

    if (is_active)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Degrees to go: %f",
        radians_to_degrees(std::abs(given_angle - angle)));
      if ((given_angle - angle) <= 0.0174) // check, if target angle is reached (< 1 degree)
      {
        snap();

        pub_cmd_vel->publish(zero_cmd_vel);

        RCLCPP_INFO(this->get_logger(), "Stiching %lu images", images_.size());

        cv::Mat pano;
        cv::Stitcher::Mode mode = cv::Stitcher::SCANS;
        cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(mode);
        cv::Stitcher::Status status = stitcher->stitch(images_, pano);
        RCLCPP_INFO(this->get_logger(), "Finished Stitching");

        cv_bridge::CvImage cv_img;
        cv_img.image = pano;
        cv_img.encoding = "bgr8";
        cv_img.header.stamp = this->now();
        pub_stitched.publish(cv_img.toImageMsg());
        RCLCPP_INFO(this->get_logger(), "Publishing Completed Panorama");
        RCLCPP_INFO(this->get_logger(), "Angle: %f", angle);
        RCLCPP_INFO(this->get_logger(), "Last Angle: %f", last_angle);

       	angle = 0.0;
        last_angle = 0.0;
	      images_.clear();

        is_active = false;
      }
      else
      {
        if (continuous) // then snap_interval is a duration
        {
          RCLCPP_INFO(this->get_logger(), "Continuous Mode panorama");
          rotate();
          rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(snap_interval)));

          snap();
          RCLCPP_INFO(this->get_logger(), "Angle Continuous: %f", angle);
          RCLCPP_INFO(this->get_logger(), "Angle Given: %f", given_angle);
        }
        else
        {
          if (hasReachedAngle())
          {
            pub_cmd_vel->publish(zero_cmd_vel); // stop before taking a snapshot
            take_snapshot = true;
          }
          if (take_snapshot)
          {
            if (std::abs(ang_vel_cur) <= 0.01) // wait until robot has stopped
            {
              snap();
              take_snapshot = false;
            }
            else
            {
              RCLCPP_INFO(this->get_logger(), "Waiting for robot to stop ... (speed = %f)", ang_vel_cur);
            }
          }
          else
          {
            rotate();
          }
        }
      }
    }
}

void turtlebot3_panorama::PanoApp::snap()
{
  RCLCPP_INFO(this->get_logger(), "snap");
  store_image = true;
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
}

void turtlebot3_panorama::PanoApp::rotate()
{
  RCLCPP_INFO(this->get_logger(), "rotate");
  pub_cmd_vel->publish(cmd_vel); // rotate a bit
}

bool turtlebot3_panorama::PanoApp::hasReachedAngle()
{
  if (angle > last_angle + degrees_to_radians(snap_interval))
  {
    last_angle = angle;
    return true;
  }
  else
  {
    return false;
  }
}

void turtlebot3_panorama::PanoApp::odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
  static double heading_last = 0.0f;
  double heading = 0.0f;

  Eigen::AngleAxisf angle_axis(Eigen::Quaternionf(msg->pose.pose.orientation.w,
                                                  msg->pose.pose.orientation.x,
                                                  msg->pose.pose.orientation.y,
                                                  msg->pose.pose.orientation.z));
  Eigen::Vector3f axis = angle_axis.axis();

  if (axis(2) > 0.0)
  {
    heading = angle_axis.angle();
  }
  else if (axis(2) < 0.0)
  {
    heading = -1.0 * angle_axis.angle();
  }

  angle += std::abs(wrap_angle(heading - heading_last));
  heading_last = heading;
  ang_vel_cur = msg->twist.twist.angular.z;
}

//*************************
// Public interface
//*************************
bool turtlebot3_panorama::PanoApp::takePanoServiceCb(
  const std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Request> request,
  const std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Response> response)
{
  if (is_active && (request->mode == request->CONTINUOUS || request->mode == request->SNAPANDROTATE))
  {
    RCLCPP_INFO(this->get_logger(), "Panorama creation already in progress.");
    response->status = request->IN_PROGRESS;
  }
  else if (is_active && (request->mode == request->STOP))
  {
    is_active = false;
    RCLCPP_INFO(this->get_logger(), "Panorama creation stopped.");
    response->status = request->STOPPED;
    return true;
  }
  else if (!is_active && (request->mode == request->STOP))
  {
    RCLCPP_INFO(this->get_logger(), "No panorama creation in progress.");
    response->status = request->STOPPED;
    return true;
  }
  else
  {
    if (request->pano_angle <= 0.0)
    {
      RCLCPP_INFO(this->get_logger(), "Specified panorama angle is zero or negative! Panorama creation aborted.");
      return true;
    }
    else if (request->snap_interval <= 0.0)
    {
      RCLCPP_INFO(this->get_logger(), "Specified snapshot interval is zero or negative! Panorama creation aborted.");
      return true;
    }
    else if (request->rot_vel == 0.0)
    {
      RCLCPP_INFO(this->get_logger(), "Specified rotating speed is zero! Panorama creation aborted.");
      return true;
    }
    else
    {
      given_angle = degrees_to_radians(request->pano_angle);
      snap_interval = request->snap_interval;
      cmd_vel.angular.z = request->rot_vel;
    }
    if (request->mode == turtlebot3_applications_msgs::srv::TakePanorama::Request::CONTINUOUS)
    {
      continuous = true;
    }
    else
    {
      continuous = false;
    }
    RCLCPP_INFO(this->get_logger(), "Starting panorama creation.");
    is_active = true;
    response->status = request->STARTED;
  }
  return true;
}

void turtlebot3_panorama::PanoApp::cameraImageCb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg)
{
  if (store_image)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    images_.push_back(cv_ptr->image);
    store_image = false;
  }
}
