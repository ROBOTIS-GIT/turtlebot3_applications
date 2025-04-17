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
 * @author Younghun Ju, Jihoon Lee, Marcus Liebhardt and Rohan Agrawal, YeonSoo Noh
 **/

#include "turtlebot3_panorama/panorama.hpp"

#include <cmath>

#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include "rcpputils/filesystem_helper.hpp"


turtlebot3_panorama::PanoApp::PanoApp(const rclcpp::NodeOptions & options)
: rclcpp::Node("turtlebot3_panorama", options)
{
}

turtlebot3_panorama::PanoApp::~PanoApp()
{
}

void turtlebot3_panorama::PanoApp::setup()
{
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

  sub_camera = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw",
    1,
    [this](
      const sensor_msgs::msg::Image::ConstSharedPtr msg) -> void
    {
      cameraImageCb(msg);
    });

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
  if (is_active)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Degrees to go: %f",
      radians_to_degrees(std::abs(given_angle - angle)));
    if ((given_angle - angle) <= 0.0174)
    {
      snap();

      pub_cmd_vel->publish(zero_cmd_vel);

      RCLCPP_INFO(this->get_logger(), "Stitching %lu images", images_.size());

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

      angle = 0.0;
      images_.clear();

      is_active = false;
      cv::imwrite("stitched_result.jpg", pano);
      const char* home_dir = std::getenv("HOME");
      if (!home_dir)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to find HOME environment variable.");
        return;
      }

      std::string save_path = std::string(home_dir) + "/panorama_results/";
      rcpputils::fs::create_directories(rcpputils::fs::path(save_path));

      std::string filename = save_path + "stitched_result.jpg";
      cv::imwrite(filename, pano);

      RCLCPP_INFO(this->get_logger(), "Panorama image successfully saved to: %s", filename.c_str());
    } else {
      if (continuous)
      {
        rotate();

        if (hasReachedAngle())
        {
          pub_cmd_vel->publish(zero_cmd_vel);
          rclcpp::sleep_for(std::chrono::milliseconds(500));

          snap();

          rclcpp::sleep_for(std::chrono::milliseconds(200));
          rotate();
        }
      } else {
        if (hasReachedAngle())
        {
          pub_cmd_vel->publish(zero_cmd_vel);
          rclcpp::sleep_for(std::chrono::milliseconds(2000));
          take_snapshot = true;
        }
        if (take_snapshot)
        {
          if (std::abs(ang_vel_cur) <= 0.005)
          {
            snap();
            take_snapshot = false;
          } else {
            RCLCPP_INFO(
              this->get_logger(),
              "Waiting for robot to stop ... (speed = %f)", ang_vel_cur);
          }
        } else {
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

  if (!images_.empty())
  {
    const char* home_dir = std::getenv("HOME");
    if (!home_dir)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to find HOME environment variable.");
      return;
    }

    std::string save_path = std::string(home_dir) + "/panorama_results/";
    rcpputils::fs::create_directories(rcpputils::fs::path(save_path));

    std::string filename = save_path + "snapshot_" + std::to_string(snapshot_index++) + ".jpg";
    if (cv::imwrite(filename, images_.back()))
    {
      RCLCPP_INFO(this->get_logger(), "Snapshot saved successfully: %s", filename.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save snapshot: %s", filename.c_str());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "No image available. Snapshot not saved.");
  }
}

void turtlebot3_panorama::PanoApp::rotate()
{
  RCLCPP_INFO(this->get_logger(), "rotate");
  pub_cmd_vel->publish(cmd_vel);
}

bool turtlebot3_panorama::PanoApp::hasReachedAngle()
{
  double next_snap_angle = degrees_to_radians(snap_interval) * (snap_count + 1);
  if (angle >= next_snap_angle)
  {
    ++snap_count;
    return true;
  }
  return false;
}

void turtlebot3_panorama::PanoApp::odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
  if (!is_active)
    return;

  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  double heading = tf2::getYaw(q);

  if (!heading_initialized) {
    heading_start = heading;
    heading_initialized = true;
    previous_heading = heading;
    angle = 0.0;
  }

  double delta = heading - previous_heading;

  if (delta > M_PI) delta -= 2.0 * M_PI;
  if (delta < -M_PI) delta += 2.0 * M_PI;

  angle += delta;
  previous_heading = heading;

  ang_vel_cur = msg->twist.twist.angular.z;
}

bool turtlebot3_panorama::PanoApp::takePanoServiceCb(
  const std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Request> request,
  const std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Response> response)
{
  if (is_active &&
    (request->mode == request->CONTINUOUS || request->mode == request->SNAPANDROTATE)) {
    RCLCPP_INFO(this->get_logger(), "Panorama creation already in progress.");
    response->status = request->IN_PROGRESS;
  }else if (is_active && (request->mode == request->STOP)) {
    is_active = false;
    pub_cmd_vel->publish(zero_cmd_vel);
    images_.clear();
    heading_initialized = false;

    RCLCPP_INFO(this->get_logger(), "Panorama creation stopped.");
    response->status = request->STOPPED;
    return true;
  }else if (!is_active && (request->mode == request->STOP)) {
    RCLCPP_INFO(this->get_logger(), "No panorama creation in progress.");
    response->status = request->STOPPED;
    return true;
  }else {
    switch (request->mode)
    {
      case turtlebot3_applications_msgs::srv::TakePanorama::Request::CONTINUOUS:
        continuous = true;
        given_angle = degrees_to_radians(180.0);
        snap_interval = 15.0;
        cmd_vel.angular.z = 0.1;
        RCLCPP_INFO(this->get_logger(), "Mode: CONTINUOUS");
        break;

        case turtlebot3_applications_msgs::srv::TakePanorama::Request::SNAPANDROTATE:
        continuous = false;
        given_angle = degrees_to_radians(180.0);
        snap_interval = 15.0;
        cmd_vel.angular.z = 0.25;
        RCLCPP_INFO(this->get_logger(), "Mode: SNAP_AND_ROTATE");
        break;

      default:
        RCLCPP_WARN(this->get_logger(), "Unknown mode received: %d", request->mode);
        return false;
    }
    heading_initialized = false;
    snap_count = 0;

    is_active = true;
    response->status = request->STARTED;

    RCLCPP_INFO(this->get_logger(), "Panorama creation started.");
  }
  return true;
}

void turtlebot3_panorama::PanoApp::cameraImageCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
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
