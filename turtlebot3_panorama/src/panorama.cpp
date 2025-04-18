// Copyright 2013 Yujin Robot, Rohan Agrawal.
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
//   This source file implements the PanoApp class for the TurtleBot3 panorama
//   application. It sets up the ROS 2 node and handles image capture, robot rotation,
//   and panoramic image stitching using OpenCV. The class supports continuous and
//   step-by-step panorama modes, and publishes the final panorama as a ROS topic.
//
// Authors: Younghun Ju, Jihoon Lee, Marcus Liebhardt, Rohan Agrawal, YeonSoo Noh

#include "turtlebot3_panorama/panorama.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <cmath>
#include <iostream>

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
  srv_start_pano_ = this->create_service<turtlebot3_applications_msgs::srv::TakePanorama>(
    "take_pano",
    [this](
      const std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Request> request,
      std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Response> response) -> void
    {
      take_pano_service_cb(request, response);
    });

  it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());

  pub_stitched_ = it_->advertise("/panorama", 1);

  sub_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw",
    1,
    [this](
      const sensor_msgs::msg::Image::ConstSharedPtr msg) -> void
    {
      camera_image_cb(msg);
    });

  pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100);

  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    100,
    [this](
      const nav_msgs::msg::Odometry::ConstSharedPtr msg) -> void
    {
      odom_cb(msg);
    });

  cmd_vel_.linear.x = 0.0f;
  cmd_vel_.linear.y = 0.0f;
  cmd_vel_.linear.z = 0.0f;
  cmd_vel_.angular.x = 0.0f;
  cmd_vel_.angular.y = 0.0f;
  cmd_vel_.angular.z = 0.0f;
  zero_cmd_vel_ = cmd_vel_;

  is_active_ = false;
  continuous_ = false;
  ang_vel_cur_ = 0.0;
  given_angle_ = 0.0;
  angle_ = 0.0;

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() -> void {
      run();
    });
}

void turtlebot3_panorama::PanoApp::run()
{
  if (is_active_) {
    RCLCPP_INFO(
      this->get_logger(), "Degrees to go: %f",
      radians_to_degrees(std::abs(given_angle_ - angle_)));

    if ((given_angle_ - angle_) <= 0.0174) {
      snap();

      pub_cmd_vel_->publish(zero_cmd_vel_);

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
      pub_stitched_.publish(cv_img.toImageMsg());

      RCLCPP_INFO(this->get_logger(), "Publishing Completed Panorama");
      RCLCPP_INFO(this->get_logger(), "Angle: %f", angle_);

      angle_ = 0.0;
      images_.clear();
      is_active_ = false;

      cv::imwrite("stitched_result.jpg", pano);

      const char * home_dir = std::getenv("HOME");
      if (!home_dir) {
        RCLCPP_ERROR(this->get_logger(), "Failed to find HOME environment variable.");
        return;
      }

      std::string save_path = std::string(home_dir) + "/panorama_results/";
      rcpputils::fs::create_directories(rcpputils::fs::path(save_path));

      std::string filename = save_path + "stitched_result.jpg";
      cv::imwrite(filename, pano);

      RCLCPP_INFO(this->get_logger(), "Panorama image successfully saved to: %s", filename.c_str());
    } else {
      if (continuous_) {
        rotate();

        if (has_reached_angle()) {
          pub_cmd_vel_->publish(zero_cmd_vel_);
          rclcpp::sleep_for(std::chrono::milliseconds(500));
          snap();
          rclcpp::sleep_for(std::chrono::milliseconds(200));
          rotate();
        }
      } else {
        if (has_reached_angle()) {
          pub_cmd_vel_->publish(zero_cmd_vel_);
          rclcpp::sleep_for(std::chrono::milliseconds(2000));
          take_snapshot_ = true;
        }

        if (take_snapshot_) {
          if (std::abs(ang_vel_cur_) <= 0.005) {
            snap();
            take_snapshot_ = false;
          } else {
            RCLCPP_INFO(
              this->get_logger(), "Waiting for robot to stop ... (speed = %f)", ang_vel_cur_);
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
  store_image_ = true;

  if (!images_.empty()) {
    const char * home_dir = std::getenv("HOME");
    if (!home_dir) {
      RCLCPP_ERROR(this->get_logger(), "Failed to find HOME environment variable.");
      return;
    }

    std::string save_path = std::string(home_dir) + "/panorama_results/";
    rcpputils::fs::create_directories(rcpputils::fs::path(save_path));

    std::string filename = save_path + "snapshot_" + std::to_string(snapshot_index_++) + ".jpg";
    if (cv::imwrite(filename, images_.back())) {
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
  pub_cmd_vel_->publish(cmd_vel_);
}

bool turtlebot3_panorama::PanoApp::has_reached_angle()
{
  double next_snap_angle = degrees_to_radians(snap_interval_) * (snap_count_ + 1);
  if (angle_ >= next_snap_angle) {
    ++snap_count_;
    return true;
  }
  return false;
}

void turtlebot3_panorama::PanoApp::odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  if (!is_active_) {
    return;
  }

  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  double heading = tf2::getYaw(q);

  if (!heading_initialized_) {
    heading_start_ = heading;
    heading_initialized_ = true;
    previous_heading_ = heading;
    angle_ = 0.0;
  }

  double delta = heading - previous_heading_;

  if (delta > M_PI) {delta -= 2.0 * M_PI;}
  if (delta < -M_PI) {delta += 2.0 * M_PI;}

  angle_ += delta;
  previous_heading_ = heading;

  ang_vel_cur_ = msg->twist.twist.angular.z;
}

bool turtlebot3_panorama::PanoApp::take_pano_service_cb(
  const std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Request> request,
  const std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama::Response> response)
{
  if (is_active_ &&
    (request->mode == request->CONTINUOUS || request->mode == request->SNAPANDROTATE))
  {
    RCLCPP_INFO(this->get_logger(), "Panorama creation already in progress.");
    response->status = request->IN_PROGRESS;
  } else if (is_active_ && (request->mode == request->STOP)) {
    is_active_ = false;
    pub_cmd_vel_->publish(zero_cmd_vel_);
    images_.clear();
    heading_initialized_ = false;

    RCLCPP_INFO(this->get_logger(), "Panorama creation stopped.");
    response->status = request->STOPPED;
    return true;
  } else if (!is_active_ && (request->mode == request->STOP)) {
    RCLCPP_INFO(this->get_logger(), "No panorama creation in progress.");
    response->status = request->STOPPED;
    return true;
  } else {
    switch (request->mode) {
      case turtlebot3_applications_msgs::srv::TakePanorama::Request::CONTINUOUS:
        continuous_ = true;
        given_angle_ = degrees_to_radians(180.0);
        snap_interval_ = 15.0;
        cmd_vel_.angular.z = 0.1;
        RCLCPP_INFO(this->get_logger(), "Mode: CONTINUOUS");
        break;

      case turtlebot3_applications_msgs::srv::TakePanorama::Request::SNAPANDROTATE:
        continuous_ = false;
        given_angle_ = degrees_to_radians(180.0);
        snap_interval_ = 15.0;
        cmd_vel_.angular.z = 0.25;
        RCLCPP_INFO(this->get_logger(), "Mode: SNAP_AND_ROTATE");
        break;

      default:
        RCLCPP_WARN(this->get_logger(), "Unknown mode received: %d", request->mode);
        return false;
    }

    heading_initialized_ = false;
    snap_count_ = 0;
    snapshot_index_ = 1;
    is_active_ = true;
    response->status = request->STARTED;

    RCLCPP_INFO(this->get_logger(), "Panorama creation started.");
  }
  return true;
}

void turtlebot3_panorama::PanoApp::camera_image_cb(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (store_image_) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    images_.push_back(cv_ptr->image);
    store_image_ = false;
  }
}
