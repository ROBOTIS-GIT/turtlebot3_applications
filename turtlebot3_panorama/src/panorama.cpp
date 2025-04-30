// Copyright (c) 2016, Yujin Robot, Rohan Agrawal.
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
 * @file panorama.cpp
 * @brief Panorama app class and ROS node implementation.
 * @date 2013-08-01
 * @author Younghun Ju, Jihoon Lee, Marcus Liebhardt, Rohan Agrawal, YeonSoo Noh
 */

#include "turtlebot3_panorama/panorama.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
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

  image_transport_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());

  pub_stitched_ = image_transport_->advertise("/panorama", 1);

  sub_camera_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    "/camera/image_raw/compressed",
    1,
    [this](
      const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) -> void
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
      (void)stitcher->stitch(images_, pano);
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

  tf2::Quaternion orientation_q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  double heading = tf2::getYaw(orientation_q);

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
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg)
{
  if (store_image_) {
    try {
      cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

      if (image.empty()) {
        RCLCPP_WARN(this->get_logger(), "Decoded image is empty.");
        return;
      }
      images_.push_back(image);
      store_image_ = false;
    } catch (const cv::Exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "cv::imdecode error: %s", ex.what());
    }
  }
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto options =
    rclcpp::NodeOptions(allocator)
    .start_parameter_services(false)
    .start_parameter_event_publisher(false);

  auto panorama_app = std::make_shared<turtlebot3_panorama::PanoApp>(options);
  panorama_app->setup();

  executor.add_node(panorama_app);
  executor.spin();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
