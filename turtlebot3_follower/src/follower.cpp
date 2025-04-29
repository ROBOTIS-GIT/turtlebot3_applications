// Copyright 2025 ROBOTIS CO., LTD.
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
// Author: Hyungyu Kim

#include "turtlebot3_follower/follower.hpp"


Follower::Follower(const std::string & follower_name, const std::string & leader_name)
: Node(follower_name + "_follower_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  leader_name_(leader_name),
  follower_name_(follower_name)
{
  get_parameter("use_sim_time", use_sim_time_);

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  nav2_action_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(
    this,
    follower_name_ + "/follow_path");

  tf_publisher();
  send_path_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&Follower::send_path, this));

  RCLCPP_INFO(
    this->get_logger(),
    "[%s] initialized successfully",
    this->get_name());
}

void Follower::tf_publisher()
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = this->get_clock()->now();
  tf_msg.header.frame_id = this->leader_name_ + "/odom";
  tf_msg.child_frame_id = this->follower_name_ + "/odom";

  if (this->use_sim_time_ == true) {
    tf_msg.transform.translation.x = 0;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
  } else {
    tf_msg.transform.translation.x = -0.14;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
  }
  RCLCPP_INFO(this->get_logger(), "tf publish");
  this->tf_broadcaster_->sendTransform(tf_msg);
}

void Follower::get_target_pose()
{
  try {
    this->target_pose_ = this->tf_buffer_.lookupTransform(
      this->follower_name_ + "/base_link",
      this->leader_name_ + "/base_link",
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Waiting for TF");
  }
}

void Follower::send_path()
{
  get_target_pose();
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = follower_name_ + "/base_link";

  geometry_msgs::msg::PoseStamped first_target_pose;
  first_target_pose.header.stamp = this->get_clock()->now();
  first_target_pose.header.frame_id = follower_name_ + "/base_link";
  first_target_pose.pose.position.x = this->prior_second_target_pose_.pose.position.x;
  first_target_pose.pose.position.y = this->prior_second_target_pose_.pose.position.y;
  first_target_pose.pose.orientation = this->prior_second_target_pose_.pose.orientation;
  path.poses.push_back(first_target_pose);

  geometry_msgs::msg::PoseStamped second_target_pose;
  second_target_pose.header.stamp = this->get_clock()->now();
  second_target_pose.header.frame_id = follower_name_ + "/base_link";
  second_target_pose.pose.position.x = this->target_pose_.transform.translation.x;
  second_target_pose.pose.position.y = this->target_pose_.transform.translation.y;
  second_target_pose.pose.orientation.w = 1.0;
  path.poses.push_back(second_target_pose);
  if (first_target_pose != second_target_pose) {
    this->prior_second_target_pose_ = second_target_pose;
  }

  auto goal_msg = nav2_msgs::action::FollowPath::Goal();
  goal_msg.path = path;

  if (!this->nav2_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Action server not available");
    return;
  }
  this->nav2_action_client_->async_send_goal(goal_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int number = 0;
  if (argc > 1) {
    try {
      number = std::stoi(argv[1]);
    } catch (const std::exception & e) {
      std::cerr << "Invalid number of followers: " << argv[1] << std::endl;
      return 1;
    }
  }

  std::vector<std::shared_ptr<Follower>> followers;
  for (int i = 1; i <= number; ++i) {
    auto node = std::make_shared<Follower>(
      "TB3_" + std::to_string(i + 1),
      "TB3_" + std::to_string(i));
    followers.push_back(node);
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  for (auto & node : followers) {
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
