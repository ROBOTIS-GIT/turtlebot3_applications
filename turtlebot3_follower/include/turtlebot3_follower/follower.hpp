#ifndef FOLLOWER_HPP
#define FOLLOWER_HPP

#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"

class Follower : public rclcpp::Node {
public:
  // using NavigateToPose = nav2_msgs::action::NavigateToPose;
  explicit Follower(const std::string & node_name, const std::string & leader_name);

private:
  void tf_publisher();
  void leader_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void follower_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void send_goal();
  void coordinate_transform();
  geometry_msgs::msg::Quaternion get_rotation(const nav_msgs::msg::Odometry::SharedPtr leader_odom_msg,
    const nav_msgs::msg::Odometry::SharedPtr follower_odom_msg);
  geometry_msgs::msg::TransformStamped target_pose_;
  geometry_msgs::msg::PoseStamped prior_target_pose_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr follower_odom_sub_;
  rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr client_;
  nav_msgs::msg::Odometry::SharedPtr leader_odom_msg_;
  nav_msgs::msg::Odometry::SharedPtr follower_odom_msg_;
  std::string leader_name_;
  std::string follower_name_;
  std::string leader_odom_;
  std::string follower_odom_;
  double follow_distance_;
  double goal_interval_;

  // rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif // FOLLOWER_HPP