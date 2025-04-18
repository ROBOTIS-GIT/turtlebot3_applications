// follower_node.cpp
#include "turtlebot3_follower/follower.hpp"

using namespace std::chrono_literals;

Follower::Follower(const std::string & follower_name, const std::string & leader_name)
: Node(follower_name+"_follower_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  leader_name_(leader_name),
  follower_name_(follower_name),
  leader_odom_(leader_name+"/odom"),
  follower_odom_(follower_name+"/odom"),
  follow_distance_(0.15),
  goal_interval_(1.0)
{
  this->get_parameter("use_sim_time", use_sim_time_);
  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, follower_name_+"/follow_path");
  RCLCPP_INFO(this->get_logger(), "Follower node initialized with NodeOptions");
  tf_publisher();
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&Follower::coordinate_transform, this)
  );
  timer2_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&Follower::send_goal, this)
  );
}

void Follower::send_goal() {
  auto goal_msg = nav2_msgs::action::FollowPath::Goal();
  geometry_msgs::msg::PoseStamped pose;
  nav_msgs::msg::Path path;
  // 이전 목표 위치를 경로에 추가
  pose.header.stamp = this->get_clock()->now();
  pose.header.frame_id = follower_name_+"/base_link";
  pose.pose.position.x = this->prior_target_pose_.pose.position.x;
  pose.pose.position.y = this->prior_target_pose_.pose.position.y;
  pose.pose.orientation = this->prior_target_pose_.pose.orientation;
  path.poses.push_back(pose);
  // 현재 목표 위치를 경로에 추가
  geometry_msgs::msg::PoseStamped pose2;
  pose2.header.stamp = this->get_clock()->now();
  pose2.header.frame_id = follower_name_+"/base_link";
  pose2.pose.position.x = this->target_pose_.transform.translation.x;
  pose2.pose.position.y = this->target_pose_.transform.translation.y;
  pose2.pose.orientation.w = 1.0;
  //골을 쏠 때마다 이전 목표 위치를 저장
  if (pose!= pose2){
    this->prior_target_pose_= pose2;
    this->prior_target_pose_.pose.orientation =this->target_pose_.transform.rotation;
  }
  path.poses.push_back(pose2);

  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = follower_name_+"/base_link";
  goal_msg.path = path;

  if (!this->client_->wait_for_action_server(1s)) {
    RCLCPP_WARN(this->get_logger(), "Action server not available");
    return;
  }
  // RCLCPP_INFO(this->get_logger(),
  // "Sending goal to action server, target_x: [%.2f], target_y: [%.2f]",
  // this->transformStamped.transform.translation.x,
  // this->transformStamped.transform.translation.y);
  this->client_->async_send_goal(goal_msg);
}

void Follower::tf_publisher() {
  // auto& leader_odom_msg = this->leader_odom_msg_;
  // auto& follower_odom_msg = this->follower_odom_msg_;

  // while (){
  //   RCLCPP_INFO(this->get_logger(), "Waiting for odometry messages...");
  //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // }

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = this->get_clock()->now();
  tf_msg.header.frame_id = this->leader_odom_;
  tf_msg.child_frame_id = this->follower_odom_;

  if(this->use_sim_time_ == true){
    tf_msg.transform.translation.x = 0;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
  } else{
    tf_msg.transform.translation.x = -0.14;//follower_odom_msg->pose.pose.position.x - leader_odom_msg->pose.pose.position.x;
    tf_msg.transform.translation.y = 0;//follower_odom_msg->pose.pose.position.y - leader_odom_msg->pose.pose.position.y;
    tf_msg.transform.translation.z = 0;//follower_odom_msg->pose.pose.position.z - leader_odom_msg->pose.pose.position.z;
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
  }
  RCLCPP_INFO(this->get_logger(),"tf publish@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
  this->tf_broadcaster_->sendTransform(tf_msg);

}

void Follower::coordinate_transform()
{
  try {
    this->target_pose_ =
      this->tf_buffer_.lookupTransform(
        this->follower_name_+"/base_link",
        this->leader_name_+"/base_link",
        tf2::TimePointZero);  // 가장 최근의 Transform

    // RCLCPP_INFO(this->get_logger(),
    //   "Transform: %s -> %s \n"
    //   "Translation: [%.2f, %.2f, %.2f]\n"
    //   "Rotation (quaternion): [x: %.2f, y: %.2f, z: %.2f, w: %.2f]",
    //   (this->leader_name_+"/base_link").c_str(),
    //   (this->follower_name_+"/base_link").c_str(),
    //   this->target_pose_.transform.translation.x,
    //   this->target_pose_.transform.translation.y,
    //   this->target_pose_.transform.translation.z,
    //   this->target_pose_.transform.rotation.x,
    //   this->target_pose_.transform.rotation.y,
    //   this->target_pose_.transform.rotation.z,
    //   this->target_pose_.transform.rotation.w
    // );
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not fine TF: %s", ex.what());
  }
}

geometry_msgs::msg::Quaternion Follower::get_rotation(const nav_msgs::msg::Odometry::SharedPtr leader_odom_msg,
  const nav_msgs::msg::Odometry::SharedPtr follower_odom_msg)
{
geometry_msgs::msg::Quaternion leader_q = leader_odom_msg->pose.pose.orientation;
geometry_msgs::msg::Quaternion follower_q = follower_odom_msg->pose.pose.orientation;

tf2::Quaternion tf_leader_q, tf_follower_q;
tf2::fromMsg(leader_q, tf_leader_q);
tf2::fromMsg(follower_q, tf_follower_q);
tf2::Quaternion relative_q = tf_follower_q * tf_leader_q.inverse();  // follower - leader

return tf2::toMsg(relative_q);
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
      "TB3_"/*"follower_*/ + std::to_string(i+1),
      "TB3_"/*"leader_*/ + std::to_string(i));
    followers.push_back(node);
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  for (auto& node : followers) {
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
