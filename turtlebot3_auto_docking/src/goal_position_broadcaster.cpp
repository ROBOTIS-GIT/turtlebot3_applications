#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point32.h>
#include <math.h>

ros::Subscriber left_point_sub;
ros::Subscriber right_point_sub;

float left_x = 0.0, left_y = 0.0, left_theta = 0.0;
float right_x = 0.0, right_y = 0.0, right_theta = 0.0;
float goal_x = 0.0, goal_y = 0.0, goal_theta = 0.0;

bool goal_point_broadcast(void);

void left_point_Callback(const geometry_msgs::Point32::ConstPtr& left_point)
{
  left_x = left_point->x;
  left_y = left_point->y;
  left_theta = atan2(left_y, left_x);
  ROS_INFO("left_x %f, left_y %f ", left_x, left_y);
}

void right_point_Callback(const geometry_msgs::Point32::ConstPtr& right_point)
{
  right_x = right_point->x;
  right_y = right_point->y;
  right_theta = atan2(right_y, right_x);
  ROS_INFO("right_x %f, right_y %f ", right_x, right_y);
}

bool goal_point_broadcast(void)
{
  tf::TransformBroadcaster broadcaster;
  tf::Transform transform;
  tf::Quaternion quaternion;

  goal_x = (left_x + right_x) / 2;
  goal_y = (left_y + right_y) / 2;
  goal_theta = atan2(goal_y, goal_x);

  transform.setOrigin( tf::Vector3(goal_x - 0.25 * cos(goal_theta), goal_y - 0.25 * sin(goal_theta), 0.0) );
  quaternion.setRPY(0, 0, goal_theta);
  transform.setRotation(quaternion);
  broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time(0), "odom", "goal_point"));

  ROS_INFO("goal_x %f, goal_y %f ", goal_x, goal_y);

  return 1;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_position_broadcaster");
  ros::NodeHandle node;

  ros::Rate rate(500);
  while (node.ok())
  {
    left_point_sub = node.subscribe<geometry_msgs::Point32>( "/left_point", 10, &left_point_Callback);
    right_point_sub = node.subscribe<geometry_msgs::Point32>( "/right_point", 10, &right_point_Callback);
    ros :: spinOnce();

    goal_point_broadcast();
    rate.sleep();
  }
  return 0;
}
