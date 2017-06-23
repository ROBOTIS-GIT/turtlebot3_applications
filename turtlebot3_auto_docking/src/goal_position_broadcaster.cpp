#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Int8.h>
#include <math.h>

#define GET_GOAL        1

ros::Subscriber goal_position_sub;
ros::Subscriber get_goal_state_sub;
ros::Publisher  move_goal_state_pub;

std_msgs::Int8 move_goal_state_msg;

float goal_position_x = 0.0, goal_position_y = 0.0, goal_position_theta = 0.0;

void goal_position_Callback(const geometry_msgs::Point32::ConstPtr& goal_position_msg)
{
  goal_position_x     = goal_position_msg->x;
  goal_position_y     = goal_position_msg->y;
  goal_position_theta = atan2(goal_position_y, goal_position_x);
}

void get_goal_state_Callback(const std_msgs::Int8::ConstPtr& get_goal_state_sub_msg)
{
  move_goal_state_msg.data = get_goal_state_sub_msg->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_position_broadcaster");
  ros::NodeHandle node;

  goal_position_sub   = node.subscribe<geometry_msgs::Point32>( "/goal_poisition", 10, &goal_position_Callback);
  get_goal_state_sub  = node.subscribe<std_msgs::Int8>("/get_goal_state", 10, &get_goal_state_Callback);
  move_goal_state_pub = node.advertise<std_msgs::Int8>("/move_goal_state", 10);

  ros::Rate rate(125);
  while (node.ok())
  {
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    tf::Quaternion quaternion;

    if(move_goal_state_msg.data == GET_GOAL)
    {
      transform.setOrigin( tf::Vector3(goal_position_x - 0.05 * cos(goal_position_theta), goal_position_y - 0.05 * sin(goal_position_theta), 0.0) );
      quaternion.setRPY(0, 0, goal_position_theta);
      transform.setRotation(quaternion);
      broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "goal_poisition"));
      move_goal_state_pub.publish(move_goal_state_msg);
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
