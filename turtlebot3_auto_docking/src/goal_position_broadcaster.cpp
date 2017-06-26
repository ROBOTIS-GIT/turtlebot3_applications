#include <ros/ros.h>
#include "turtlebot3_auto_docking/ChangeNode.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Int8.h>
#include <math.h>

#define GET_GOAL        1

ros::Subscriber    goal_position_sub;
ros::Subscriber    get_goal_state_sub;
ros::Publisher     move_goal_state_pub;
ros::ServiceClient change_node_srv;

turtlebot3_auto_docking::ChangeNode change_node;
std_msgs::Int8                      move_goal_state_msg;

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

  move_goal_state_pub = node.advertise<std_msgs::Int8>("/move_goal_state", 10);
  change_node_srv     = node.serviceClient<turtlebot3_auto_docking::ChangeNode>("change_node");
  goal_position_sub   = node.subscribe( "/goal_poisition", 10, &goal_position_Callback);
  get_goal_state_sub  = node.subscribe("/get_goal_state", 10, &get_goal_state_Callback);

  if (change_node_srv.call(change_node))
  {
    ROS_INFO("Sum: %ld", (long int)change_node.response.change_node);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  ros::Rate rate(100);
  while (node.ok())
  {
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    tf::Quaternion quaternion;

    if(move_goal_state_msg.data == GET_GOAL)
    {
      transform.setOrigin( tf::Vector3(goal_position_x - 0.1 * cos(goal_position_theta), goal_position_y - 0.1 * sin(goal_position_theta), 0.0) );
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
