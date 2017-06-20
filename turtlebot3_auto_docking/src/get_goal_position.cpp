#include <ros/ros.h>
#include <ros/time.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>

ros::Publisher left_point_pub;
ros::Publisher right_point_pub;
ros::Publisher final_goal_point_pub;
ros::Publisher check_cloud;
ros::Publisher cmd_vel_pub;
ros::Subscriber laserscan_sub;
ros::Subscriber robot_state_sub;
ros::Subscriber turtlebot_position_sub;

sensor_msgs::PointCloud cloud;
geometry_msgs::Point32 left_point;
geometry_msgs::Point32 right_point;
geometry_msgs::Point32 final_goal_point;
geometry_msgs::Twist vel_msg;
std_msgs::Int8 robot_state;
nav_msgs::Odometry odom;
laser_geometry::LaserProjection projector;


int count = 0;
int state = 0;
int j = 0;

void stateCallback(const std_msgs::Int8 &robot_state)
{
  state = robot_state.data;
}

void poseCallback(const sensor_msgs::LaserScan &scan_filtered)
{
 if(state == 1)
{
   if(count == 0)
 {
   projector.projectLaser(scan_filtered, cloud);
  //  index = cloud.channels[1].values
  //  indensity = cloud.channels[0].values
   for(int i=0; i<300; i++)
     {
       float index = 0;
       float next_index = 0;

       if(cloud.channels[1].values[i] != 0.0)
        {
      //    index = cloud.channels[1].values[i];
          j += 1;
      //    next_index = cloud.channels[1].values[i+1];
      //
      //    if(cloud.channels[0].values[index] - cloud.channels[0].values[next_index] < -2000 )
      //      {
      //        left_point = cloud.points[index];
      //      }
      //    else if( cloud.channels[0].values[index] - cloud.channels[0].values[next_index] > 2000 )
      //      {
      //        right_point = cloud.points[index];
      //      }
           final_goal_point.x += cloud.points[i].x;
           final_goal_point.y += cloud.points[i].y;
           final_goal_point.z += cloud.points[i].z;
        }
     }
     final_goal_point.x = (final_goal_point.x)/j;
     final_goal_point.y = (final_goal_point.y)/j;
     final_goal_point.z = (final_goal_point.z)/j;
     count = 1;
   }
   
   ROS_INFO("goal_x %f, goal_y %f ", final_goal_point.x, final_goal_point.y);

   check_cloud.publish(cloud);
   left_point_pub.publish(left_point);
   right_point_pub.publish(right_point);
   final_goal_point_pub.publish(final_goal_point);
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_goal_position");
  ros::NodeHandle node;

  left_point_pub = node.advertise<geometry_msgs::Point32>("/left_point", 10);
  right_point_pub = node.advertise<geometry_msgs::Point32>("/right_point", 10);
  final_goal_point_pub = node.advertise<geometry_msgs::Point32>("/final_goal_point", 10);
  check_cloud =  node.advertise<sensor_msgs::PointCloud>("/check_cloud", 10);
  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  robot_state_sub = node.subscribe("/robot_state", 10, &stateCallback);
  laserscan_sub = node.subscribe("/scan_filtered", 10, &poseCallback);
  turtlebot_position_sub = node.subscribe<nav_msgs::Odometry>("/odom", 10, &odom_callback);


  ros::spin();
  return 0;
}
