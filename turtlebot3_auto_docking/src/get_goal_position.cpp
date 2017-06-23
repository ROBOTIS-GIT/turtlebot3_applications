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

#define SEARCH      0
#define GET_GOAL    1
#define FIND        1

ros::Publisher  goal_poisition_pub;
ros::Publisher  get_goal_state_pub;
ros::Subscriber scan_filtered_sub;
ros::Subscriber searching_dock_state_sub;
ros::Subscriber turtlebot_position_sub;

sensor_msgs::PointCloud cloud;
nav_msgs::Odometry odom;
geometry_msgs::Point32 turtlebot_point;
geometry_msgs::Point32 final_goal_point;
std_msgs::Int8 searching_dock_state_msg;
std_msgs::Int8 get_goal_state_msg;
laser_geometry::LaserProjection projector;

float point_x_sum       = 0.0;
float point_y_sum       = 0.0;
float point_z_sum       = 0.0;
float average_intensity = 0.0;
float intensity_sum     = 0.0;
int j = 0;
int k = 0;
int count = 0;

void get_goal_position(void);

void searching_dock_state_Callback(const std_msgs::Int8::ConstPtr &searching_dock_state_sub_msg)
{
  searching_dock_state_msg.data = searching_dock_state_sub_msg->data;
}

void odom_Callback(const nav_msgs::Odometry &odom)
{
  turtlebot_point.x = odom.pose.pose.position.x;
  turtlebot_point.y = odom.pose.pose.position.y;
  turtlebot_point.z = odom.pose.pose.position.z;
}

void scan_point_Callback(const sensor_msgs::LaserScan &scan_filtered)
{
  projector.projectLaser(scan_filtered, cloud);
  //  index = cloud.channels[1].values
  //  indensity = cloud.channels[0].values
}

void get_goal_position()
{
  if(searching_dock_state_msg.data == FIND)
  {
    if(count == 0)
     {
      for(int i=0; i<360; i++)
      {
         if(cloud.channels[1].values[i] != 0.0)
           {
             j += 1;
             intensity_sum  += cloud.channels[0].values[i];
           }
      }
    average_intensity = intensity_sum / j;

    for(int i=0; i<360; i++)
    {
      if(cloud.channels[1].values[i] != 0.0)
      {
        if(cloud.channels[0].values[i] > average_intensity)
        {
          k += 1;
          point_x_sum += cloud.points[i].x;
          point_y_sum += cloud.points[i].y;
          point_z_sum += cloud.points[i].z;
        }
      }
    }
    final_goal_point.x = {(point_x_sum / k) + turtlebot_point.x};
    final_goal_point.y = {(point_y_sum / k) + turtlebot_point.y};
    final_goal_point.z = {(point_z_sum / k) + turtlebot_point.z};

    ROS_INFO("goal_x %f, goal_y %f ", final_goal_point.x, final_goal_point.y);
    ROS_INFO("average_intensity %f", average_intensity);
  }
    count = 1;
    get_goal_state_msg.data = GET_GOAL;
  }
  // else if(searching_dock_state_msg.data == SEARCH)
  // {
  //   get_goal_state_msg.data = SEARCH;
  // }
  goal_poisition_pub.publish(final_goal_point);
  get_goal_state_pub.publish(get_goal_state_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_goal_position");
  ros::NodeHandle node;

  goal_poisition_pub       = node.advertise<geometry_msgs::Point32>("/goal_poisition", 10);
  get_goal_state_pub       = node.advertise<std_msgs::Int8>("/get_goal_state", 10);
  searching_dock_state_sub = node.subscribe("/searching_dock_state", 10, &searching_dock_state_Callback);
  turtlebot_position_sub   = node.subscribe("/odom", 10, &odom_Callback);
  scan_filtered_sub        = node.subscribe("/scan_filtered", 10, &scan_point_Callback);

  ros::Rate rate(125);
  while(node.ok())
  {
    get_goal_position();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
