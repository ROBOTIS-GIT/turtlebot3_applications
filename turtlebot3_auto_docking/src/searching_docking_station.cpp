#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/LaserScan.h>
#include <float.h>

#define SEARCH 0
#define FIND   1

ros::Publisher  searching_dock_vel_pub;
ros::Publisher  searching_dock_state_pub;
ros::Subscriber scan_filtered_sub;

geometry_msgs::Twist searching_dock_vel_msg;
std_msgs::Int8       searching_dock_state_msg;

float intensities_array[360] = {0};
float ranges_array[360]      = {0};

void catch_docking_station(int x);

void catch_docking_station(int x)
{
  if(x == SEARCH)
  {
    searching_dock_vel_msg.angular.z = 0.1;
    searching_dock_vel_msg.linear.x  = 0.03;
  }
  else if(x == FIND)
  {
    searching_dock_vel_msg.angular.z = 0.0;
    searching_dock_vel_msg.linear.x = 0.0;
  }
}

void scan_filter_Callback(const sensor_msgs::LaserScan::ConstPtr &scan_filtered)
{
  for (int i = 0; i<360; i++)
  {
    intensities_array[i] = scan_filtered->intensities[i];
    ranges_array[i] = scan_filtered->ranges[i];

    if(intensities_array[i] == 0 && intensities_array[i+2] == 0)
    {
      intensities_array[i+1] = 0;
      i++;
    }
    if((intensities_array[i] * ranges_array[i]) > 0.0 && (intensities_array[i+1] * ranges_array[i+1]) > 0.0 && (intensities_array[i+2] * ranges_array[i+2]) > 0.0)
    {
      searching_dock_state_msg.data = FIND;
    }
  }

  if(searching_dock_state_msg.data != FIND)
  {
    searching_dock_state_msg.data = SEARCH;
  }
  catch_docking_station(searching_dock_state_msg.data);

  searching_dock_state_pub.publish(searching_dock_state_msg);
  searching_dock_vel_pub.publish(searching_dock_vel_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "searching_docking_station");
  ros::NodeHandle node;

  searching_dock_vel_pub   = node.advertise<geometry_msgs::Twist>("/searching_dock_vel", 10);
  searching_dock_state_pub = node.advertise<std_msgs::Int8>("/searching_dock_state", 10);
  scan_filtered_sub        = node.subscribe<sensor_msgs::LaserScan>("/scan_filtered", 10, &scan_filter_Callback);

  ros::spin();
  return 0;
}
