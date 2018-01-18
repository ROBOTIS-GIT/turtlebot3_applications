#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <float.h>

#define FALSE  0
#define TRUE   1
#define SEARCH 0
#define FIND   1

ros::Publisher       searching_dock_vel_pub;
ros::Publisher       searching_dock_state_pub;
ros::Subscriber      scan_filtered_sub;

geometry_msgs::Twist searching_dock_vel_msg;
std_msgs::Int8       searching_dock_state_msg;

float intensities_array[360]   = {0};
float ranges_array[360]        = {0};
float check_docking_array[360] = {0};
float nan_check                = 0.0;
int robot_state                = 1;
int ret                        = FALSE;

void catch_docking_station(int x);
bool check_docking_station(void);

void catch_docking_station(int x)
{
  if(x == SEARCH)
  {
    switch(robot_state)
    {
      case 1:
        searching_dock_vel_msg.angular.z = 0.2;
        searching_dock_vel_msg.linear.x  = 0.04;
      break;

      case 2:
        searching_dock_vel_msg.angular.z = 0.2;
        searching_dock_vel_msg.linear.x  = -0.04;
      break;
    }
  }
  else if(x == FIND)
  {
    searching_dock_vel_msg.angular.z = 0.0;
    searching_dock_vel_msg.linear.x = 0.0;
  }
}

bool check_docking_station(void)
{
  for(int i=0; i<360; i++)
  {
    // Save the intensities and ranges of the received LaserScan ​​to check_docking_array.
    check_docking_array[i] = intensities_array[i] * ranges_array[i];
    // check_docking_array has a value of nan, we change nan to 0.
    nan_check = isnan(check_docking_array[i]);
    if(nan_check == 1.0)
    {
       check_docking_array[i] = 0;
    }
  }
  //  This is find the dockint station. If the condition is satisfied, ret is returned as 1.
  for(int i=0; i<358; i++)
  {
    if(check_docking_array[i] > 3000 && check_docking_array[i+1] > 3000 && check_docking_array[i+2] > 3000 && check_docking_array[i+3] > 3000)
    {
      ret = TRUE;
    }
    if(ret != TRUE)
    {
      ret = FALSE;
    }
  }
  return ret;
}

void scan_filter_Callback(const sensor_msgs::LaserScan::ConstPtr &scan_filtered)
{
  for (int i = 0; i<360; i++)
  {
    // Receive a scan_filtered value..
    intensities_array[i] = scan_filtered->intensities[i];
    ranges_array[i] = scan_filtered->ranges[i];
  }

  check_docking_station();
  if(ret == TRUE)
  {
    searching_dock_state_msg.data = FIND;
  }
  else if(ret == FALSE)
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
  scan_filtered_sub        = node.subscribe("/scan_filtered", 10, &scan_filter_Callback);

  ros::spin();
  return 0;
}
