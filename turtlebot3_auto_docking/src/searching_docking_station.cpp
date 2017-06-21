#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/LaserScan.h>

#define SEARCH 0
#define STOP   1

ros::Publisher searching_dock_vel_pub;
ros::Publisher robot_state_pub;
ros::Subscriber scan_filtered_sub;

geometry_msgs::Twist searching_dock_vel_msg;
std_msgs::Int8 state;

float intensities_array[360] = {0};
float ranges_array[360] = {0};

bool catch_docking_station(void);

bool catch_docking_station()
{
  if(state.data = SEARCH)
  {
    searching_dock_vel_msg.angular.z = 0.1;
    searching_dock_vel_msg.linear.x  = 0.02;
  }
  else if(state.data == STOP)
  {
    searching_dock_vel_msg.angular.z = 0.0;
    searching_dock_vel_msg.linear.x = 0.0;
  }
}

void scan_filter_callback(const sensor_msgs::LaserScan::ConstPtr &scan_filtered)
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

    if(intensities_array[i] * ranges_array[i] !=0 && intensities_array[i+1] * ranges_array[i+1] !=0)
    {
      state.data = STOP;
    }
    else
      state.data = SEARCH;
      
    ROS_INFO("intensities = %f, ranges = %f", intensities_array[i], ranges_array[i]);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "searching_docking_station");
  ros::NodeHandle node;

  searching_dock_vel_pub = node.advertise<geometry_msgs::Twist>("/searching_docking_station", 10);
  robot_state_pub = node.advertise<std_msgs::Int8>("/robot_state", 10);
  scan_filtered_sub = node.subscribe<sensor_msgs::LaserScan>("/scan_filtered", 10, &scan_filter_callback);

  ros::Rate rate(125);
  while (node.ok())
  {
    searching_dock_vel_pub.publish(searching_dock_vel_msg);
    robot_state_pub.publish(state);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
