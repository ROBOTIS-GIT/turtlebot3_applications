#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>

ros::Publisher cmd_vel_pub;
ros::Publisher robot_state_pub;
ros::Subscriber scan_filtered_sub;

geometry_msgs::Twist vel_msg;
sensor_msgs::LaserScan second_filtered;
std_msgs::Int8 robot_state;

float intensities_array[360] = {0};
float ranges_array[360] = {0};

bool catch_docking_station(void);

bool catch_docking_station()
{
  vel_msg.angular.z = 0.1;
  vel_msg.linear.x  = 0.02;

  if(robot_state.data == 1)
  {
    vel_msg.angular.z = 0.0;
    vel_msg.linear.x = 0.0;
  }
}

void scan_filter_callback(const sensor_msgs::LaserScan::ConstPtr &scan_filtered)
{
  float first = 0;
  float second = 0;
  float third = 0;

  for (int i = 0; i<360; i++)
  {
    intensities_array[i] = scan_filtered->intensities[i];
    ranges_array[i] = scan_filtered->ranges[i];

    first = scan_filtered->intensities[i];
    second = scan_filtered->intensities[i+1];
    third = scan_filtered->intensities[i+2];

    if(first == 0 && third == 0)
    {
      intensities_array[i+1] = 0;
      i++;
    }

    if(intensities_array[i] * ranges_array[i] !=0 && intensities_array[i+1] * ranges_array[i+1] !=0)
    {
      robot_state.data = 1;
    }
    ROS_INFO("intensities = %f, ranges = %f", intensities_array[i], ranges_array[i]);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_goal_position");
  ros::NodeHandle node;

  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  robot_state_pub = node.advertise<std_msgs::Int8>("/robot_state", 10);
  scan_filtered_sub = node.subscribe<sensor_msgs::LaserScan>("/scan_filtered", 10, &scan_filter_callback);

  ros::Rate rate(125);
  while (node.ok())
  {
    catch_docking_station();
    cmd_vel_pub.publish(vel_msg);
    robot_state_pub.publish(robot_state);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
