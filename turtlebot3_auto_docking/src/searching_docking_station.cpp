#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher cmd_vel_pub;
ros::Publisher second_filtered_pub;
ros::Subscriber scan_filtered_sub;

geometry_msgs::Twist vel_msg;
sensor_msgs::LaserScan second_filtered;

float sensor_array[360] = {0};

bool catch_docking_station(void);

bool catch_docking_station()
{
  vel_msg.angular.z = 0.1;
  vel_msg.linear.x  = 0.02;
}

void scan_filter_callback(const sensor_msgs::LaserScan::ConstPtr &scan_filtered)
{
  float first = 0;
  float second = 0;
  float third = 0;

  for (int i = 0; i<360; i++)
  {
    second_filtered.intensities[i] = scan_filtered->intensities[i];
    second_filtered.ranges[i] = scan_filtered->ranges[i];

    first = scan_filtered->intensities[i];
    second = scan_filtered->intensities[i+1];
    third = scan_filtered->intensities[i+2];

    // if(first == 0 && third == 0)
    // {
    //   second_filtered.intensities[i+1] = 0;
    //   i++;
    // }
  }
  second_filtered_pub.publish(second_filtered);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_goal_position");
  ros::NodeHandle node;

  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  second_filtered_pub = node.advertise<sensor_msgs::LaserScan>("/second_filtered", 10);
  scan_filtered_sub = node.subscribe<sensor_msgs::LaserScan>("/scan_filtered", 10, &scan_filter_callback);

  ros::Rate rate(1000);
  while (node.ok())
  {
    catch_docking_station();
    cmd_vel_pub.publish(vel_msg);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
