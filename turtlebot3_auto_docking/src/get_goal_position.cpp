#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

ros::Publisher cloud_pub;
ros::Subscriber laserscan_sub;

void poseCallback(const sensor_msgs::LaserScan& scan_filtered)
{
   sensor_msgs::PointCloud cloud;
   geometry_msgs::Point32 point;
   laser_geometry::LaserProjection projector;

   projector.projectLaser(scan_filtered, cloud);

   point = cloud.points[0];
   float x = point.x;
   float y = point.y;
   ROS_INFO("%f, %f ", x, y);
   cloud_pub.publish(point);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_goal_position");
  ros::NodeHandle node;

  laserscan_sub = node.subscribe( "/scan_filtered", 10, &poseCallback);
  cloud_pub = node.advertise<geometry_msgs::Point32>("/goal_point", 10);

  ros::spin();
  return 0;
}
