#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

ros::Publisher left_point_pub;
ros::Publisher right_point_pub;
//ros::Publisher check_cloud;
ros::Subscriber laserscan_sub;

void poseCallback(const sensor_msgs::LaserScan& scan)
{
   sensor_msgs::PointCloud cloud;
   geometry_msgs::Point32 left_point;
   geometry_msgs::Point32 right_point;
   laser_geometry::LaserProjection projector;

   projector.projectLaser(scan, cloud);

   // index = cloud.channels[1].values
   // indensity = cloud.channels[0].values
   for(int i=0; i<300; i++)
     {
       float index = 0;
       float next_index = 0;

       if(cloud.channels[1].values[i] != 0.0)
       {
         index = cloud.channels[1].values[i];
         next_index = cloud.channels[1].values[i+1];

         if((cloud.channels[0].values[index] - cloud.channels[0].values[next_index]) < -7000 && cloud.channels[0].values[index] > 10000 )
           {
             left_point = cloud.points[index];
           }
         else if((cloud.channels[0].values[next_index] - cloud.channels[0].values[index]) < -7000 && cloud.channels[0].values[index] > 10000 )
           {
             right_point = cloud.points[index];
           }
       }
     }

  // check_cloud.publish(cloud);
   left_point_pub.publish(left_point);
   right_point_pub.publish(right_point);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_goal_position");
  ros::NodeHandle node;

  left_point_pub = node.advertise<geometry_msgs::Point32>("/left_point", 10);
  right_point_pub = node.advertise<geometry_msgs::Point32>("/right_point", 10);
  //check_cloud =  node.advertise<sensor_msgs::PointCloud>("/check_cloud", 10);
  laserscan_sub = node.subscribe( "/scan", 10, &poseCallback);

  ros::spin();
  return 0;
}
