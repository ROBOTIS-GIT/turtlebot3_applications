#include <ros/ros.h>
#include <ros/time.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher left_point_pub;
ros::Publisher right_point_pub;
ros::Publisher final_goal_point_pub;
ros::Publisher check_cloud;
ros::Subscriber laserscan_sub;
ros::Publisher ts_pub;

sensor_msgs::PointCloud cloud;
geometry_msgs::Point32 left_point;
geometry_msgs::Point32 right_point;
geometry_msgs::Point32 final_goal_point;
laser_geometry::LaserProjection projector;
geometry_msgs::TransformStamped ts;

int count = 0;
float left_array[3] = {0, 0, 0};
float right_array[3] = {0, 0, 0};

void poseCallback(const sensor_msgs::LaserScan &scan_filtered)
{
//  if(count == 0)
//  {
   projector.projectLaser(scan_filtered, cloud);

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
      count = 1;
//  }
   ts.header.stamp = ros::Time::now();
   ts.header.frame_id = "base_link";
   ts.child_frame_id = "ts";
   ts.transform.translation.x = final_goal_point.x;
   ts.transform.translation.y = final_goal_point.y;
   ts.transform.translation.z = final_goal_point.z;


   final_goal_point.x = (left_point.x + right_point.x)/2;
   final_goal_point.y = (left_point.y + right_point.y)/2;
   final_goal_point.z = 0.0;

   ROS_INFO("goal_x %f, goal_y %f ", final_goal_point.x, final_goal_point.y);

//   check_cloud.publish(cloud);
//   left_point_pub.publish(left_point);
//   right_point_pub.publish(right_point);
   ts_pub.publish(ts);
   final_goal_point_pub.publish(final_goal_point);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_goal_position");
  ros::NodeHandle node;

  ros::Rate rate(125); // 10 hz

  left_point_pub = node.advertise<geometry_msgs::Point32>("/left_point", 10);
  right_point_pub = node.advertise<geometry_msgs::Point32>("/right_point", 10);
  final_goal_point_pub = node.advertise<geometry_msgs::Point32>("/final_goal_point", 10);
  check_cloud =  node.advertise<sensor_msgs::PointCloud>("/check_cloud", 10);
  ts_pub = node.advertise<geometry_msgs::TransformStamped>("/ts", 10);

  laserscan_sub = node.subscribe("/scan", 10, &poseCallback);

  while (node.ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
