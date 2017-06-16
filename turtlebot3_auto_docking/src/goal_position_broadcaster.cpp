#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point32.h>
#include <math.h>

ros::Subscriber final_goal_point_sub;
float goal_x = 0.0, goal_y = 0.0, goal_theta = 0.0;

void final_goal_point_Callback(const geometry_msgs::Point32::ConstPtr& final_goal_point)
{
  goal_x = final_goal_point->x;
  goal_y = final_goal_point->y;
  goal_theta = atan2(goal_y, goal_x);

  ROS_INFO("goal_x %f, goal_y %f ", goal_x, goal_y);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_position_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster broadcaster;
  tf::Transform transform;
  tf::Quaternion quaternion;

  final_goal_point_sub = node.subscribe<geometry_msgs::Point32>( "/final_goal_point", 10, &final_goal_point_Callback);

  ros::Rate rate(125.0);
  while (node.ok())
  {
    transform.setOrigin( tf::Vector3(goal_x , goal_y , 0.0) );
    quaternion.setRPY(0, 0, goal_theta);
    transform.setRotation(quaternion);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "final_goal_point"));

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
