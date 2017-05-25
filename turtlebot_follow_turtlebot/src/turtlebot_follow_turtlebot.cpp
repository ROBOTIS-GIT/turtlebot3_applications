#include "turtlebot_follow_turtlebot.h"

using namespace turtlebot_follow_turtlebot;

turtlebotfollowturtlebot::turtlebotfollowturtlebot()
  :nh_priv("~"),
   turtlebot(""),
   target_turtlebot(""),
   target_turtlebot_behind("")
{
  nh_priv.getParam("turtlebot", turtlebot);
  nh_priv.getParam("target_turtlebot", target_turtlebot);
  nh_priv.getParam("target_turtlebot_behind", target_turtlebot_behind);

  turtlebot_vel_pub = node.advertise<geometry_msgs::Twist>(turtlebot+"/cmd_vel", 10);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_follow_turtlebot");
  turtlebotfollowturtlebot tft;

  ros::Rate rate(125);

  while (ros::ok())
  {
    tft.turtlebot_get_position_loop();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

bool turtlebotfollowturtlebot::turtlebot_get_position_loop()
{
  try
  {
    listener.lookupTransform(turtlebot, target_turtlebot_behind, ros::Time(0), transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  x = transform.getOrigin().x();
  y = transform.getOrigin().y();
  q = transform.getRotation();
  tf::Matrix3x3 m(q);
  m.getRPY(roll,pitch,yaw);

  try
  {
    listener.lookupTransform(turtlebot, target_turtlebot, ros::Time(0), transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  x_ = transform.getOrigin().x();
  y_ = transform.getOrigin().y();

  theta =  atan2(y,x);                      //target_turtlebot_behind
  dist  =  sqrt(pow(x, 2) + pow(y, 2));     //target_turtlebot_behind

  theta_ =  atan2(y_,x_);                   //target_turtlebot
  dist_  =  sqrt(pow(x_, 2) + pow(y_, 2));  //target_turtlebot

  switch(robot_state)
  {
    case 0:
      if(dist_>=0.33)
      {
        if( get_theta(10) == true)
        robot_state = 1;
      }
      else
      {
         robot_state = 2;
      }
      break;

    case 1:
      if( get_distance(0.03) == true )
      robot_state = 2;
      break;

    case 2:
      if(dist_>0.3)
      robot_state = 0;
      break;
  }

  turtlebot_vel_pub.publish(vel_msg);
}

bool turtlebotfollowturtlebot::get_distance(double target_dist)
{
  bool ret = false;

  if(dist >target_dist)
  {
    vel_msg.angular.z =  3.0  * theta;
    vel_msg.linear.x  =  0.5  * dist;
  }
  else
  {
    vel_msg.angular.z = 0.0;
    vel_msg.linear.x =  0.0;
    ret = true;
  }
  return ret;
}


bool turtlebotfollowturtlebot::get_theta(double target_deg)
{
  bool ret = false;

  if(theta > DEG2RAD(target_deg) || theta < DEG2RAD(-target_deg) )
  {
    vel_msg.angular.z = 1.0 * theta;
    vel_msg.linear.x  = 0.0;
  }
  else
  {
    vel_msg.angular.z = 0.0;
    vel_msg.linear.x  = 0.0;
    ret = true;
  }
  return ret;
}
