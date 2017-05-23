#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

double x, x_;
double y, y_;
double theta, theta_;
double dist, dist_;
double roll, pitch, yaw;
int8_t robot_state = 0;

bool get_theta(double target_deg);
bool get_distance(double target_dist);

std::string turtlebot;
std::string target_turtlebot;
std::string target_turtlebot_behind;

geometry_msgs::Twist vel_msg;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle nh_priv("~");
  ros::NodeHandle node;

  nh_priv.getParam("turtlebot", turtlebot);
  nh_priv.getParam("target_turtlebot", target_turtlebot);
  nh_priv.getParam("target_turtlebot_behind", target_turtlebot_behind);

  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>(turtlebot+"/cmd_vel", 10);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::Quaternion q;

  ros::Rate rate(125);

  while (node.ok())
  {
    try
    {
      listener.lookupTransform(turtlebot, target_turtlebot_behind, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
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
      continue;
    }

    x_ = transform.getOrigin().x();
    y_ = transform.getOrigin().y();

    theta =  atan2(y,x);
    dist  =  sqrt(pow(x, 2) + pow(y, 2));

    theta_ =  atan2(y_,x_);
    dist_  =  sqrt(pow(x_, 2) + pow(y_, 2));

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

    turtle_vel.publish(vel_msg);
    rate.sleep();
  }
  return 0;
}

bool get_distance(double target_dist)
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


bool get_theta(double target_deg)
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
