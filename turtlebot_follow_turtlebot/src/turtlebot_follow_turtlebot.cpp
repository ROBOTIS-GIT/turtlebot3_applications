#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define WAIT         0
#define MOVE_TURN    1
#define MOVE_FORWARD 2

double x, x_;
double y, y_;
double theta, theta_;
double dist, dist_;
double roll, pitch, yaw;
int8_t robot_state = 0;

bool angle_of_btw_behind_and_tb3(double target_deg);
bool dist_of_btw_behind_and_tb3(double target_dist);

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

  ////////////////////////////////////////get positin target_turtlebot_behind////////////////////////////
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
//    q = transform.getRotation();
//    tf::Matrix3x3 m(q);
//    m.getRPY(roll,pitch,yaw);

    ////////////////////////////////////////get positin target_turtlebot////////////////////////////
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

    //////////////////////////////////////////////////////////////////////////////////////////////////

    theta =  atan2(y,x);                      //target_turtlebot_behind
    dist  =  sqrt(pow(x, 2) + pow(y, 2));     //target_turtlebot_behind

    theta_ =  atan2(y_,x_);                   //target_turtlebot
    dist_  =  sqrt(pow(x_, 2) + pow(y_, 2));  //target_turtlebot

    switch(robot_state)
    {
    case MOVE_TURN:
      if(angle_of_btw_behind_and_tb3(2) == false)
      {
        robot_state = MOVE_TURN;
      }
      else
      {
        if (dist >= 0.10)
        {
          robot_state = MOVE_FORWARD;
        }
      }
      break;

    case MOVE_FORWARD:
      if(dist_of_btw_behind_and_tb3(0.01) == false)
      {
        robot_state = MOVE_FORWARD;
      }
      else
      {
        robot_state = WAIT;
      }
      break;

    case WAIT:
      vel_msg.angular.z = 0.0;
      vel_msg.linear.x  = 0.0;

      if (dist >= 0.10)
      {
        robot_state = MOVE_TURN;
      }
      else
      {
        robot_state = WAIT;
      }
      break;

    default:
      robot_state = WAIT;
      break;
    }

//    ROS_INFO("dist_ = %f, theta_ = %f", dist_, theta_);
//    ROS_INFO("dist = %f, theta = %f", dist, DEG2RAD(theta));
    ROS_INFO("robot_state: %d", robot_state);

    turtle_vel.publish(vel_msg);
    rate.sleep();
  }
  return 0;
}

bool dist_of_btw_behind_and_tb3(double target_dist)
{
  bool ret = false;

  if(dist >= target_dist)
  {
    vel_msg.angular.z =  2.0 * theta;
    vel_msg.linear.x  =  0.7  * dist;
  }
  else
  {
    vel_msg.linear.x =  0.0;

    ret = true;
  }
  return ret;
}


bool angle_of_btw_behind_and_tb3(double target_deg)
{
  bool ret = false;

  if(theta > DEG2RAD(target_deg) || theta < DEG2RAD(-target_deg) )
  {
    vel_msg.angular.z = 3.0 * theta;
    vel_msg.linear.x  = vel_msg.linear.x;
  }
  else
  {
    vel_msg.angular.z = 0.0;

    ret = true;
  }
  return ret;
}
