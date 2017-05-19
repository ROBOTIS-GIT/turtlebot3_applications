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

  geometry_msgs::Twist vel_msg;

bool get_theta(int32_t target_deg);
bool get_distance(double target_dist);
bool get_position(int32_t target_deg_);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;
  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("/turtlebotB/cmd_vel", 10);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::Quaternion q;


  ros::Rate rate(125);

  while (node.ok())
  {
    try
    {
      listener.lookupTransform("/turtlebotB", "/turtleA_behind",  ros::Time(0), transform);
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
      listener.lookupTransform("/turtlebotB", "/turtlebotA",  ros::Time(0), transform);
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
      case 0 :
      if(dist_>0.25)
      {
        if( get_position(15) == true)
        robot_state = 1;
      }
      else
      {
         robot_state = 2;
      }

      break;

      case 1 :

      if( get_distance(0.02) == true )
       robot_state = 2;

      break;

      case 2 :

      if(get_theta(5) == true)
       robot_state = 3;

       break;

       case 3 :

       if(dist_>0.2)
       robot_state = 0;

       break;

    }
ROS_INFO("robot_state = %d", robot_state );
  //  get_theta(5);
  //  get_distance(0.05);
  //  get_position(15);




    turtle_vel.publish(vel_msg);

    // if(dist <0.03)
    // {
    //   vel_msg.angular.z = 0.0;
    //   vel_msg.linear.x =  0.0;
    //   turtle_vel.publish(vel_msg);
    // }
    //
    // else if (dist < 0.1 && dist > 0.05)
    // {
    //   vel_msg.angular.z = 0.5 * yaw;
    //   vel_msg.linear.x  = 0.5 * dist;
    //   {
    //     if(yaw < 0.1 && yaw > -0.1)
    //     {
    //       vel_msg.angular.z = 0.0;
    //       vel_msg.linear.x  = 0.5 * dist;
    //     }
    //   }
    //   turtle_vel.publish(vel_msg);
    // }
   //
   //
  //   if (dist_ < dist)
  //   {
   //
  //     if( RAD2DEG(theta_) > 30)
  //     {
  //       theta_ = DEG2RAD(30);
  //     }
  //     if(  RAD2DEG(theta_) < -30)
  //     {
  //       theta_ = DEG2RAD(-30);
  //     }
   //
  //     vel_msg.angular.z =  3.0 * theta_;
  //     vel_msg.linear.x  =  0.5 * dist_;
   //
  //     turtle_vel.publish(vel_msg);
  //   }
   //
  //  else
  //   {
  //     if( RAD2DEG(theta) > 30)
  //     {
  //       theta = DEG2RAD(30);
  //     }
  //     else if(  RAD2DEG(theta) < -30)
  //     {
  //       theta = DEG2RAD(-30);
  //     }
   //
  //     vel_msg.angular.z =  3.0 * theta;
  //     vel_msg.linear.x  =  0.5 * dist;
   //
  //     turtle_vel.publish(vel_msg);
  //   }

  // if(yaw > DEG2RAD(5) || yaw < DEG2RAD(5) )
  // {
  //   vel_msg.angular.z = yaw;
  //   vel_msg.linear.x  = 0.0;
  //
  //   turtle_vel.publish(vel_msg);
  // }
  //
  // else
  // {
  //   vel_msg.angular.z = 0.0;
  //   vel_msg.linear.x  = 0.0;
  //
  //   turtle_vel.publish(vel_msg);
  // }

     //ROS_INFO("x = %f y = %f theta = %f", transform.getOrigin().x(), transform.getOrigin().y(), atan2(y,x)* 180/M_PI);
     ROS_INFO("yaw = %f", yaw * 180/M_PI);
     ROS_INFO("dist = %f", dist);
     //ROS_INFO("x_ = %f y_ = %f", x_, y_);
     // ROS_INFO("dist = %f theta = %f", dist, theta * 180/M_PI);
     // ROS_INFO("angular.z = %f",3.0 * theta);
    // ROS_INFO("linear = %f angular = %f", vel_msg.linear.x, vel_msg.angular.z);

    rate.sleep();
  }
  return 0;
}

bool get_theta(int32_t target_deg)
{
  bool ret = false;

//  geometry_msgs::Twist vel_msg;

  if(yaw > DEG2RAD(target_deg) || yaw < DEG2RAD(-target_deg) )
  {
    vel_msg.angular.z = yaw;
    vel_msg.linear.x  = 0.0;

  //  turtle_vel.publish(vel_msg);
  }

  else
  {
    vel_msg.angular.z = 0.0;
    vel_msg.linear.x  = 0.0;

  //  turtle_vel.publish(vel_msg);

    ret = true;
  }
  return ret;
}

bool get_distance(double target_dist)
{
  bool ret = false;

  if(dist >target_dist)
  {
    vel_msg.angular.z =  3.0 * theta;
    vel_msg.linear.x  =  0.7 * dist;
  }

  else
  {
    vel_msg.angular.z = 0.0;
    vel_msg.linear.x =  0.0;

    ret = true;
  }

  return ret;
}

bool get_position(int32_t target_deg_)
{
  bool ret = false;

//  geometry_msgs::Twist vel_msg;

  if(theta > DEG2RAD(target_deg_) || theta < DEG2RAD(-target_deg_) )
  {
    vel_msg.angular.z = 1.5 * theta;
    vel_msg.linear.x  = 0.0;

  //  turtle_vel.publish(vel_msg);
  }

  else
  {
    vel_msg.angular.z = 0.0;
    vel_msg.linear.x  = 0.0;

  //  turtle_vel.publish(vel_msg);

    ret = true;
  }
  return ret;
}
