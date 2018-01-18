#include "ros/ros.h"
#include "turtlebot3_auto_docking/ChangeNode.h"

bool change_node(turtlebot3_auto_docking::ChangeNode::Request  &req,
                 turtlebot3_auto_docking::ChangeNode::Response &res)
{
  res.change_node = req.next_node;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "change_node");
  ros::NodeHandle node;

  ros::ServiceServer service = node.advertiseService("change_node", change_node);
  ROS_INFO("change_node");
  ros::spin();
  return 0;
}

//
// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "add_two_ints_client");
//   if (argc != 3)
//   {
//     ROS_INFO("usage: add_two_ints_client X Y");
//     return 1;
//   }
//
//   ros::NodeHandle n;
//   ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
//   beginner_tutorials::AddTwoInts srv;
//   srv.request.a = atoll(argv[1]);
//   srv.request.b = atoll(argv[2]);
//   if (client.call(srv))
//   {
//     ROS_INFO("Sum: %ld", (long int)srv.response.sum);
//   }
//   else
//   {
//     ROS_ERROR("Failed to call service add_two_ints");
//     return 1;
//   }
//
//   return 0;
// }
