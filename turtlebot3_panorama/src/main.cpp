// Copyright 2023 ROBOTIS CO., LTD.
// Authors: Gilbert

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <turtlebot3_panorama/panorama.hpp>


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  std::shared_ptr<turtlebot3_panorama::PanoApp> panorama_app;

  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto options =
    rclcpp::NodeOptions(allocator)
    .start_parameter_services(false)
    .start_parameter_event_publisher(false);

  panorama_app = std::make_shared<turtlebot3_panorama::PanoApp>(options);

  executor.add_node(panorama_app);

  executor.spin();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
