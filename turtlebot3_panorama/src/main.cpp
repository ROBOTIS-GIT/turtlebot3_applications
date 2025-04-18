// Copyright 2023 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Description:
//   Entry point of the TurtleBot3 panorama application. This file initializes
//   the ROS 2 system, configures node options, creates the PanoApp node,
//   and starts the main executor loop for processing callbacks.
//
// Authors: Gilbert, YeonSoo Noh

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "turtlebot3_panorama/panorama.hpp"


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
  panorama_app->setup();

  executor.add_node(panorama_app);
  executor.spin();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
