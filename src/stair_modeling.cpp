// Copyright 2023 Nimrod Curtis
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Standard library includes
#include <sys/resource.h>
#include <memory>

// ROS application/library includes
#include <rclcpp/rclcpp.hpp>

// Custom includes
#include "zion_components/stair_modeling_component.hpp"

int main(int argc, char * argv[])
{ 
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  
  // Initialize any global resources needed by the middleware and the client library.
  rclcpp::init(argc, argv);

  // Build nodes
  rclcpp::NodeOptions options;

  // Enable intraprocess communication
  options.use_intra_process_comms(true); 

  // Set components nodes
  auto stair_modeling = std::make_shared<zion::StairModeling>(options);

  // Set executor and spin
  // Set the desired number of threads for the executor
  rclcpp::ExecutorOptions exec_options;
  
  const size_t num_threads = 8; // Change this number to your desired value
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(exec_options,num_threads);
  executor->add_node(stair_modeling);
  executor->spin();

  // Shutdown
  rclcpp::shutdown();
  return 0;
}