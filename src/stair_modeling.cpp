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
  
  const size_t num_threads = 2; // Change this number to your desired value
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(exec_options,num_threads);
  executor->add_node(stair_modeling);
  executor->spin();

  // Shutdown
  rclcpp::shutdown();
  return 0;
}