// system
#include <sys/resource.h>

// Custom includes
// #include "zion_broadcaster.hpp"
#include "stair_modeling/zion_broadcaster.hpp"

namespace zion
{
  ZionBroadcaster::ZionBroadcaster(const rclcpp::NodeOptions &options)
    : Node("zion_broadcaster",options)
  { 

      RCLCPP_INFO(get_logger(), "********************************");
      RCLCPP_INFO(get_logger(), "      Zion broadcaster node ");
      RCLCPP_INFO(get_logger(), "********************************");
      RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
      RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
      RCLCPP_INFO(get_logger(), "********************************");

      // Loading parameters
      loadParams();

      // Initialize the transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      rclcpp::QoS qos_profile(100);
      // qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>( "/tf", qos_profile,
          std::bind(&ZionBroadcaster::tfCallback, this, std::placeholders::_1));

      tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }

  ZionBroadcaster::~ZionBroadcaster() {
      RCLCPP_DEBUG(get_logger(), "Destroying node");
  }


  void ZionBroadcaster::loadParams()
  {   
      RCLCPP_INFO(get_logger(),"************   Parameters   ************");

      // Frame IDs
      this->declare_parameter("frame_ids.map_frame", "map");
      this->get_parameter("frame_ids.map_frame", map_frame_);
      RCLCPP_INFO(get_logger(),"* map_frame: '%s'", map_frame_.c_str());

      this->declare_parameter("frame_ids.base_frame", "zedm_base_link");
      this->get_parameter("frame_ids.base_frame", base_frame_);
      RCLCPP_INFO(get_logger(),"* base_frame: '%s'", base_frame_.c_str());

      this->declare_parameter("frame_ids.output_cloud_frame", "zedm_base_link_projected");      
      this->get_parameter("frame_ids.output_cloud_frame", output_frame_);
      RCLCPP_INFO(get_logger(),"* output_cloud_frame: '%s'", output_frame_.c_str());

  }


  void ZionBroadcaster::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr tf_msg)
  {
      (void)tf_msg;

      // Initialized frames
      std::string fromFrameRel = base_frame_;
      std::string toFrameRel = map_frame_;
      geometry_msgs::msg::TransformStamped base2map;
      geometry_msgs::msg::TransformStamped base_projected2map;
      geometry_msgs::msg::TransformStamped base_projected2camera;

      if (tf_buffer_->canTransform(fromFrameRel, toFrameRel, tf2::TimePointZero))
      {
          try {
            base2map = tf_buffer_->lookupTransform(
              toFrameRel, fromFrameRel,
              tf2::TimePointZero);

            // Extract the rotation from existing transform
            tf2::Transform tf2_transform;
            tf2::fromMsg(base2map.transform, tf2_transform);
            tf2::Vector3 translation(tf2_transform.getOrigin());
            tf2::Matrix3x3 rotation_matrix(tf2_transform.getRotation());
            double roll, pitch, yaw;
            rotation_matrix.getRPY(roll, pitch, yaw);
        
            // Create a new transform to broadcast
            base_projected2map.header.stamp = this->get_clock()->now();
            base_projected2map.header.frame_id = toFrameRel;

            base_projected2map.child_frame_id = output_frame_;

            // Set rotation 
            tf2::Quaternion new_rotation;
            new_rotation.setRPY(0, 0, yaw);
            new_rotation.normalized();

            // Set base_projected->map transform 
            base_projected2map.transform.translation = base2map.transform.translation;
            base_projected2map.transform.rotation = tf2::toMsg(new_rotation);

            // Broadcasr transform
            tf_broadcaster_->sendTransform(base_projected2map);
            
          }
          catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
              this->get_logger(), "Could not transform %s to %s: %s",
              fromFrameRel.c_str(), toFrameRel.c_str(), ex.what());
            return;
          }
      }
  }
} // namespace


int main(int argc, char * argv[])
{ 
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  
  // Initialize any global resources needed by the middleware and the client library.
  rclcpp::init(argc, argv);

  // Build nodes
  rclcpp::NodeOptions options;

  // Set node
  auto broadcaster = std::make_shared<zion::ZionBroadcaster>(options);

  // Run executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(broadcaster);

  auto executor_thread = std::thread(
    [&](){
      int nice = -5;
      setpriority(PRIO_PROCESS, gettid(),nice);
      executor->spin();
    }
  );

  executor_thread.join();

  // Shutdown
  rclcpp::shutdown();
  return 0;
}

