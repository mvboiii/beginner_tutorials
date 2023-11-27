// Copyright [2023] <Manav Nagda [manav19@umd.edu]>
/**
 * @file publisher.cpp
 * @author manav19@umd.edu
 * @brief This publiher publishes on chatter and creates tf_frames
 */
#include "beginner_tutorials/srv/change_string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

/**
 * @brief A class representing a ROS 2 node that publishes messages and provides
 * a service.
 */
class Publisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the Publisher class.
   * @param transformations A vector containing parameters for static
   * transformations.
   */
  explicit Publisher(const std::vector<std::string>& transformations)
      : Node("publisher"), count1(0) {
    try {
      // Create a publisher for string messages
      publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

      // Declare a parameter for the publishing frequency
      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      param_desc.description =
          "This parameter is updated by the given argument in the launch file "
          "and is used by the publisher and subscriber";
      this->declare_parameter("frequency", 2, param_desc);

      // Get the frequency parameter value
      auto frequency =
          this->get_parameter("frequency").get_parameter_value().get<int>();

      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Parameter 'frequency' set to: " << frequency);

      // Create a timer for publishing messages based on the frequency
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(1000 / (frequency != 0 ? frequency : 1)),
          std::bind(&Publisher::timer_callback, this));

      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized the Publisher");

      // Create a service for changing the string
      server_ = this->create_service<beginner_tutorials::srv::ChangeString>(
          "service_node",
          std::bind(&Publisher::changeString, this, std::placeholders::_1,
                    std::placeholders::_2));

      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized the Server");

      // Create a broadcaster for static transforms
      tf_static_broadcaster_ =
          std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

      // Generate and send static transforms
      make_transforms(transformations);
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized the transform");
    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Initialization error: " << e.what());
      RCLCPP_FATAL_STREAM(this->get_logger(), "Publisher may not work!!");
    }
  }

  /**
   * @brief Callback function for the service that changes a string.
   * @param request The service request.
   * @param response The service response.
   */
  void changeString(
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ChangeString::Response>
          response) {
    // Modify the received string and send it back as a response
    response->op = request->ip + " Edited by service";
    server_resp_message = response->op;

    RCLCPP_INFO(this->get_logger(), "Received service request\nInput: '%s'",
                request->ip.c_str());
    RCLCPP_INFO(this->get_logger(), "Sending back response: '%s'",
                response->op.c_str());
  }

 private:
  /**
   * @brief Timer callback function that publishes messages.
   */
  void timer_callback() {
    // Prepare and publish a string message
    auto message = std_msgs::msg::String();
    message.data = server_resp_message;

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Inserted message data");
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

    publisher_->publish(message);
  }

  /**
   * @brief Creates static transformations based on the provided parameters.
   * @param transformation A vector containing parameters for static
   * transformations.
   */
  void make_transforms(const std::vector<std::string>& transformation) {
    if (transformation.size() < 6) {
      RCLCPP_WARN(this->get_logger(),
                  "Invalid number of parameters for transformation");
      return;
    }

    // Create a static transform message
    geometry_msgs::msg::TransformStamped t;

    // Set the header information
    t.header.stamp = this->now();
    t.header.frame_id = "world";
    t.child_frame_id = transformation[0];

    // Set the translation information
    t.transform.translation.x = std::stod(transformation[1]);
    t.transform.translation.y = std::stod(transformation[2]);
    t.transform.translation.z = std::stod(transformation[3]);

    // Set the rotation information using Roll, Pitch, and Yaw
    tf2::Quaternion q;
    q.setRPY(std::stod(transformation[4]), std::stod(transformation[5]),
             std::stod(transformation[6]));

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the static transform
    tf_static_broadcaster_->sendTransform(t);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr server_;
  std::string server_resp_message = "Pudhil station Dadar";
  size_t count1;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

/**
 * @brief Main function for the ROS 2 node.
 * @param argc Number of command line arguments.
 * @param argv Command line arguments.
 * @return 0 on success, 1 on failure.
 */
int main(int argc, char* argv[]) {
  if (argc < 7) {
    // Display a warning if the number of parameters is invalid
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                "Invalid number of parameters\nusage: "
                "$ ros2 run beginner_tutorials talker "
                "child_frame_name tx ty tz roll pitch yaw %d",
                argc);
    return 1;
  }

  if (strcmp(argv[1], "world") == 0) {
    // Display an info message if the static turtle name is "world"
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Your static turtle name cannot be world");
    return 1;
  }

  // Copy command line arguments into a vector
  std::vector<std::string> transformations(argv + 1, argv + argc);

  // Initialize ROS 2
  rclcpp::init(argc, argv);
  // Create an instance of the Publisher class
  auto node = std::make_shared<Publisher>(transformations);
  // Spin the node
  rclcpp::spin(node);
  // Shutdown ROS 2
  rclcpp::shutdown();

  // Display a warning message upon shutdown
  RCLCPP_WARN_STREAM(node->get_logger(), "Shutting Down!! " << 4);
  return 0;
}
