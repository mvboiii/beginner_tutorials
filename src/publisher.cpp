// Copyright [2023] <Manav Nagda [manav19@umd.edu]>
/**
 * @file publisher.cpp
 * @author manav19@umd.edu
 * @brief This publisher program is used publish custom messages
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>
// ROS 2 includes
#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @brief A simple ROS 2 node that publishes messages and handles service
 * requests.
 */
class Publisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the Publisher class.
   */
  Publisher() : Node("publisher"), count_(0) {
    try {
      // Create a publisher for the 'chatter' topic
      publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

      // Declare a parameter 'frequency' with a default value of 2
      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      param_desc.description =
          "This parameter is updated by the given argument in the launch file "
          "and is "
          "used by the publisher and subscriber";
      this->declare_parameter("frequency", 2, param_desc);

      // Get the value of the 'frequency' parameter
      auto frequency =
          this->get_parameter("frequency").get_parameter_value().get<int>();

      // Log the parameter value
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Parameter 'frequency' set to: " << frequency);

      // Create a timer that calls the timer_callback with the specified
      // frequency
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(int((1000 / frequency))),
          std::bind(&Publisher::timer_callback, this));

      // Log initialization success
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized the Publisher");

      // Create a service server for the 'change_string' service
      server_ = this->create_service<beginner_tutorials::srv::ChangeString>(
          "service_node",
          std::bind(&Publisher::changeString, this, std::placeholders::_1,
                    std::placeholders::_2));

      // Log service server initialization success
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized the Server");
    } catch (const std::exception& e) {
      // Log initialization error and exit
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Initialization error: " << e.what());
      RCLCPP_FATAL_STREAM(this->get_logger(), "Publisher may not work!!");
    }
  }

  /**
   * @brief Callback function for the 'change_string' service.
   *
   * @param request The service request containing the input string.
   * @param response The service response containing the modified string.
   */
  void changeString(
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ChangeString::Response>
          response) {
    response->op = request->ip + " Edited by service";

    // Save the response message for use in the timer callback
    server_resp_message = response->op;

    // Log the service request and response
    RCLCPP_INFO(this->get_logger(), "Received service request\nInput: '%s'",
                request->ip.c_str());
    RCLCPP_INFO(this->get_logger(), "Sending back response: '%s'",
                response->op.c_str());
  }

 private:
  /**
   * @brief Timer callback function. Publishes messages with the modified
   * string.
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = server_resp_message;

    // Log message insertion and publishing
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Inserted message data");
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

    // Publish the message
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;  //!< Timer for periodic publishing
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      publisher_;  //!< ROS 2 publisher
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr
      server_;  //!< ROS 2 service server
  std::string server_resp_message =
      "Pudhil station Dadar";  //!< Response message from the service
  size_t count_;               //!< Counter for internal use
};

/**
 * @brief Main function to create and run the Publisher node.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit code.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  try {
    // Create and run the Publisher node
    auto node = std::make_shared<Publisher>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    // Log initialization error and exit
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
                        "Exception in main(): " << e.what());
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"),
                        "Node initialization failed!!!");
  }

  // Shutdown ROS 2
  rclcpp::shutdown();

  // Log shutdown and exit
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Shutting Down!! ");
  return 0;
}
