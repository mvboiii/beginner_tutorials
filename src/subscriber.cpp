// Copyright [2023] <Manav Nagda [manav19@umd.edu]>
/**
 * @file subscriber.cpp
 * @author manav19@umd.edu
 * @brief This subscriber program is used to print the message
 * sent by the publisher to topic
 */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @brief A simple ROS 2 node that subscribes to messages on the 'chatter'
 * topic.
 */
class Subscriber : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the Subscriber class.
   */
  Subscriber() : Node("subscriber") {
    try {
      // Create a subscription to the 'chatter' topic
      subscription_ = this->create_subscription<std_msgs::msg::String>(
          "chatter", 10,
          std::bind(&Subscriber::topic_callback, this, std::placeholders::_1));

      // Log initialization success
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized the Subscriber");
    } catch (const std::exception& e) {
      // Log initialization error and exit
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Error during initialization: " << e.what());
      RCLCPP_FATAL_STREAM(this->get_logger(), "Subscriber may not work!!");
    }
  }

 private:
  /**
   * @brief Callback function for the 'chatter' topic.
   *
   * @param msg The received message on the 'chatter' topic.
   */
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    // Log the received message
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      subscription_;  //!< ROS 2 subscription
};

/**
 * @brief Main function to create and run the Subscriber node.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit code.
 */
int main(int argc, char* argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  try {
    // Create and run the Subscriber node
    auto node = std::make_shared<Subscriber>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    // Log initialization error and exit
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
                        "Exception in main: " << e.what());
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"),
                        "Node initialization failed!");
  }

  // Shutdown ROS 2
  rclcpp::shutdown();

  // Log shutdown and exit
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Shutting Down!! ");
  return 0;
}
