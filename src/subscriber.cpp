#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Subscriber : public rclcpp::Node {
public:
  Subscriber() : Node("subscriber") {
    try {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
          "chatter", 10,
          std::bind(&Subscriber::topic_callback, this, std::placeholders::_1));
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized the Subscriber");
    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error during initialization: " << e.what());
      RCLCPP_FATAL_STREAM(this->get_logger(), "Subscriber may not work!!");
    }
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<Subscriber>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Exception in main: " << e.what());
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), "Node initialization failed!");
  }

  rclcpp::shutdown();

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Shutting Down!! ");
  return 0;
}
