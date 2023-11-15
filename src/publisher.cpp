#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node {
public:
  Publisher() : Node("publisher"), count_(0) {
    try {
      publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      param_desc.description =
          "This parameter is updated by the given argument in the launch file and is "
          "used by the publisher and subscriber";
      this->declare_parameter("frequency", 2, param_desc);
      auto frequency =
          this->get_parameter("frequency").get_parameter_value().get<int>();

      RCLCPP_INFO_STREAM(this->get_logger(), "Parameter 'frequency' set to: " << frequency);
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(int((1000 / frequency))),
          std::bind(&Publisher::timer_callback, this));
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized the Publisher");
      server_ = this->create_service<beginner_tutorials::srv::ChangeString>(
          "service_node",
          std::bind(&Publisher::changeString, this, std::placeholders::_1,
                    std::placeholders::_2));
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized the Server");

    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Initialization error: " << e.what());
      RCLCPP_FATAL_STREAM(this->get_logger(), "Publisher may not work!!");
    }
  }

  void changeString(
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request> request,
      std::shared_ptr<beginner_tutorials::srv::ChangeString::Response> response) {
    response->op = request->ip + " Edited by service";

    server_resp_message = response->op;
    RCLCPP_INFO(this->get_logger(), "Received service request\nInput: '%s'",
                request->ip.c_str());
    RCLCPP_INFO(this->get_logger(), "Sending back response: '%s'",
                response->op.c_str());
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = server_resp_message;

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Inserted message data");
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr server_;
  std::string server_resp_message = "Pudhil station Dadar";
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<Publisher>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Exception in main(): " << e.what());
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), "Node initialization failed!!!");
  }

  rclcpp::shutdown();

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Shutting Down!! ");
  return 0;
}
