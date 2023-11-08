/**
 * @file subscriber.cpp
 * @author manav19@umd.edu
 * @brief This subscriber program is used to print the message
 * sent by the publisher to topic
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

/**
 * @brief The class subscriber captures the message whenever published
 * to topic
 *
 */
class Subscriber : public rclcpp::Node {
 public:
  Subscriber() : Node("Subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&Subscriber::topic_callback, this, _1));
  }

 private:
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "Yaatri gan krupiya dhayaan de :%s'",
                msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}
