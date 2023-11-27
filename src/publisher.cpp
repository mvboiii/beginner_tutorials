#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "beginner_tutorials/srv/change_string.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node {
public:
  explicit Publisher(const std::vector<std::string>& transformations)
      : Node("publisher"), count1(0) {
    try {
      publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      param_desc.description =
          "This parameter is updated by the given argument in the launch file "
          "and is used by the publisher and subscriber";
      this->declare_parameter("frequency", 2, param_desc);

      auto frequency =
          this->get_parameter("frequency").get_parameter_value().get<int>();

      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Parameter 'frequency' set to: " << frequency);

      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(1000 / (frequency != 0 ? frequency : 1)),
          std::bind(&Publisher::timer_callback, this));

      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized the Publisher");

      server_ = this->create_service<beginner_tutorials::srv::ChangeString>(
          "service_node",
          std::bind(&Publisher::changeString, this, std::placeholders::_1,
                    std::placeholders::_2));

      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized the Server");

      tf_static_broadcaster_ =
          std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

      make_transforms(transformations);
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized the transform");
    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Initialization error: " << e.what());
      RCLCPP_FATAL_STREAM(this->get_logger(), "Publisher may not work!!");
    }
  }

  void changeString(
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ChangeString::Response>
          response) {
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

  void make_transforms(const std::vector<std::string>& transformation) {
    if (transformation.size() < 6) {
      RCLCPP_WARN(this->get_logger(),
                   "Invalid number of parameters for transformation");
      return;
    }

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->now();
    t.header.frame_id = "world";
    t.child_frame_id = transformation[0];

    // Assuming child_frame_name is the first parameter
    t.transform.translation.x = std::stod(transformation[1]);
    t.transform.translation.y = std::stod(transformation[2]);
    t.transform.translation.z = std::stod(transformation[3]);
    
    tf2::Quaternion q;
    q.setRPY(std::stod(transformation[4]), std::stod(transformation[5]),
             std::stod(transformation[6]));
    
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr server_;
  std::string server_resp_message = "Pudhil station Dadar";
  size_t count1;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char* argv[]) {
  if (argc < 7) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                "Invalid number of parameters\nusage: "
                "$ ros2 run beginner_tutorials talker "
                "child_frame_name tx ty tz roll pitch yaw %d",
                argc);
    return 1;
  }

  if (strcmp(argv[1], "world") == 0) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Your static turtle name cannot be world");
    return 1;
  }

  // Copy command line arguments into a vector
  std::vector<std::string> transformations(argv + 1, argv + argc);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<Publisher>(transformations);
  rclcpp::spin(node);
  rclcpp::shutdown();

  RCLCPP_WARN_STREAM(node->get_logger(), "Shutting Down!! " << 4);
  return 0;
}
