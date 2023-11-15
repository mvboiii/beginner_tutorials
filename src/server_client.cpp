#include <chrono>
#include <memory>
#include <string>

#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ServerClient : public rclcpp::Node {
public:
  ServerClient() : Node("server_client") {
    client = this->create_client<beginner_tutorials::srv::ChangeString>(
        "service_node");
  }

  auto getRequest(char **argv) {
    auto request = std::make_shared<beginner_tutorials::srv::ChangeString::Request>();
    request->ip = argv[1];
    return request;
  }

  rclcpp::Client<beginner_tutorials::srv::ChangeString>::SharedPtr client;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto SClient = std::make_shared<ServerClient>();
  
  // Wait for the service to become available
  if (!SClient->client->wait_for_service(5s)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Service not available. Exiting.");
    return 0;
  }

  auto request = SClient->getRequest(argv);
  auto result = SClient->client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(SClient, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Changed string: '%s'",
                result.get()->op.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service change_string");
  }

  rclcpp::shutdown();
  return 0;
}
