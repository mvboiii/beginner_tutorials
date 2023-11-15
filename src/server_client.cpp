// Copyright [2023] <Manav Nagda [manav19@umd.edu]>
/**
 * @file publisher.cpp
 * @author manav19@umd.edu
 * @brief This ros 2 node acts as a cient for change_string service
 */
#include <chrono>
#include <memory>
#include <string>

#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/**
 * @brief A simple ROS 2 node that acts as a client for the 'change_string'
 * service.
 */
class ServerClient : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the ServerClient class.
   */
  ServerClient() : Node("server_client") {
    // Create a client for the 'change_string' service
    client = this->create_client<beginner_tutorials::srv::ChangeString>(
        "service_node");
  }

  /**
   * @brief Create and return a service request with the input string.
   *
   * @param argv Command-line arguments containing the input string.
   * @return Shared pointer to the service request.
   */
  auto getRequest(char **argv) {
    auto request =
        std::make_shared<beginner_tutorials::srv::ChangeString::Request>();
    request->ip = argv[1];
    return request;
  }
  //!< ROS 2 service client
  rclcpp::Client<beginner_tutorials::srv::ChangeString>::SharedPtr client;
};

/**
 * @brief Main function to create and run the ServerClient node.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit code.
 */
int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create an instance of the ServerClient node
  auto SClient = std::make_shared<ServerClient>();

  // Wait for the 'change_string' service to become available
  if (!SClient->client->wait_for_service(5s)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Service not available. Exiting.");
    return 0;
  }

  // Create a service request
  auto request = SClient->getRequest(argv);

  // Send the service request asynchronously
  auto result = SClient->client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(SClient, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    // Log the changed string received from the service
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Changed string: '%s'",
                result.get()->op.c_str());
  } else {
    // Log an error if the service call fails
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service change_string");
  }

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
