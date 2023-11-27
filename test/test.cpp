// Copyright [2023] <Manav Nagda [manav19@umd.edu]>
/**
 * @file subscriber.cpp
 * @author manav19@umd.edu
 * @brief This is a test code to perform Gtests on the program
 */
#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @brief Creation of the shared node for test.
 *
 * This class sets up a shared node for testing purposes.
 */
class TestNode : public testing::Test {
 protected:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Construct a new test object, which will test the publishers count.
 *
 * This test checks if the number of publishers for a specific topic is correct.
 */
TEST_F(TestNode, test_for_publishers_count) {
  // Create a node for testing
  node_ = std::make_shared<rclcpp::Node>("test_publisher_count");

  // Create a publisher for the 'chatter' topic with a QoS setting
  auto test_publisher =
      node_->create_publisher<std_msgs::msg::String>("chatter", 1.0);

  // Get the number of publishers for the 'chatter' topic
  auto publishers_number = node_->count_publishers("chatter");

  // Check if the number of publishers is as expected (1 in this case)
  EXPECT_EQ(1, static_cast<int>(publishers_number));
}

/**
 * @brief Main function for running Google Test cases for the ROS 2 publisher.
 *
 * This function initializes ROS 2, runs Google Test, shuts down ROS 2, and
 * returns the result.
 */
int main(int argc, char** argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Initialize Google Test
  ::testing::InitGoogleTest(&argc, argv);

  // Run all Google Test cases
  int result = RUN_ALL_TESTS();

  // Shutdown ROS 2
  rclcpp::shutdown();

  // Return the result of the Google Test run
  return result;
}
