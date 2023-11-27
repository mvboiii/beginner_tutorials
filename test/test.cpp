#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @brief Creaton of the shared node for test
 *
 */
class TestNode : public testing::Test {
 protected:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Construct a new test f object, which will test the publishers count
 *
 */
TEST_F(TestNode, test_for_publishers_count) {
  node_ = std::make_shared<rclcpp::Node>("test_publisher_count");
  auto test_publisher =
      node_->create_publisher<std_msgs::msg::String>("chatter", 1.0);
  auto publishers_number = node_->count_publishers("chatter");
  EXPECT_EQ(1, static_cast<int>(publishers_number));
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
