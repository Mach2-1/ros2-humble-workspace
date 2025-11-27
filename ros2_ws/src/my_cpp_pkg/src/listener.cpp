#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CppListener : public rclcpp::Node
{
public:
  CppListener()
  : Node("cpp_listener")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "cpp_chatter", 10,
      std::bind(&CppListener::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CppListener>());
  rclcpp::shutdown();
  return 0;
}
