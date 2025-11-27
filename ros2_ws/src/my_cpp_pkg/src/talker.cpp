#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class CppTalker : public rclcpp::Node
{
public:
  CppTalker()
  : Node("cpp_talker"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("cpp_chatter", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&CppTalker::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello from ROS2 (C++)! Count: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CppTalker>());
  rclcpp::shutdown();
  return 0;
}
