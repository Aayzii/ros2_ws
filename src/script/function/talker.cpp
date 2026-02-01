#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TalkerNode : public rclcpp::Node

{
public:
  TalkerNode() : Node("talker_node")

  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "chatter", 10);
    this->declare_parameter("publish_rate", 0.5);
    double rate = this->get_parameter("publish_rate").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(rate),
      std::bind(&TalkerNode::timer_callback, this));
  }
private:
  void timer_callback()
  {
    count_++;
    auto message = std_msgs::msg::String();
    message.data = "Hello ROS 2! Count: " + std::to_string(count_);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  int count_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TalkerNode>());
  rclcpp::shutdown();
  return 0;
}
