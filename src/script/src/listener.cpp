#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ListenerNode : public rclcpp::Node

{
public:
  ListenerNode() : Node("listener_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter",
      10,
      std::bind(&ListenerNode::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ListenerNode>());
  rclcpp::shutdown();
  return 0;
}
