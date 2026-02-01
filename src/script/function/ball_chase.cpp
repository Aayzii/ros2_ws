#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <limits>
#include <cmath>

class BallChaser : public rclcpp::Node
{
public:
  BallChaser() : Node("ball_chaser")
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&BallChaser::scanCallback, this, std::placeholders::_1)
    );

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    target_distance_ = 0.25;   // meters
    max_speed_ = 0.3;
    turn_gain_ = 1.5;

    RCLCPP_INFO(this->get_logger(), "Ball chaser node started");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    float min_dist = std::numeric_limits<float>::infinity();
    float min_angle = 0.0;

    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      float r = msg->ranges[i];
      if (r > msg->range_min && r < min_dist && r < msg->range_max)
      {
        min_dist = r;
        min_angle = msg->angle_min + i * msg->angle_increment;
      }
    }

    geometry_msgs::msg::Twist cmd;

    if (std::isfinite(min_dist))
    {
      // Turn toward the ball
      cmd.angular.z = turn_gain_ * min_angle;

      // Move forward if far enough
      if (min_dist > target_distance_)
      {
        cmd.linear.x = std::min(max_speed_, 0.5 * static_cast<double>(min_dist));
      }
      else
      {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
      }
    }
    else
    {
      // Nothing detected
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }

    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  double target_distance_;
  double max_speed_;
  double turn_gain_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallChaser>());
  rclcpp::shutdown();
  return 0;
}
