#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class BallChaser : public rclcpp::Node
{
public:
    BallChaser() : Node("ball_chaser")
    {
        // Declare HSV parameters
        this->declare_parameter("h_low", 0);
        this->declare_parameter("s_low", 0);
        this->declare_parameter("v_low", 0);
        this->declare_parameter("h_high", 179);
        this->declare_parameter("s_high", 255);
        this->declare_parameter("v_high", 255);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10,
            std::bind(&BallChaser::imageCallback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Ball chaser ready (HSV via parameters)");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) {
            return;
        }

        // Read HSV parameters LIVE
        int h_low, s_low, v_low, h_high, s_high, v_high;
        this->get_parameter("h_low", h_low);
        this->get_parameter("s_low", s_low);
        this->get_parameter("v_low", v_low);
        this->get_parameter("h_high", h_high);
        this->get_parameter("s_high", s_high);
        this->get_parameter("v_high", v_high);

        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv,
                    cv::Scalar(h_low, s_low, v_low),
                    cv::Scalar(h_high, s_high, v_high),
                    mask);

        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        geometry_msgs::msg::Twist cmd;

        if (contours.empty()) {
            // Search
            cmd.angular.z = 0.4;
            cmd.linear.x = 0.0;
            cmd_pub_->publish(cmd);
            return;
        }

        auto largest = std::max_element(
            contours.begin(), contours.end(),
            [](auto &a, auto &b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });

        double area = cv::contourArea(*largest);
        cv::Moments m = cv::moments(*largest);

        if (m.m00 == 0) return;

        int cx = int(m.m10 / m.m00);
        int error = cx - frame.cols / 2;

        cmd.angular.z = -error * 0.002;

        if (area < 1500) {
            cmd.linear.x = 0.55;
        } else {
            cmd.linear.x = 0.0;
        }

        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallChaser>());
    rclcpp::shutdown();
    return 0;
}
