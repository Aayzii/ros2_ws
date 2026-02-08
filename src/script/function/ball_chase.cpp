#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <std_msgs/msg/int32_multi_array.hpp>

using std::placeholders::_1;

class BallChaser : public rclcpp::Node
{
public:
    BallChaser() : Node("ball_chaser")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10,
            std::bind(&BallChaser::imageCallback, this, _1));

        // Subscribe to HSV values published by HSV_finder node
        hsv_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/ball_chase/hsv", 10,
            std::bind(&BallChaser::hsvCallback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/ball_chase/mask", 10);

        // initialize default HSV thresholds; these will be overwritten
        // when HSV_finder publishes values on /ball_chase/hsv
        lower_h_ = 5;
        lower_s_ = 120;
        lower_v_ = 70;
        upper_h_ = 15;
        upper_s_ = 255;
        upper_v_ = 255;

        RCLCPP_INFO(this->get_logger(), "Ball chaser node started (subscribing to /ball_chase/hsv)");
        RCLCPP_INFO(this->get_logger(), "\n========== HSV TUNING ==========");
        RCLCPP_INFO(this->get_logger(), "Use these commands to adjust HSV values in real-time:");
        RCLCPP_INFO(this->get_logger(), "  ros2 param set /ball_chaser hsv_lower_h 5");
        RCLCPP_INFO(this->get_logger(), "  ros2 param set /ball_chaser hsv_upper_h 15");
        RCLCPP_INFO(this->get_logger(), "  ros2 param set /ball_chaser hsv_lower_s 120");
        RCLCPP_INFO(this->get_logger(), "  ros2 param set /ball_chaser hsv_upper_s 255");
        RCLCPP_INFO(this->get_logger(), "================================");
        RCLCPP_INFO(this->get_logger(), "Mask published on: /ball_chase/mask");
        RCLCPP_INFO(this->get_logger(), "View with: ros2 topic echo /ball_chase/mask");
    }

private:
    void hsvCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 6) {
            lower_h_ = msg->data[0];
            lower_s_ = msg->data[1];
            lower_v_ = msg->data[2];
            upper_h_ = msg->data[3];
            upper_s_ = msg->data[4];
            upper_v_ = msg->data[5];
            RCLCPP_DEBUG(this->get_logger(), "Received HSV: %d %d %d - %d %d %d",
                lower_h_, lower_s_, lower_v_, upper_h_, upper_s_, upper_v_);
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error");
            return;
        }

        // Use latest HSV thresholds received from HSV_finder (or defaults)
        int lower_h = lower_h_;
        int lower_s = lower_s_;
        int lower_v = lower_v_;
        int upper_h = upper_h_;
        int upper_s = upper_s_;
        int upper_v = upper_v_;

        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Use dynamic HSV values
        cv::Scalar lower(lower_h, lower_s, lower_v);
        cv::Scalar upper(upper_h, upper_s, upper_v);
        cv::inRange(hsv, lower, upper, mask);

        // Noise cleanup
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // Publish the mask image for visualization
        auto mask_msg = cv_bridge::CvImage(msg->header, "mono8", mask).toImageMsg();
        mask_pub_->publish(*mask_msg);

        if (gui_enabled_) {
            cv::imshow("Frame", frame);
            cv::imshow("Mask", mask);
            cv::waitKey(1);
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        geometry_msgs::msg::Twist cmd;

        if (contours.empty()) {
            // 🔄 Search
            cmd.angular.z = 0.0;
            cmd.linear.x = 0.0;
            cmd_pub_->publish(cmd);
            return;
        }

        // Largest contour = ball
        auto largest = std::max_element(
            contours.begin(), contours.end(),
            [](auto &a, auto &b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });

        double area = cv::contourArea(*largest);
        cv::Moments m = cv::moments(*largest);

        int cx = int(m.m10 / m.m00);
        int width = frame.cols;
        int error = cx - width / 2;

        // 🧠 Control gains
        cmd.angular.z = -error * 0.002;

        if (area < 1200) {
            cmd.linear.x = 0.25;
        } else {
            cmd.linear.x = 0.0; // Ball reached
        }

        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr hsv_sub_;

    // Stored HSV thresholds (updated from /ball_chase/hsv)
    int lower_h_, lower_s_, lower_v_, upper_h_, upper_s_, upper_v_;
    bool gui_enabled_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallChaser>());
    rclcpp::shutdown();
    return 0;
}
