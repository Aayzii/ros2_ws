#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cstdlib>

using std::placeholders::_1;

class BallChaser : public rclcpp::Node
{
public:
    BallChaser() : Node("ball_chaser")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10,
            std::bind(&BallChaser::imageCallback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/ball_chase/mask", 10);

        // Declare HSV parameters with callbacks for real-time tuning
        rcl_interfaces::msg::ParameterDescriptor lower_h_desc;
        lower_h_desc.description = "HSV Lower Hue (0-180)";
        this->declare_parameter("hsv_lower_h", 5, lower_h_desc);

        rcl_interfaces::msg::ParameterDescriptor lower_s_desc;
        lower_s_desc.description = "HSV Lower Saturation (0-255)";
        this->declare_parameter("hsv_lower_s", 120, lower_s_desc);

        rcl_interfaces::msg::ParameterDescriptor lower_v_desc;
        lower_v_desc.description = "HSV Lower Value (0-255)";
        this->declare_parameter("hsv_lower_v", 70, lower_v_desc);

        rcl_interfaces::msg::ParameterDescriptor upper_h_desc;
        upper_h_desc.description = "HSV Upper Hue (0-180)";
        this->declare_parameter("hsv_upper_h", 15, upper_h_desc);

        rcl_interfaces::msg::ParameterDescriptor upper_s_desc;
        upper_s_desc.description = "HSV Upper Saturation (0-255)";
        this->declare_parameter("hsv_upper_s", 255, upper_s_desc);

        rcl_interfaces::msg::ParameterDescriptor upper_v_desc;
        upper_v_desc.description = "HSV Upper Value (0-255)";
        this->declare_parameter("hsv_upper_v", 255, upper_v_desc);

        // Set up parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&BallChaser::onParametersChange, this, _1));

        // Detect whether an X/Wayland display is available (WSLg or X forwarding).
        gui_enabled_ = (std::getenv("DISPLAY") != nullptr) || (std::getenv("WAYLAND_DISPLAY") != nullptr);

        if (gui_enabled_) {
            // Initialize GUI trackbar values from declared parameters
            gui_low_h_ = this->get_parameter("hsv_lower_h").as_int();
            gui_low_s_ = this->get_parameter("hsv_lower_s").as_int();
            gui_low_v_ = this->get_parameter("hsv_lower_v").as_int();
            gui_high_h_ = this->get_parameter("hsv_upper_h").as_int();
            gui_high_s_ = this->get_parameter("hsv_upper_s").as_int();
            gui_high_v_ = this->get_parameter("hsv_upper_v").as_int();

            cv::namedWindow("HSV Controls", cv::WINDOW_AUTOSIZE);
            cv::createTrackbar("LowH", "HSV Controls", &gui_low_h_, 179);
            cv::createTrackbar("HighH", "HSV Controls", &gui_high_h_, 179);
            cv::createTrackbar("LowS", "HSV Controls", &gui_low_s_, 255);
            cv::createTrackbar("HighS", "HSV Controls", &gui_high_s_, 255);
            cv::createTrackbar("LowV", "HSV Controls", &gui_low_v_, 255);
            cv::createTrackbar("HighV", "HSV Controls", &gui_high_v_, 255);

            RCLCPP_INFO(this->get_logger(), "HSV GUI enabled (DISPLAY available)");
        } else {
            RCLCPP_INFO(this->get_logger(), "No DISPLAY found — running headless (use ros2 params to tune)");
        }

        RCLCPP_INFO(this->get_logger(), "Ball chaser node started");
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
    rcl_interfaces::msg::SetParametersResult onParametersChange(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param : parameters) {
            if (param.get_name().find("hsv_") == 0) {
                RCLCPP_INFO(this->get_logger(), "Parameter '%s' changed to: %s", 
                    param.get_name().c_str(), param.value_to_string().c_str());
            }
        }
        return result;
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

        // Get current HSV parameters (prefer GUI values when available)
        int lower_h, lower_s, lower_v, upper_h, upper_s, upper_v;
        if (gui_enabled_) {
            lower_h = gui_low_h_;
            lower_s = gui_low_s_;
            lower_v = gui_low_v_;
            upper_h = gui_high_h_;
            upper_s = gui_high_s_;
            upper_v = gui_high_v_;

            // Mirror GUI values to ROS parameters so tooling stays in sync
            this->set_parameter(rclcpp::Parameter("hsv_lower_h", lower_h));
            this->set_parameter(rclcpp::Parameter("hsv_lower_s", lower_s));
            this->set_parameter(rclcpp::Parameter("hsv_lower_v", lower_v));
            this->set_parameter(rclcpp::Parameter("hsv_upper_h", upper_h));
            this->set_parameter(rclcpp::Parameter("hsv_upper_s", upper_s));
            this->set_parameter(rclcpp::Parameter("hsv_upper_v", upper_v));
        } else {
            lower_h = this->get_parameter("hsv_lower_h").as_int();
            lower_s = this->get_parameter("hsv_lower_s").as_int();
            lower_v = this->get_parameter("hsv_lower_v").as_int();
            upper_h = this->get_parameter("hsv_upper_h").as_int();
            upper_s = this->get_parameter("hsv_upper_s").as_int();
            upper_v = this->get_parameter("hsv_upper_v").as_int();
        }

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
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    // GUI-controlled HSV values (used only if a display is available)
    int gui_low_h_, gui_high_h_, gui_low_s_, gui_high_s_, gui_low_v_, gui_high_v_;
    bool gui_enabled_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallChaser>());
    rclcpp::shutdown();
    return 0;
}
