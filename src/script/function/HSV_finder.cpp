#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>

using std::placeholders::_1;

class HSVFinder : public rclcpp::Node
{
public:
    HSVFinder() : Node("hsv_finder")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10, std::bind(&HSVFinder::imageCallback, this, _1));

        hsv_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/ball_chase/hsv", 10);

        // declare parameters for defaults
        this->declare_parameter("hsv_lower_h", 5);
        this->declare_parameter("hsv_lower_s", 120);
        this->declare_parameter("hsv_lower_v", 70);
        this->declare_parameter("hsv_upper_h", 15);
        this->declare_parameter("hsv_upper_s", 255);
        this->declare_parameter("hsv_upper_v", 255);

        gui_enabled_ = (std::getenv("DISPLAY") != nullptr) || (std::getenv("WAYLAND_DISPLAY") != nullptr);

        // initialize thresholds from parameters
        lower_h_ = this->get_parameter("hsv_lower_h").as_int();
        lower_s_ = this->get_parameter("hsv_lower_s").as_int();
        lower_v_ = this->get_parameter("hsv_lower_v").as_int();
        upper_h_ = this->get_parameter("hsv_upper_h").as_int();
        upper_s_ = this->get_parameter("hsv_upper_s").as_int();
        upper_v_ = this->get_parameter("hsv_upper_v").as_int();

        if (gui_enabled_) {
            cv::namedWindow("HSV Controls", cv::WINDOW_AUTOSIZE);
            cv::createTrackbar("LowH", "HSV Controls", &lower_h_, 179);
            cv::createTrackbar("HighH", "HSV Controls", &upper_h_, 179);
            cv::createTrackbar("LowS", "HSV Controls", &lower_s_, 255);
            cv::createTrackbar("HighS", "HSV Controls", &upper_s_, 255);
            cv::createTrackbar("LowV", "HSV Controls", &lower_v_, 255);
            cv::createTrackbar("HighV", "HSV Controls", &upper_v_, 255);
            RCLCPP_INFO(this->get_logger(), "HSV Finder GUI enabled");
        } else {
            RCLCPP_INFO(this->get_logger(), "HSV Finder running headless; use params to tune");
        }

        // publish initial values
        publishAndSave();

        RCLCPP_INFO(this->get_logger(), "HSV finder node started (publishing to /ball_chase/hsv)");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error in HSVFinder");
            return;
        }

        // Create HSV mask from current thresholds
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Scalar lower(lower_h_, lower_s_, lower_v_);
        cv::Scalar upper(upper_h_, upper_s_, upper_v_);
        cv::inRange(hsv, lower, upper, mask);

        // Noise cleanup for nicer preview
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // If GUI enabled, show a small preview and the mask
        if (gui_enabled_) {
            cv::imshow("HSV Controls", frame);
            cv::imshow("Mask", mask);
            cv::waitKey(1);
        }

        // If parameters changed via GUI or externally, publish and save periodically
        if (valuesChanged()) {
            publishAndSave();
        }
    }

    bool valuesChanged()
    {
        bool changed = (lower_h_ != prev_[0]) || (lower_s_ != prev_[1]) || (lower_v_ != prev_[2]) ||
                       (upper_h_ != prev_[3]) || (upper_s_ != prev_[4]) || (upper_v_ != prev_[5]);
        if (changed) {
            prev_[0] = lower_h_;
            prev_[1] = lower_s_;
            prev_[2] = lower_v_;
            prev_[3] = upper_h_;
            prev_[4] = upper_s_;
            prev_[5] = upper_v_;
        }
        return changed;
    }

    void publishAndSave()
    {
        std_msgs::msg::Int32MultiArray msg;
        msg.data = {lower_h_, lower_s_, lower_v_, upper_h_, upper_s_, upper_v_};
        hsv_pub_->publish(msg);

        // write to a simple file for persistence
        std::ofstream ofs("/tmp/hsv_values.yaml");
        if (ofs) {
            ofs << "lower_h: " << lower_h_ << "\n";
            ofs << "lower_s: " << lower_s_ << "\n";
            ofs << "lower_v: " << lower_v_ << "\n";
            ofs << "upper_h: " << upper_h_ << "\n";
            ofs << "upper_s: " << upper_s_ << "\n";
            ofs << "upper_v: " << upper_v_ << "\n";
            ofs.close();
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr hsv_pub_;

    bool gui_enabled_ = false;

    int lower_h_, lower_s_, lower_v_, upper_h_, upper_s_, upper_v_;
    int prev_[6] = { -1, -1, -1, -1, -1, -1 };
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HSVFinder>());
    rclcpp::shutdown();
    return 0;
}
