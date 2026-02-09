#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

static int h_low = 0, s_low = 0, v_low = 0;
static int h_high = 179, s_high = 255, v_high = 255;

class HSVFinder : public rclcpp::Node
{
public:
    HSVFinder() : Node("hsv_finder")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10,
            std::bind(&HSVFinder::imageCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&HSVFinder::guiLoop, this));

        cv::namedWindow("HSV Finder", cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("H low",  "HSV Finder", &h_low, 179);
        cv::createTrackbar("H high", "HSV Finder", &h_high, 179);
        cv::createTrackbar("S low",  "HSV Finder", &s_low, 255);
        cv::createTrackbar("S high", "HSV Finder", &s_high, 255);
        cv::createTrackbar("V low",  "HSV Finder", &v_low, 255);
        cv::createTrackbar("V high", "HSV Finder", &v_high, 255);

        RCLCPP_INFO(this->get_logger(), "HSV Finder running (GUI thread safe)");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            latest_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) {
            return;
        }
    }

    void guiLoop()
    {
        if (latest_frame_.empty()) return;

        cv::Mat hsv, mask, display;
        cv::cvtColor(latest_frame_, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(
            hsv,
            cv::Scalar(h_low, s_low, v_low),
            cv::Scalar(h_high, s_high, v_high),
            mask
        );

        cv::bitwise_and(latest_frame_, latest_frame_, display, mask);
        cv::imshow("HSV Finder", display);

        int key = cv::waitKey(1);
        if (key == 's' || key == 'S') {
            saveHSVToYAML();
        }
    }

    void saveHSVToYAML()
    {
        std::string yaml_path =
            std::string(getenv("HOME")) + "/.ros/ball_HSV.yaml";

        std::ofstream file(yaml_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", yaml_path.c_str());
            return;
        }

        file << "ball_chaser:\n";
        file << "  ros__parameters:\n";
        file << "    h_low: " << h_low << "\n";
        file << "    s_low: " << s_low << "\n";
        file << "    v_low: " << v_low << "\n";
        file << "    h_high: " << h_high << "\n";
        file << "    s_high: " << s_high << "\n";
        file << "    v_high: " << v_high << "\n";
        file.close();

        RCLCPP_INFO(this->get_logger(), "HSV saved to %s", yaml_path.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::Mat latest_frame_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HSVFinder>());
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
