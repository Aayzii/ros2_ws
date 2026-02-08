#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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

        // ✅ ASYNC client (NO executor conflict)
        param_client_ =
            std::make_shared<rclcpp::AsyncParametersClient>(
                this, "ball_chaser");

        // ✅ REQUIRED on WSL / X11
        cv::startWindowThread();

        cv::namedWindow("HSV Finder", cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("H low",  "HSV Finder", &h_low, 179);
        cv::createTrackbar("H high", "HSV Finder", &h_high, 179);
        cv::createTrackbar("S low",  "HSV Finder", &s_low, 255);
        cv::createTrackbar("S high", "HSV Finder", &s_high, 255);
        cv::createTrackbar("V low",  "HSV Finder", &v_low, 255);
        cv::createTrackbar("V high", "HSV Finder", &v_high, 255);

        RCLCPP_INFO(this->get_logger(), "HSV Finder running (GUI enabled)");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!param_client_->service_is_ready()) return;

        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) {
            return;
        }

        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(
            hsv,
            cv::Scalar(h_low, s_low, v_low),
            cv::Scalar(h_high, s_high, v_high),
            mask
        );

        cv::imshow("HSV Finder", mask);
        cv::waitKey(1);

        // ✅ Non-blocking parameter update
        param_client_->set_parameters({
            rclcpp::Parameter("h_low", h_low),
            rclcpp::Parameter("s_low", s_low),
            rclcpp::Parameter("v_low", v_low),
            rclcpp::Parameter("h_high", h_high),
            rclcpp::Parameter("s_high", s_high),
            rclcpp::Parameter("v_high", v_high)
        });
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HSVFinder>());
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
