#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class ImageViewer : public rclcpp::Node
{
public:
    ImageViewer() : Node("image_viewer")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/ball_chase/mask", 10,
            std::bind(&ImageViewer::imageCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Image viewer started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /ball_chase/mask");
        RCLCPP_INFO(this->get_logger(), "Press 'q' in the window to quit");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "mono8")->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error");
            return;
        }

        cv::imshow("Ball Chase Mask", frame);
        
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {  // 'q' or ESC
            RCLCPP_INFO(this->get_logger(), "Quitting...");
            rclcpp::shutdown();
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageViewer>());
    rclcpp::shutdown();
    return 0;
}
