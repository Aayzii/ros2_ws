#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class BallDetector : public rclcpp::Node
{
public:
  BallDetector() : Node("ball_detector")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 10,
      std::bind(&BallDetector::imageCallback, this, _1));

    // Create trackbars once
    cv::namedWindow("MaskTrackbars", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("LowH", "MaskTrackbars", &lowH, 180);
    cv::createTrackbar("HighH", "MaskTrackbars", &highH, 180);
    cv::createTrackbar("LowS", "MaskTrackbars", &lowS, 255);
    cv::createTrackbar("HighS", "MaskTrackbars", &highS, 255);
    cv::createTrackbar("LowV", "MaskTrackbars", &lowV, 255);
    cv::createTrackbar("HighV", "MaskTrackbars", &highV, 255);

    RCLCPP_INFO(this->get_logger(), "Ball detector started");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // Read HSV values from trackbars
    cv::Scalar lower(lowH, lowS, lowV);
    cv::Scalar upper(highH, highS, highV);

    cv::inRange(hsv, lower, upper, mask);

    cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty())
    {
      auto largest = *std::max_element(contours.begin(), contours.end(),
          [](const auto &a, const auto &b){ return cv::contourArea(a) < cv::contourArea(b); });

      double area = cv::contourArea(largest);

      if (area > 300)
      {
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(largest, center, radius);

        cv::circle(frame, center, radius, {0,255,0}, 2);
        cv::circle(frame, center, 5, {0,0,255}, -1);

        int error_x = static_cast<int>(center.x - frame.cols / 2);
        RCLCPP_INFO(this->get_logger(), "Ball X error: %d", error_x);
      }
    }

    cv::imshow("Ball Detection", frame);
    cv::imshow("Mask", mask);
    cv::waitKey(1); // Small delay for OpenCV to update windows
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  // Trackbar variables
  int lowH = 0, highH = 180;
  int lowS = 0, highS = 255;
  int lowV = 0, highV = 255;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallDetector>());
  rclcpp::shutdown();
  return 0;
}
