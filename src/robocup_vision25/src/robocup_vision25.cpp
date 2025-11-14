#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp> 
#include <memory>
#include <vector>

#include "humanoid_interfaces/msg/human_pj_vision.hpp"
#include "geometry_msgs/msg/point.hpp"

int dir = 0;
int move = 0;

class ImageViewer : public rclcpp::Node
{
public:
  ImageViewer()
      : Node("robocup_vision25_node")
  {
    const std::string image_topic = "/camera/image_raw";

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, 10, std::bind(&ImageViewer::image_callback, this, std::placeholders::_1));

    coords_pub_ = this->create_publisher<humanoid_interfaces::msg::HumanPjVision>(
        "/robit_mj/red_pixel_flag", 10);

    RCLCPP_INFO(this->get_logger(), "Image viewer node has been started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing red pixel flag to: /robit_mj/red_pixel_flag");

    cv::namedWindow("oCam Feed");
    cv::namedWindow("White Mask");
    cv::namedWindow("Red Mask");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {

      cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

      // Convert ROS Image to OpenCV Image
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat frame = cv_ptr->image;
      cv::Mat display_frame = frame.clone();
      // Convert BGR to HSV
      cv::Mat hsv_frame;
      cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

      // --- White Color Detection ---
      cv::Scalar lower_white = cv::Scalar(0, 0, 35);
      cv::Scalar upper_white = cv::Scalar(255, 40, 230);
      cv::Mat white_mask;
      cv::inRange(hsv_frame, lower_white, upper_white, white_mask);
      cv::dilate(white_mask, white_mask, k);
      cv::erode(white_mask, white_mask, k);
      cv::erode(white_mask, white_mask, k);
      cv::dilate(white_mask, white_mask, k);
      cv::dilate(white_mask, white_mask, k);
      cv::dilate(white_mask, white_mask, k);
      cv::erode(white_mask, white_mask, k);
      cv::erode(white_mask, white_mask, k);

      // --- Red Color Detection ---
      cv::Scalar lower_red1 = cv::Scalar(0, 70, 10);
      cv::Scalar upper_red1 = cv::Scalar(10, 255, 255);
      cv::Mat red_mask1;
      cv::inRange(hsv_frame, lower_red1, upper_red1, red_mask1);
      cv::Scalar lower_red2 = cv::Scalar(170, 70, 10);
      cv::Scalar upper_red2 = cv::Scalar(180, 255, 205);
      cv::Mat red_mask2;
      cv::inRange(hsv_frame, lower_red2, upper_red2, red_mask2);
      cv::Mat red_mask;
      cv::bitwise_or(red_mask1, red_mask2, red_mask);
      cv::dilate(red_mask, red_mask, k);
      cv::erode(red_mask, red_mask, k);
      cv::erode(red_mask, red_mask, k);
      cv::dilate(red_mask, red_mask, k);
      cv::dilate(red_mask, red_mask, k);
      cv::dilate(red_mask, red_mask, k);
      cv::erode(red_mask, red_mask, k);
      cv::erode(red_mask, red_mask, k);

      // --- 선그리기---

      cv::line(display_frame, cv::Point(frame.cols / 2, 0), cv::Point(frame.cols / 2, frame.rows), cv::Scalar(0, 255, 255), 1);
      cv::line(display_frame, cv::Point(frame.cols / 4 * 3, 0), cv::Point(frame.cols / 4 * 3, frame.rows), cv::Scalar(0, 255, 255), 1);
      cv::line(display_frame, cv::Point(frame.cols / 4, 0), cv::Point(frame.cols / 4, frame.rows), cv::Scalar(0, 255, 255), 1);

      cv::line(display_frame, cv::Point(0, frame.rows / 2), cv::Point(frame.cols, frame.rows / 2), cv::Scalar(0, 255, 255), 1);
      cv::line(display_frame, cv::Point(0, frame.rows / 4 * 3), cv::Point(frame.cols, frame.rows / 4 * 3), cv::Scalar(0, 255, 255), 1);
      cv::line(display_frame, cv::Point(0, frame.rows / 4), cv::Point(frame.cols, frame.rows / 4), cv::Scalar(0, 255, 255), 1);

      // On White Mask (Grayscale image)
      cv::line(white_mask, cv::Point(frame.cols * 0.2, 0), cv::Point(frame.cols * 0.2, frame.rows), cv::Scalar(255), 1);
      cv::line(white_mask, cv::Point(frame.cols * 0.4, 0), cv::Point(frame.cols * 0.4, frame.rows), cv::Scalar(255), 1);
      cv::line(white_mask, cv::Point(frame.cols * 0.6, 0), cv::Point(frame.cols * 0.6, frame.rows), cv::Scalar(255), 1);
      cv::line(white_mask, cv::Point(frame.cols * 0.8, 0), cv::Point(frame.cols * 0.8, frame.rows), cv::Scalar(255), 1);

      cv::line(white_mask, cv::Point(0, frame.rows * 0.2), cv::Point(frame.cols, frame.rows * 0.2), cv::Scalar(255), 1);
      cv::line(white_mask, cv::Point(0, frame.rows * 0.4), cv::Point(frame.cols, frame.rows * 0.4), cv::Scalar(255), 1);
      cv::line(white_mask, cv::Point(0, frame.rows * 0.6), cv::Point(frame.cols, frame.rows * 0.6), cv::Scalar(255), 1);
      cv::line(white_mask, cv::Point(0, frame.rows * 0.8), cv::Point(frame.cols, frame.rows * 0.8), cv::Scalar(255), 1);

      // On Red Mask (Grayscale image)
      cv::line(red_mask, cv::Point(frame.cols / 2, 0), cv::Point(frame.cols / 2, frame.rows), cv::Scalar(255), 1);
      cv::line(red_mask, cv::Point(frame.cols / 4 * 3, 0), cv::Point(frame.cols / 4 * 3, frame.rows), cv::Scalar(255), 1);
      cv::line(red_mask, cv::Point(frame.cols / 4, 0), cv::Point(frame.cols / 4, frame.rows), cv::Scalar(255), 1);

      cv::line(red_mask, cv::Point(0, frame.rows / 2), cv::Point(frame.cols, frame.rows / 2), cv::Scalar(255), 1);
      cv::line(red_mask, cv::Point(0, frame.rows / 4 * 3), cv::Point(frame.cols, frame.rows / 4 * 3), cv::Scalar(255), 1);
      cv::line(red_mask, cv::Point(0, frame.rows / 4), cv::Point(frame.cols, frame.rows / 4), cv::Scalar(255), 1);

      // Display the images
      cv::imshow("oCam Feed", display_frame);
      cv::imshow("White Mask", white_mask);
      cv::imshow("Red Mask", red_mask);

      cv::waitKey(1);

      std::vector<cv::Point> red_locations;
      cv::findNonZero(red_mask, red_locations);
      std::vector<cv::Point> white_locations;
      cv::findNonZero(white_mask, white_locations);

      // --- 픽셀 수 계산 ---
      int lower_region_boundary_row = frame.rows * 0.5; // 하단 절반 영역
      int lower_region_boundary_col = frame.cols * 0.65;
      const int min_red_pixels_in_region = 10000;
      int red_pixels_in_lower_region = 0;
      for (const auto &pt : red_locations)
      {
        if (pt.y > lower_region_boundary_row)
        {
          if (pt.x < lower_region_boundary_col)
            red_pixels_in_lower_region++;
        }
      }

      int lower_region_boundary2_row = frame.rows * 0.6;
      int lower_region_boundary2_col = frame.cols * 0.4;
      const int min_white_pixels_in_region = 4000;
      int white_pixels_in_lower_region = 0;
      for (const auto &pt : white_locations)
      {
        if (pt.y > lower_region_boundary2_row)
        {
          if (pt.x > frame.cols * 0.15 && pt.x < lower_region_boundary2_col)
          {
            white_pixels_in_lower_region++;
          }
        }
      }
      // FLAG 계산
      bool is_red_detected = red_pixels_in_lower_region > min_red_pixels_in_region;
      bool is_white_detected = white_pixels_in_lower_region > min_white_pixels_in_region;
      int flag = 0;
      bool has_seen_white_while_red_is_on_ = false;
       if (!is_red_detected)
      {
        flag = 0;
        has_seen_white_while_red_is_on_ = false;
      }
      else
      {
        // Red is detected
        if (is_white_detected)
        {
          has_seen_white_while_red_is_on_ = true;
        }

        if (has_seen_white_while_red_is_on_)
        {
          flag = 2;
        }
        else
        {
          flag = 1;
        }
      }

      // --- 퍼블리시 ---
      auto msg_out = std::make_unique<humanoid_interfaces::msg::HumanPjVision>();
      msg_out->header = msg->header;
      msg_out->flag = flag;

      coords_pub_->publish(std::move(msg_out));

      RCLCPP_INFO(this->get_logger(), "Published flag: %d red cnt : %d", flag, red_pixels_in_lower_region);

      RCLCPP_INFO(this->get_logger(), "white cnt : %d", white_pixels_in_lower_region);
    }

    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<humanoid_interfaces::msg::HumanPjVision>::SharedPtr coords_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageViewer>());
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}