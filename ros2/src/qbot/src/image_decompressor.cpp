// Core ROS2
#include <rclcpp/rclcpp.hpp>

// Message types
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp> // For image_encodings::BGR8

// cv_bridge
#include <cv_bridge/cv_bridge.hpp>

// Standard libraries
#include <memory>        // For std::make_shared
#include <functional>    // For std::bind, std::placeholders

// OpenCV headers for ORB and drawing
#include <opencv2/features2d.hpp> // For ORB
#include <opencv2/imgproc.hpp>    // For cv::cvtColor (if converting to grayscale)
#include <opencv2/highgui.hpp>    // For cv::imshow (if you want to display images, typically not in a ROS node)

class ImageDecompressor : public rclcpp::Node
{
public:
  ImageDecompressor()
  : Node("image_decompressor")
  {
    compressed_subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/qbot/camera/compressed_image",
      10,
      std::bind(&ImageDecompressor::compressedImageCallback, this, std::placeholders::_1));

    // We might want a separate publisher for the image with keypoints drawn
    uncompressed_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/qbot/camera/image_with_features", // New topic name for processed image
      10);

    // Initialize ORB detector
    orb_ = cv::ORB::create();

    RCLCPP_INFO(this->get_logger(), "Image Decompressor Node started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", compressed_subscription_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Publishing processed images to: %s", uncompressed_publisher_->get_topic_name());
  }

private:
  void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    try {
      // Use toCvCopy to get a modifiable copy. BGR8 is common for OpenCV processing.
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat image_bgr = cv_ptr->image; // Get the OpenCV Mat from cv_bridge

      // Convert to grayscale for ORB
      cv::Mat image_gray;
      cv::cvtColor(image_bgr, image_gray, cv::COLOR_BGR2GRAY);

      // Detect keypoints and compute descriptors with ORB
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat descriptors;
      orb_->detectAndCompute(image_gray, cv::noArray(), keypoints, descriptors);

      // Draw keypoints on the BGR image (for visualization)
      cv::drawKeypoints(image_bgr, keypoints, image_bgr, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);

      // Update the cv_ptr with the modified image
      cv_ptr->image = image_bgr;
      cv_ptr->encoding = sensor_msgs::image_encodings::BGR8; // Ensure encoding is still BGR8

      // Create a new Image message from the modified cv_ptr
      sensor_msgs::msg::Image::SharedPtr ros_image = cv_ptr->toImageMsg();

      // The header information should already be part of cv_ptr from toCvCopy
      // If not, uncomment below:
      // ros_image->header = msg->header;

      uncompressed_publisher_->publish(*ros_image); // Publish the message data
      // RCLCPP_INFO(this->get_logger(), "Published uncompressed image with ORB features."); // Optional: Can be verbose
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) { // Catch other potential standard exceptions
      RCLCPP_ERROR(this->get_logger(), "Standard exception: %s", e.what());
    } catch (...) { // Catch any other unknown exceptions
      RCLCPP_ERROR(this->get_logger(), "Unknown exception occurred during image decompression and feature detection.");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr uncompressed_publisher_;
  cv::Ptr<cv::ORB> orb_; // ORB detector object
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageDecompressor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}