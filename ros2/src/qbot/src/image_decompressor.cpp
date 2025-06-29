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

// You might need OpenCV headers if you do more manipulation,
// but cv_bridge includes them implicitly for the conversion functions.
// #include <opencv2/opencv.hpp> // Or specific headers like <opencv2/imgproc.hpp>

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

    // Publish the uncompressed image to a standard topic
    uncompressed_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/qbot/camera/image", // Standard topic name for uncompressed images
      10);

    RCLCPP_INFO(this->get_logger(), "Image Decompressor Node started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", compressed_subscription_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Publishing uncompressed images to: %s", uncompressed_publisher_->get_topic_name());
  }

private:
  void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    try {
      // Use toCvCopy to get a modifiable copy. BGR8 is common for OpenCV processing.
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // Create a new Image message
      sensor_msgs::msg::Image::SharedPtr ros_image = cv_ptr->toImageMsg();

      uncompressed_publisher_->publish(*ros_image);
      // RCLCPP_INFO(this->get_logger(), "Published uncompressed image"); // Optional: Can be verbose
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) { // Catch other potential standard exceptions
      RCLCPP_ERROR(this->get_logger(), "Standard exception: %s", e.what());
    } catch (...) { // Catch any other unknown exceptions
      RCLCPP_ERROR(this->get_logger(), "Unknown exception occurred during image decompression.");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr uncompressed_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageDecompressor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}