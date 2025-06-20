// Core ROS2
#include <rclcpp/rclcpp.hpp>

// Message types
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp> // Include for IMU messages
#include <sensor_msgs/image_encodings.hpp> // For image_encodings::BGR8

// cv_bridge
#include <cv_bridge/cv_bridge.hpp>

// Standard libraries
#include <memory>        // For std::make_shared
#include <functional>    // For std::bind, std::placeholders
#include <iostream>      // For std::cin, std::cout
#include <string>        // For std::getline
#include <thread>        // For std::thread
#include <mutex>         // For std::mutex, std::unique_lock
#include <vector>        // For std::vector
#include <limits>        // For std::numeric_limits
#include <cmath>         // For std::sqrt, std::pow
#include <deque>         // For std::deque
#include <iomanip>       // For std::fixed, std::setprecision

// OpenCV headers for ORB and drawing
#include <opencv2/features2d.hpp> // For ORB, BFMatcher
#include <opencv2/imgproc.hpp>    // For cv::cvtColor
#include <opencv2/highgui.hpp>    // For cv::imwrite
#include <opencv2/calib3d.hpp>    // For findHomography (though not directly used for motion here, useful for pose estimation)

// Structure to hold timestamped motion data
struct MotionData {
  rclcpp::Time timestamp;
  double motion;
};

class CameraImuSync : public rclcpp::Node
{
public:
  CameraImuSync()
  : Node("camera_imu_sync"),
    latest_orb_image_ptr_(nullptr), // Initialize shared_ptr to nullptr
    previous_keypoints_(),          // Initialize empty
    previous_descriptors_()         // Initialize empty
  {
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/qbot/camera/image", // Subscribe to the uncompressed image topic
      10,
      std::bind(&CameraImuSync::imageCallback, this, std::placeholders::_1));

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/qbot/imu",
      10,
      std::bind(&CameraImuSync::imuCallback, this, std::placeholders::_1));

    // Initialize ORB detector and BFMatcher
    orb_ = cv::ORB::create();
    matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, true); // Cross-check matching

    // Start a separate thread for user input to avoid blocking the ROS spin
    input_thread_ = std::thread(&CameraImuSync::handleInput, this);

    RCLCPP_INFO(this->get_logger(), "ORB Detector Node started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to image topic: %s", image_subscription_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Subscribing to IMU topic: %s", imu_subscription_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Press ENTER in this console to save the latest ORB image as ORB.jpeg");
  }

  ~CameraImuSync()
  {
    // Join the input thread to ensure it finishes before the node is destroyed
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // Convert ROS image to OpenCV Mat
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat image_bgr = cv_ptr->image;

      // Convert to grayscale for ORB
      cv::Mat image_gray;
      cv::cvtColor(image_bgr, image_gray, cv::COLOR_BGR2GRAY);

      // Detect keypoints and compute descriptors with ORB
      std::vector<cv::KeyPoint> current_keypoints;
      cv::Mat current_descriptors;
      orb_->detectAndCompute(image_gray, cv::noArray(), current_keypoints, current_descriptors);

      double camera_motion = std::numeric_limits<double>::max(); // Initialize with a very large value

      // Only compute motion if we have previous keypoints and descriptors
      if (!previous_keypoints_.empty() && !previous_descriptors_.empty() && !current_descriptors.empty()) {
        std::vector<cv::DMatch> matches;
        matcher_->match(previous_descriptors_, current_descriptors, matches);

        if (!matches.empty()) {
          // Find the minimum movement
          for (const auto& match : matches) {
            // Get the keypoints from the previous and current frames
            cv::Point2f p_prev = previous_keypoints_[match.queryIdx].pt;
            cv::Point2f p_curr = current_keypoints[match.trainIdx].pt;

            // Calculate the Euclidean distance moved
            double dx = p_curr.x - p_prev.x;
            double dy = p_curr.y - p_prev.y;
            double distance = std::sqrt(dx*dx + dy*dy);

            if (distance < camera_motion) {
              camera_motion = distance;
            }
          }
          RCLCPP_INFO(this->get_logger(), "Camera Motion (Min Pixels Moved): %.4f", camera_motion);
        } else {
          RCLCPP_WARN(this->get_logger(), "No matches found between frames.");
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "Waiting for second frame to compute camera motion...");
      }

      // Store current keypoints and descriptors for the next frame
      previous_keypoints_ = current_keypoints;
      previous_descriptors_ = current_descriptors.clone(); // Make a deep copy

      // Draw keypoints on the BGR image
      cv::Mat image_with_features;
      cv::drawKeypoints(image_bgr, current_keypoints, image_with_features, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);

      // Store the latest image with features for saving
      std::unique_lock<std::mutex> lock(image_mutex_);
      latest_orb_image_ = image_with_features.clone(); // Make a deep copy
      latest_orb_image_ptr_ = &latest_orb_image_; // Update the pointer

      // Buffer image data
      if (image_buffer_.size() >= 250) {
        image_buffer_.pop_front(); // Remove the oldest element
      }
      image_buffer_.push_back({msg->header.stamp, camera_motion});

      RCLCPP_DEBUG(this->get_logger(), "Updated latest ORB image."); // Use DEBUG for less verbose output
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Standard exception: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception occurred during ORB detection.");
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Calculate the norm of the angular velocity
    double imu_motion = std::sqrt(
      std::pow(msg->angular_velocity.x, 2) +
      std::pow(msg->angular_velocity.y, 2) +
      std::pow(msg->angular_velocity.z, 2));

    RCLCPP_INFO(this->get_logger(), "IMU Motion (Angular Velocity Norm): %.4f", imu_motion);

    // Buffer IMU data
    if (imu_buffer_.size() >= 500) {
      imu_buffer_.pop_front(); // Remove the oldest element
    }
    imu_buffer_.push_back({msg->header.stamp, imu_motion});
  }

  void handleInput()
  {
    std::string line;
    while (rclcpp::ok()) { // Keep running as long as ROS is OK
      std::cout << "Press ENTER to save ORB.jpeg..." << std::endl;
      std::getline(std::cin, line);

      if (line.empty() && rclcpp::ok()) { // Check rclcpp::ok() again in case shutdown happened during getline
        saveORBImage();
      } else if (!rclcpp::ok()) {
        // Node is shutting down, exit input thread
        break;
      }
    }
  }

  void saveORBImage()
  {
    std::unique_lock<std::mutex> lock(image_mutex_);
    if (latest_orb_image_ptr_ && !latest_orb_image_ptr_->empty()) {
      std::string filename = "ORB.jpeg";
      if (cv::imwrite(filename, *latest_orb_image_ptr_)) {
        RCLCPP_INFO(this->get_logger(), "Saved latest ORB image to %s", filename.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save ORB image to %s", filename.c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "No ORB image available to save yet or image is empty.");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_; // IMU subscription
  cv::Ptr<cv::ORB> orb_;
  cv::Ptr<cv::BFMatcher> matcher_; // For matching descriptors between frames

  // To store the latest processed image for saving
  cv::Mat latest_orb_image_;
  cv::Mat* latest_orb_image_ptr_; // Pointer to the latest_orb_image_
  std::mutex image_mutex_; // Mutex to protect access to latest_orb_image_

  std::thread input_thread_; // Thread for handling user input

  // Members to store previous frame's data for motion estimation
  std::vector<cv::KeyPoint> previous_keypoints_;
  cv::Mat previous_descriptors_;

  // Buffers for IMU and Image data
  std::deque<MotionData> imu_buffer_;
  std::deque<MotionData> image_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraImuSync>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}