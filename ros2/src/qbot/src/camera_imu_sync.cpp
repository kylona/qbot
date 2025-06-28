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
#include <numeric>       // For std::accumulate (used in average calculation)
#include <algorithm>     // For std::max_element

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

// Define expected sensor frequencies for buffer sizing and gap detection
const double IMU_FREQUENCY = 100.0; // Hz
const double CAMERA_FREQUENCY = 50.0; // Hz

// Threshold for triggering correlation calculation (average motion)
const double AVERAGE_MOTION_CORRELATION_THRESHOLD = 0.5; // Adjust as needed
// Duration of data window for correlation (e.g., 2 seconds)
const rclcpp::Duration CORRELATION_WINDOW_DURATION = rclcpp::Duration::from_seconds(2.0);
// Range of time offsets to search (e.g., +/- 200 ms)
const rclcpp::Duration CORRELATION_LAG_MAX = rclcpp::Duration::from_seconds(0.2); // +/- 200 ms
// Step size for searching time offsets (e.g., 1 ms)
const rclcpp::Duration CORRELATION_LAG_STEP = rclcpp::Duration::from_seconds(0.001); // 1 ms


// Max expected duration between consecutive messages to detect a "gap"
const rclcpp::Duration IMU_MAX_EXPECTED_GAP_DURATION = rclcpp::Duration::from_seconds(1.0 / IMU_FREQUENCY * 2.0); // 2x period
const rclcpp::Duration CAMERA_MAX_EXPECTED_GAP_DURATION = rclcpp::Duration::from_seconds(1.0 / CAMERA_FREQUENCY * 2.0); // 2x period

// Maximum buffer sizes to cover CORRELATION_WINDOW_DURATION + a small margin
// Max buffer size ensures we keep 'enough' data, but we'll clear on discontinuity
const size_t IMU_MAX_BUFFER_SIZE = static_cast<size_t>(CORRELATION_WINDOW_DURATION.seconds() * IMU_FREQUENCY) + 10;
const size_t CAMERA_MAX_BUFFER_SIZE = static_cast<size_t>(CORRELATION_WINDOW_DURATION.seconds() * CAMERA_FREQUENCY) + 5;

// Threshold for "active" motion points for the O(1) correlation trigger check
// These represent the minimum number of "above threshold" motion points needed
const size_t MIN_IMU_POINTS = static_cast<size_t>(CORRELATION_WINDOW_DURATION.seconds() * IMU_FREQUENCY * 0.2); // Buffer at least 20% filled
const size_t MIN_CAMERA_POINTS = static_cast<size_t>(CORRELATION_WINDOW_DURATION.seconds() * CAMERA_FREQUENCY * 0.2); // Buffer at least 20% filled

// Threshold for "active" motion points for the O(1) correlation trigger check
// These represent the minimum number of "above threshold" motion points needed
const size_t MIN_ACTIVE_IMU_POINTS = static_cast<size_t>(CORRELATION_WINDOW_DURATION.seconds() * IMU_FREQUENCY * 0.1); // At least 10% of points show motion
const size_t MIN_ACTIVE_CAMERA_POINTS = static_cast<size_t>(CORRELATION_WINDOW_DURATION.seconds() * CAMERA_FREQUENCY * 0.1); // At least 10% of points show motion

// Define motion thresholds for logging
const double CAMERA_MOTION_LOG_THRESHOLD = 1.0; // Pixels
const double IMU_MOTION_LOG_THRESHOLD = 1.0;    // Radians/sec


class CameraImuSync : public rclcpp::Node
{
    public:
    CameraImuSync() :
        Node("camera_imu_sync"),
        latest_orb_image_ptr_(nullptr),
        previous_keypoints_(),
        previous_descriptors_(),
        active_imu_motion_count_(0),
        active_camera_motion_count_(0),
        time_offset_(0.0)
    {
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/qbot/camera/image",
            10,
            std::bind(&CameraImuSync::imageCallback, this, std::placeholders::_1)
        );
        
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/qbot/imu",
            10,
            std::bind(&CameraImuSync::imuCallback, this, std::placeholders::_1)
        );
        
        // Initialize ORB detector and BFMatcher
        orb_ = cv::ORB::create();
        matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, true); // Cross-check matching
        
        // Start a separate thread for user input to avoid blocking the ROS spin
        input_thread_ = std::thread(&CameraImuSync::handleInput, this);
        
        // Create a timer to periodically perform cross-correlation
        // Runs every 500 ms (or adjust as needed based on desired update rate vs CPU usage)
        correlation_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CameraImuSync::correlationTimerCallback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "CameraImuSync Node started.");
        RCLCPP_INFO(this->get_logger(), "Subscribing to image topic: %s", image_subscription_->get_topic_name());
        RCLCPP_INFO(this->get_logger(), "Subscribing to IMU topic: %s", imu_subscription_->get_topic_name());
        RCLCPP_INFO(this->get_logger(), "Camera motion log threshold: %.2f pixels", CAMERA_MOTION_LOG_THRESHOLD);
        RCLCPP_INFO(this->get_logger(), "IMU motion log threshold: %.2f rad/s", IMU_MOTION_LOG_THRESHOLD);
        RCLCPP_INFO(this->get_logger(), "Correlation calculation triggered when average motion > %.2f", AVERAGE_MOTION_CORRELATION_THRESHOLD);
        RCLCPP_INFO(this->get_logger(), "Press ENTER in this console to save the latest ORB image as ORB.jpeg");
    }
    
    ~CameraImuSync()
    {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }
    
    private:

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::TimerBase::SharedPtr correlation_timer_; // Timer for correlation
    
    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::BFMatcher> matcher_;
    
    cv::Mat latest_orb_image_;
    cv::Mat* latest_orb_image_ptr_;
    std::mutex imu_mutex_;
    std::mutex image_mutex_;
    
    std::thread input_thread_;
    
    std::vector<cv::KeyPoint> previous_keypoints_;
    cv::Mat previous_descriptors_;
    
    std::deque<MotionData> imu_buffer_;
    std::deque<MotionData> image_buffer_;
    
    // New: Counters for "active" motion points
    size_t active_imu_motion_count_;
    size_t active_camera_motion_count_;
    
    double time_offset_;



    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Image Callback triggered");
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image_bgr = cv_ptr->image;
            
            cv::Mat image_gray;
            cv::cvtColor(image_bgr, image_gray, cv::COLOR_BGR2GRAY);
            
            std::vector<cv::KeyPoint> current_keypoints;
            cv::Mat current_descriptors;
            orb_->detectAndCompute(image_gray, cv::noArray(), current_keypoints, current_descriptors);
            
            double camera_motion = std::numeric_limits<double>::max(); // Initialize with a very large value
            
            if (!previous_keypoints_.empty() && !previous_descriptors_.empty() && !current_descriptors.empty()) {
                std::vector<cv::DMatch> matches;
                matcher_->match(previous_descriptors_, current_descriptors, matches);
                
                if (!matches.empty()) {
                    for (const auto& match : matches) {
                        cv::Point2f p_prev = previous_keypoints_[match.queryIdx].pt;
                        cv::Point2f p_curr = current_keypoints[match.trainIdx].pt;
                        
                        double dx = p_curr.x - p_prev.x;
                        double dy = p_curr.y - p_prev.y;
                        double distance = std::sqrt(dx*dx + dy*dy);
                        
                        if (distance < camera_motion) {
                            camera_motion = distance;
                        }
                    }
                    if (camera_motion > CAMERA_MOTION_LOG_THRESHOLD) {
                        RCLCPP_DEBUG(this->get_logger(), "Camera Motion (Min Pixels Moved): %.4f", camera_motion);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "No matches found between frames.");
                }
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Waiting for second frame to compute camera motion...");
            }
            
            previous_keypoints_ = current_keypoints;
            previous_descriptors_ = current_descriptors.clone();
            
            cv::Mat image_with_features;
            cv::drawKeypoints(image_bgr, current_keypoints, image_with_features, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);
            
            latest_orb_image_ = image_with_features.clone();
            latest_orb_image_ptr_ = &latest_orb_image_;
            
            std::unique_lock<std::mutex> image_lock(image_mutex_); // Lock for buffer and counter access
            
            // --- Discontinuity Detection and Buffer Clearing  ---
            if (!image_buffer_.empty()) {
                rclcpp::Time msg_timestamp(msg->header.stamp); // Convert to rclcpp::Time
                rclcpp::Duration time_diff = msg_timestamp - image_buffer_.back().timestamp;
                if (time_diff > CAMERA_MAX_EXPECTED_GAP_DURATION) {
                    RCLCPP_DEBUG(this->get_logger(), "Detected camera data discontinuity (gap %.4f s). Clearing image buffer and resetting motion count.", time_diff.seconds());
                    image_buffer_.clear();
                    active_camera_motion_count_ = 0; // Reset counter
                }
            }
            
            // --- Pop front if buffer full (before push_back) ---
            // Capture old data motion before popping for decrementing counter
            if (image_buffer_.size() >= CAMERA_MAX_BUFFER_SIZE) {
                if (image_buffer_.front().motion > AVERAGE_MOTION_CORRELATION_THRESHOLD) { // Use motion threshold to check if it was 'active'
                    active_camera_motion_count_--;
                }
                image_buffer_.pop_front();
            }
            
            // --- Push back new data ---
            image_buffer_.push_back({msg->header.stamp, camera_motion});
            if (camera_motion > AVERAGE_MOTION_CORRELATION_THRESHOLD) { // Check new data's motion
                active_camera_motion_count_++;
            }
            
            // ... existing latest_orb_image_ptr_ update ...
            // Note: latest_orb_image_ptr_ is outside the MotionData flow, ensure its mutex is used if separate.
            // It's currently locked by image_mutex_, which is good.
            latest_orb_image_ = image_with_features.clone();
            latest_orb_image_ptr_ = &latest_orb_image_; // Update the pointer to the latest image
            image_lock.unlock();
            
            
            RCLCPP_DEBUG(this->get_logger(), "Updated latest ORB image.");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Standard exception: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Unknown exception occurred during ORB detection.");
        }
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "IMU Callback triggered");
        double imu_motion = std::sqrt(
            std::pow(msg->angular_velocity.x, 2) +
            std::pow(msg->angular_velocity.y, 2) +
            std::pow(msg->angular_velocity.z, 2)
        );
        
        if (imu_motion > IMU_MOTION_LOG_THRESHOLD) {
            RCLCPP_DEBUG(this->get_logger(), "IMU Motion (Angular Velocity Norm): %.4f", imu_motion);
        }
        
        std::unique_lock<std::mutex> imu_lock(imu_mutex_); // Lock for buffer and counter access
        
        // --- Discontinuity Detection and Buffer Clearing (NEW) ---
        if (!imu_buffer_.empty()) {
            rclcpp::Time msg_timestamp(msg->header.stamp); // Convert to rclcpp::Time
            rclcpp::Duration time_diff = msg_timestamp - imu_buffer_.back().timestamp;
            if (time_diff > IMU_MAX_EXPECTED_GAP_DURATION) {
                RCLCPP_DEBUG(this->get_logger(), "Detected IMU data discontinuity (gap %.4f s). Clearing IMU buffer and resetting motion count.", time_diff.seconds());
                imu_buffer_.clear();
                active_imu_motion_count_ = 0; // Reset counter
            }
        }
        
        // --- Pop front if buffer full (before push_back) ---
        // Capture old data motion before popping for decrementing counter
        if (imu_buffer_.size() >= IMU_MAX_BUFFER_SIZE) {
            if (imu_buffer_.front().motion > AVERAGE_MOTION_CORRELATION_THRESHOLD) { // Use motion threshold to check if it was 'active'
                active_imu_motion_count_--;
            }
            imu_buffer_.pop_front();
        }
        
        // --- Push back new data ---
        imu_buffer_.push_back({msg->header.stamp, imu_motion});
        if (imu_motion > AVERAGE_MOTION_CORRELATION_THRESHOLD) { // Check new data's motion
            active_imu_motion_count_++;
        }
        imu_lock.unlock();
        
        RCLCPP_DEBUG(this->get_logger(), "Active IMU Motion Count: %ld", active_imu_motion_count_);
    }
    
    // Timer callback for periodically performing cross-correlation
    void correlationTimerCallback() {
        RCLCPP_DEBUG(this->get_logger(), "Correlation Timer Triggered");
        // --- 1. Correlation Trigger Condition  ---
        std::unique_lock<std::mutex> imu_lock(imu_mutex_);
        std::unique_lock<std::mutex> image_lock(image_mutex_); // Use image_mutex_ for image_buffer_ and active_camera_motion_count_
        
        // Check if enough data is available in the buffers 
        // And if there's sufficient "active" motion points in those buffers
        if (imu_buffer_.size() < MIN_IMU_POINTS ||
        image_buffer_.size() < MIN_CAMERA_POINTS ||
        active_imu_motion_count_ < MIN_ACTIVE_IMU_POINTS ||
        active_camera_motion_count_ < MIN_ACTIVE_CAMERA_POINTS)
        {
            RCLCPP_DEBUG(this->get_logger(),
            "Correlation skipped. IMU size: %zu/%zu, Active IMU: %ld/%zu. Cam size: %zu/%zu, Active Cam: %ld/%zu",
            imu_buffer_.size(), MIN_IMU_POINTS, active_imu_motion_count_, MIN_ACTIVE_IMU_POINTS,
            image_buffer_.size(), MIN_CAMERA_POINTS, active_camera_motion_count_, MIN_ACTIVE_CAMERA_POINTS);
            return; // Locks released on return
        }
        // Release locks now that we've checked the conditions.
        imu_lock.unlock();
        image_lock.unlock();
        
        // If we reach here, conditions are met, proceed with correlation
        performCrossCorrelation();
    }
    
    
    // Function to perform cross-correlation and estimate time offset
    void performCrossCorrelation() {
        RCLCPP_DEBUG(this->get_logger(), "Starting cross correlation");
        // --- 1. Extract recent data for correlation (copy all from guaranteed-fresh deques) ---
        
        // Acquire locks for thread-safe access to buffers during copying.
        std::unique_lock<std::mutex> imu_lock(imu_mutex_);
        std::unique_lock<std::mutex> image_lock(image_mutex_); // Use image_mutex_ for image_buffer_
        
        // Create local vectors to store the data for correlation.
        std::vector<MotionData> imu_data_for_correlation;
        std::vector<MotionData> camera_data_for_correlation;
        
        // Copy ALL data from the deques.
        // Due to discontinuity handling and fixed sizing, the deque *is* our desired window.
        imu_data_for_correlation.assign(imu_buffer_.begin(), imu_buffer_.end());
        camera_data_for_correlation.assign(image_buffer_.begin(), image_buffer_.end());
        
        // Release locks now that data is copied locally into temporary vectors.
        imu_lock.unlock();
        image_lock.unlock();
        
        // After copying, ensure we have enough data points.
        // This check is mostly a safeguard, as the trigger condition should largely prevent this.
        if (imu_data_for_correlation.size() < 2 || camera_data_for_correlation.size() < 2) {
            RCLCPP_DEBUG(this->get_logger(), "Not enough data copied into correlation vectors (less than 2 samples).");
            return;
        }
        
        // After copying, ensure we have enough data points in these specific windows to be meaningful.
        // For example, if a buffer only had 1 point newer than its window start time, it's not enough.
        // We need at least 2 points to perform meaningful correlation later.
        if (imu_data_for_correlation.size() < 2 || camera_data_for_correlation.size() < 2) {
            RCLCPP_DEBUG(this->get_logger(), "Not enough data copied into correlation windows (less than 2 samples after filtering).");
            return;
        }
        
        // Lambda for Z-score normalization - put it here so it's defined once.
        auto normalize_vector = [](std::vector<double>& vec) {
            if (vec.empty()) return;
            double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
            double mean = sum / vec.size();
            double sq_sum = std::inner_product(vec.begin(), vec.end(), vec.begin(), 0.0);
            double std_dev = std::sqrt(sq_sum / vec.size() - mean * mean);
            
            if (std_dev < 1e-9) { // Avoid division by zero if all values are same (or nearly so)
                std::fill(vec.begin(), vec.end(), 0.0);
            } else {
                for (double& val : vec) {
                    val = (val - mean) / std_dev;
                }
            }
        };
        
        // --- 2. Compute Cross-Correlation for various lags ---
        // Renumbered this section. This is the main correlation loop.
        double max_correlation = -2.0; // Initialize with a value lower than any possible correlation
        double best_lag_seconds = 0.0;
        
        double lag_max_s = CORRELATION_LAG_MAX.seconds();
        double lag_step_s = CORRELATION_LAG_STEP.seconds();
        
        for (double current_lag_s = -lag_max_s; current_lag_s <= lag_max_s; current_lag_s += lag_step_s) {
            rclcpp::Duration lag = rclcpp::Duration::from_seconds(current_lag_s);
            
            // For each lag, we re-interpolate camera data to align with IMU data.
            // These vectors will hold the time-aligned and corresponding motion values for the current lag.
            std::vector<double> current_lag_camera_motions;
            std::vector<double> current_lag_imu_motions;
            
            // Iterate through each IMU point, and for its timestamp (adjusted by lag),
            // find the corresponding camera motion through interpolation.
            for (const auto& imu_point : imu_data_for_correlation) {
                rclcpp::Time target_cam_time = imu_point.timestamp - lag; // Shift target time by lag
                
                double current_interpolated_cam_value = 0.0;
                bool interp_success = false;
                
                // Find two nearest camera points to interpolate from for the current target_cam_time
                auto it_upper = std::upper_bound(camera_data_for_correlation.begin(), camera_data_for_correlation.end(), target_cam_time,
                [](const rclcpp::Time& ts, const MotionData& md){ return ts < md.timestamp; });
                
                if (it_upper != camera_data_for_correlation.end()) {
                    if (it_upper != camera_data_for_correlation.begin()) {
                        auto it_lower = std::prev(it_upper);
                        
                        // If target time matches lower bound timestamp, use it directly (exact match)
                        if (it_lower->timestamp == target_cam_time) { // Note: Changed to target_cam_time
                            current_interpolated_cam_value = it_lower->motion;
                            interp_success = true;
                        } else {
                            double t_lower = it_lower->timestamp.seconds();
                            double t_upper = it_upper->timestamp.seconds();
                            double t_target = target_cam_time.seconds();
                            
                            if (t_upper - t_lower > 1e-9) { // Avoid division by zero
                                current_interpolated_cam_value = it_lower->motion +
                                (it_upper->motion - it_lower->motion) *
                                ((t_target - t_lower) / (t_upper - t_lower));
                                interp_success = true;
                            }
                        }
                    } else { // target_cam_time is before or at the first camera_data_for_correlation point
                        current_interpolated_cam_value = it_upper->motion; // Use the first camera value
                        interp_success = true;
                    }
                } else if (!camera_data_for_correlation.empty()) { // target_cam_time is after the last camera_data_for_correlation point
                    current_interpolated_cam_value = camera_data_for_correlation.back().motion; // Use the last camera value
                    interp_success = true;
                }
                
                // Only add data points where interpolation was successful
                if (interp_success) {
                    current_lag_camera_motions.push_back(current_interpolated_cam_value);
                    current_lag_imu_motions.push_back(imu_point.motion); // Keep original IMU motion for this time point
                }
            }
            
            // Check if enough data points were successfully interpolated for this lag
            if (current_lag_camera_motions.size() < 2 || current_lag_imu_motions.size() < 2 ||
            current_lag_camera_motions.size() != current_lag_imu_motions.size()) {
                RCLCPP_DEBUG(this->get_logger(), "Not enough aligned data points for correlation at lag %.4f. Skipping.", current_lag_s);
                continue; // Skip this lag if data is insufficient
            }
            
            // Normalize the motion vectors for the current lag before computing correlation
            std::vector<double> imu_vec_norm = current_lag_imu_motions;
            std::vector<double> cam_vec_norm = current_lag_camera_motions;
            normalize_vector(imu_vec_norm);
            normalize_vector(cam_vec_norm);
            
            // Calculate Pearson correlation coefficient
            double correlation = 0.0;
            // The check for size() > 1 is already handled by the prior if condition
            double dot_product = std::inner_product(imu_vec_norm.begin(), imu_vec_norm.end(), cam_vec_norm.begin(), 0.0);
            correlation = dot_product / imu_vec_norm.size();
            
            // Update best correlation and lag
            if (correlation > max_correlation) {
                max_correlation = correlation;
                best_lag_seconds = current_lag_s; // Store the lag that produced the best correlation
            }
        }
        
        // --- 3. Update and Log Time Offset ---
        // Renumbered this section.
        if (max_correlation > 0.5) { // Check if a meaningful correlation was found (e.g., not initial -2.0)
            time_offset_ = best_lag_seconds; // IMU_time - Camera_time = offset (i.e., add offset to camera time to align with IMU)
            
            RCLCPP_INFO(this->get_logger(),
            "Synchronization updated! New time_offset (IMU - Camera): %.4f seconds (Correlation: %.4f)",
            time_offset_, max_correlation);
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not find a strong correlation for synchronization over the tested lags.");
        }
    }
    
    
    void handleInput()
    {
        std::string line;
        while (rclcpp::ok()) {
            std::cout << "Press ENTER to save ORB.jpeg..." << std::endl;
            std::getline(std::cin, line);
            
            if (line.empty() && rclcpp::ok()) {
                saveORBImage();
            } else if (!rclcpp::ok()) {
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
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraImuSync>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}