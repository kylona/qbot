#include <numeric>       // For std::accumulate (used in average calculation)
#include <vector>        // For std::vector
#include "cross_correlate_motion.h"

void normalize_vector(std::vector<MotionData>& motion_data_vec) {
    std::vector<double> vec;
    vec.reserve(motion_data_vec.size());
    for (auto& data : motion_data_vec) {
        vec.push_back(data.motion);
    }
    
    if (vec.empty()) return;
    double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
    double mean = sum / vec.size();
    double sq_sum = std::inner_product(vec.begin(), vec.end(), vec.begin(), 0.0);
    double std_dev = std::sqrt(sq_sum / vec.size() - mean * mean);
    
    if (std_dev < 1e-9) { // Avoid division by zero if all values are same (or nearly so)
        for (auto& data : motion_data_vec) {
            data.motion = 0.0;
        }
    } else {
        for (auto& data : motion_data_vec) {
            data.motion = (data.motion - mean) / std_dev;
        }
    }
};

// Function to perform cross-correlation and estimate time offset
void performCrossCorrelation(std::vector<MotionData> &first_data, std::vector<MotionData> &second_data, double &time_offset, double &scale_correction) {
    RCLCPP_DEBUG(this->get_logger(), "Starting cross correlation");

    // Ensure we have enough data points.
    if (first_data.size() < 2 || second_data.size() < 2) {
        RCLCPP_DEBUG(this->get_logger(), "Not enough data copied into correlation vectors (less than 2 samples).");
        return;
    }

    normalize_vector(first_data);
    normalize_vector(second_data);

    // --- 2. Compute Cross-Correlation for various lags ---
    // Renumbered this section. This is the main correlation loop.
    double max_correlation = -2.0; // Initialize with a value lower than any possible correlation
    double best_lag_seconds = 0.0;
    
    const double lag_max_s = CORRELATION_LAG_MAX.seconds();
    const double lag_step_s = CORRELATION_LAG_STEP.seconds();
    
    for (double current_lag_s = -lag_max_s; current_lag_s <= lag_max_s; current_lag_s += lag_step_s) {
        rclcpp::Duration lag = rclcpp::Duration::from_seconds(current_lag_s);
        
        // For each lag, we re-interpolate camera data to align with IMU data.
        // These vectors will hold the time-aligned and corresponding motion values for the current lag.
        std::vector<double> current_lag_camera_motions;
        std::vector<double> current_lag_imu_motions;
        
        // Iterate through each IMU point, and for its timestamp (adjusted by lag),
        // find the corresponding camera motion through interpolation.
        for (const auto& first_point : first_data) {
            rclcpp::Time target_cam_time = first_point.timestamp - lag; // Shift target time by lag
            
            double current_interpolated_cam_value = 0.0;
            bool interp_success = false;
            
            // Find two nearest camera points to interpolate from for the current target_cam_time
            auto it_upper = std::upper_bound(second_data.begin(), second_data.end(), target_cam_time,
            [](const rclcpp::Time& ts, const MotionData& md){ return ts < md.timestamp; });
            
            if (it_upper != second_data.end()) {
                if (it_upper != second_data.begin()) {
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
            } else if (!second_data.empty()) { // target_cam_time is after the last camera_data_for_correlation point
                current_interpolated_cam_value = second_data.back().motion; // Use the last camera value
                interp_success = true;
            }
            
            // Only add data points where interpolation was successful
            if (interp_success) {
                current_lag_camera_motions.push_back(current_interpolated_cam_value);
                current_lag_imu_motions.push_back(first_point.motion); // Keep original IMU motion for this time point
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
