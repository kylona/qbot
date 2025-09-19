#include <stdint.h>
#include <iostream>
// #include <rclcpp/rclcpp.hpp>

#ifndef RCLCPP_HPP

#define RCLCPP_DEBUG(LOGGER, A) std::cout << (A) << std::endl

namespace rclcpp {
    class Time {
        private:
            double _seconds;
        public:
            double seconds() {
                return this->_seconds;
            }
    };

    class Duration {
        private:
            double _seconds;
        public:
            Duration(double seconds) {
                this->_seconds = seconds;
            }
            static Duration from_seconds(double seconds) {
                return Duration(seconds);
            }
            double seconds() {
                return this->_seconds;
            }
    };
}
#endif

// Threshold for triggering correlation calculation (average motion)
const double AVERAGE_MOTION_CORRELATION_THRESHOLD = 0.5; // Adjust as needed
// Duration of data window for correlation (e.g., 2 seconds)
const rclcpp::Duration CORRELATION_WINDOW_DURATION = rclcpp::Duration::from_seconds(2.0);
// Range of time offsets to search (e.g., +/- 200 ms)
const rclcpp::Duration CORRELATION_LAG_MAX = rclcpp::Duration::from_seconds(0.2); // +/- 200 ms
// Step size for searching time offsets (e.g., 1 ms)
const rclcpp::Duration CORRELATION_LAG_STEP = rclcpp::Duration::from_seconds(0.001); // 1 ms


// Structure to hold timestamped motion data
struct MotionData {
    rclcpp::Time timestamp;
    double motion;
};
