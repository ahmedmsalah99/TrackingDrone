#ifndef VIDEO_MANAGER_HPP
#define VIDEO_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <shm_msgs/msg/image1m.hpp>
#include <shm_msgs/opencv_conversions.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

class VideoManager : public rclcpp::Node
{
public:
    explicit VideoManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~VideoManager();

private:
    // Subscription callbacks
    void currentFrameCallback(const shm_msgs::msg::Image1m::SharedPtr msg);
    void delayedFrameCallback(const shm_msgs::msg::Image1m::SharedPtr msg);
    
    // Timer callback for display updates
    void displayTimerCallback();
    
    // Helper functions
    void displayFrameWithMetadata(const cv::Mat& frame, const std::string& window_name, 
                                  const std_msgs::msg::Header& header, const std::string& frame_type);
    std::string formatTimestamp(const builtin_interfaces::msg::Time& timestamp);
    void overlayMetadata(cv::Mat& frame, const std::string& metadata_text);
    
    // ROS2 components
    rclcpp::Subscription<shm_msgs::msg::Image1m>::SharedPtr current_frame_subscription_;
    rclcpp::Subscription<shm_msgs::msg::Image1m>::SharedPtr delayed_frame_subscription_;
    rclcpp::TimerBase::SharedPtr display_timer_;
    
    // Frame storage
    shm_msgs::CvImageConstPtr current_frame_;
    shm_msgs::CvImageConstPtr delayed_frame_;
    
    // Synchronization
    std::mutex frame_mutex_;
    
    // Display parameters
    static constexpr int DISPLAY_WIDTH = 640;
    static constexpr int DISPLAY_HEIGHT = 480;
    static constexpr double TIMER_PERIOD_MS = 20.0; // 20ms as requested
};

#endif // VIDEO_MANAGER_HPP