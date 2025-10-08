#ifndef VIDEO_MANAGER_HPP
#define VIDEO_MANAGER_HPP

#include <common_msgs/msg/detections.hpp>
#include <memory>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shm_msgs/msg/image1m.hpp>
#include <shm_msgs/opencv_conversions.hpp>
#include <string>
class VideoManager : public rclcpp::Node {
public:
  explicit VideoManager(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~VideoManager();

private:
  // Subscription callbacks
  void currentFrameCallback(const shm_msgs::msg::Image1m::ConstSharedPtr &msg);
  void delayedFrameCallback(const shm_msgs::msg::Image1m::SharedPtr msg);
  void detectionCallback(const common_msgs::msg::Detections::SharedPtr msg);
  // Timer callback for display updates
  void displayFrames();

  // Helper functions
  void displayFrameWithMetadata(const cv::Mat &frame,
                                const std::string &window_name,
                                const std_msgs::msg::Header &header,
                                const std::string &frame_type);
  std::string formatTimestamp(const builtin_interfaces::msg::Time &timestamp);
  void overlayMetadata(cv::Mat &frame, const std::string &metadata_text);

  // ROS2 components
  std::shared_ptr<message_filters::Subscriber<shm_msgs::msg::Image1m>>
      current_frame_subscription_;
  rclcpp::Subscription<shm_msgs::msg::Image1m>::SharedPtr
      delayed_frame_subscription_;
  rclcpp::Subscription<common_msgs::msg::Detections>::SharedPtr
      detection_subscription_;
  std::shared_ptr<message_filters::Cache<shm_msgs::msg::Image1m>> frame_cache_;

  std::mutex detection_frame_mutex;

  // Frame storage
  shm_msgs::CvImageConstPtr current_frame_;
  shm_msgs::CvImageConstPtr detection_frame_;
  shm_msgs::CvImageConstPtr delayed_frame_;

  // Detection storage
  std::vector<common_msgs::msg::Detection> current_detections;
  int current_target_id = -1;
  // Display parameters
  static constexpr int DISPLAY_WIDTH = 640;
  static constexpr int DISPLAY_HEIGHT = 480;
};

#endif // VIDEO_MANAGER_HPP