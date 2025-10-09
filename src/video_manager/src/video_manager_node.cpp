#include "video_manager/draw.hpp"
#include "video_manager/video_manager.hpp"
#include <iomanip>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <sstream>

VideoManager::VideoManager(const rclcpp::NodeOptions &options)
    : Node("video_manager", options) {
  RCLCPP_INFO(this->get_logger(), "Initializing VideoManager node");
    declare_parameter<int>("DISPLAY_WIDTH", 640);
    declare_parameter<int>("DISPLAY_HEIGHT", 480);
    declare_parameter<int>("num_frames_cached", 50);
    declare_parameter<double>("detection_time_out", 0.7);

    // Get parameters
    DISPLAY_WIDTH = get_parameter("DISPLAY_WIDTH").as_int();
    DISPLAY_HEIGHT = get_parameter("DISPLAY_HEIGHT").as_int();
    int num_frames_cached = get_parameter("num_frames_cached").as_int();
    detection_time_out = get_parameter("detection_time_out").as_double();

  // Create subscriptions to the stream manager topics
  rclcpp::QoS qos(rclcpp::KeepLast(1));

  current_frame_subscription_ =
      std::make_shared<message_filters::Subscriber<shm_msgs::msg::Image1m>>(
          this, "/stream_manager/current_frame");
  frame_cache_ =
      std::make_shared<message_filters::Cache<shm_msgs::msg::Image1m>>(
          *current_frame_subscription_, num_frames_cached);
  frame_cache_->registerCallback(std::bind(&VideoManager::currentFrameCallback,
                                           this, std::placeholders::_1));

  detection_subscription_ =
      this->create_subscription<common_msgs::msg::Detections>(
          "detections", qos,
          std::bind(&VideoManager::detectionCallback, this,
                    std::placeholders::_1));


  // Initialize OpenCV windows
  cv::namedWindow("Current Frame", cv::WINDOW_AUTOSIZE);

  RCLCPP_INFO(this->get_logger(), "VideoManager node initialized successfully");
  RCLCPP_INFO(this->get_logger(), "Subscribed to:");
  RCLCPP_INFO(this->get_logger(), "  - /stream_manager/current_frame");
}

VideoManager::~VideoManager() {
  RCLCPP_INFO(this->get_logger(), "Shutting down VideoManager node");
  cv::destroyAllWindows();
}

void VideoManager::currentFrameCallback(
    const shm_msgs::msg::Image1m::ConstSharedPtr &msg) {
  try {
    current_frame_ = shm_msgs::toCvShare(msg);
    displayFrames();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing current frame: %s",
                 e.what());
  }
}

void VideoManager::detectionCallback(
    const common_msgs::msg::Detections::SharedPtr detmsg) {
  rclcpp::Time oldesttime = frame_cache_->getOldestTime();
  rclcpp::Time latesttime = frame_cache_->getLatestTime();
  rclcpp::Time detection_stamp(detmsg->stamp);

  if (detection_stamp < oldesttime) {

    RCLCPP_WARN(this->get_logger(),
                "Detection timestamp is older than cached frames. "
                "Detection time: %.3f, Oldest cached frame time: %.3f",
                detection_stamp.seconds(), oldesttime.seconds());
    return;
  }

  shm_msgs::msg::Image1m::ConstSharedPtr framemsg =
      frame_cache_->getElemBeforeTime(detection_stamp +
                                      rclcpp::Duration::from_nanoseconds(10));
  if (!framemsg) {
    RCLCPP_WARN(this->get_logger(),
                "No current frame available for timestamp: %d",
                detmsg->stamp.nanosec);
    return;
  }
  rclcpp::Time msg_time(framemsg->header.stamp);
  rclcpp::Time det_time(detmsg->stamp);

  if ((msg_time - det_time) > rclcpp::Duration::from_seconds(0.05)) {
    RCLCPP_WARN(this->get_logger(), "Detection timed out");
    return;
  }
  std::lock_guard<std::mutex> lock(detection_frame_mutex);
  detection_frame_ = shm_msgs::toCvShare(framemsg);
  if (detection_frame_->image.empty())
    return;

  current_detections = detmsg->detections;
  current_target_id = detmsg->current_target_id;
}


void VideoManager::displayFrames() {
  // Display current frame if available
  if (!detection_frame_ && current_frame_ && !current_frame_->image.empty()) {
    displayFrameWithMetadata(current_frame_->image, "Current Frame",
                             current_frame_->header, "CURRENT");
  } else if (detection_frame_ && !detection_frame_->image.empty()) {
    if ((get_clock()->now() - rclcpp::Time(detection_frame_->header.stamp)) >
        rclcpp::Duration::from_seconds(detection_time_out)) {
      detection_frame_ = nullptr;
    } else {
      std::lock_guard<std::mutex> lock(detection_frame_mutex);
      cv::Mat image = drawBoundingBox(detection_frame_->image,
                                      current_detections, current_target_id);
      displayFrameWithMetadata(image, "Current Frame", detection_frame_->header,
                               "CURRENT");
    }
  }

  // Process OpenCV events
  cv::waitKey(1);
}

void VideoManager::displayFrameWithMetadata(const cv::Mat &frame,
                                            const std::string &window_name,
                                            const std_msgs::msg::Header &header,
                                            const std::string &frame_type) {
  if (frame.empty()) {
    return;
  }

  // Create a copy of the frame to overlay metadata
  cv::Mat display_frame = frame.clone();

  // Resize frame if needed for display
  if (display_frame.cols > DISPLAY_WIDTH ||
      display_frame.rows > DISPLAY_HEIGHT) {
    double scale =
        std::min(static_cast<double>(DISPLAY_WIDTH) / display_frame.cols,
                 static_cast<double>(DISPLAY_HEIGHT) / display_frame.rows);
    cv::resize(display_frame, display_frame, cv::Size(), scale, scale);
  }

  // Prepare metadata text
  std::stringstream metadata_ss;
  metadata_ss << frame_type << " FRAME\n";
  metadata_ss << "Frame ID: " << header.frame_id << "\n";
  metadata_ss << "Timestamp: " << formatTimestamp(header.stamp) << "\n";
  metadata_ss << "Size: " << frame.cols << "x" << frame.rows << "\n";

  // Calculate age of frame
  auto now = this->get_clock()->now();
  auto age_ns = (now - header.stamp).nanoseconds();
  auto age_ms = age_ns / 1000000.0;
  metadata_ss << "Age: " << std::fixed << std::setprecision(1) << age_ms
              << " ms\n";

  // Add current time
  rclcpp::Time current_time = this->now(); // Uses node’s clock
  std::time_t time_t = static_cast<std::time_t>(current_time.seconds());
  metadata_ss << "Sim Time: "
              << std::put_time(std::localtime(&time_t), "%H:%M:%S");

  // Overlay metadata on frame
  // overlayMetadata(display_frame, metadata_ss.str());

  // Display the frame
  cv::imshow(window_name, display_frame);
}

std::string VideoManager::formatTimestamp(const builtin_interfaces::msg::Time &timestamp) {
  // Convert ROS time to system time for display
  rclcpp::Time time_point(timestamp);

  // Convert to time_t (seconds since epoch)
  std::time_t time_t_value = static_cast<std::time_t>(time_point.seconds());

  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t_value), "%H:%M:%S");

  // Add milliseconds
  int ms = static_cast<int>((time_point.nanoseconds() % 1000000000) / 1000000);
  ss << "." << std::setw(3) << std::setfill('0') << ms;

  return ss.str();
}

void VideoManager::overlayMetadata(cv::Mat &frame,
                                   const std::string &metadata_text) {
  // Split metadata text into lines
  std::vector<std::string> lines;
  std::stringstream ss(metadata_text);
  std::string line;
  while (std::getline(ss, line)) {
    lines.push_back(line);
  }

  // Set up text parameters
  int font_face = cv::FONT_HERSHEY_SIMPLEX;
  double font_scale = 0.5;
  int thickness = 1;
  cv::Scalar text_color(0, 255, 0); // Green text
  cv::Scalar bg_color(0, 0, 0);     // Black background

  // Calculate text size for background rectangle
  int max_width = 0;
  int total_height = 0;
  int line_height = 0;

  for (const auto &text_line : lines) {
    int baseline = 0;
    cv::Size text_size =
        cv::getTextSize(text_line, font_face, font_scale, thickness, &baseline);
    max_width = std::max(max_width, text_size.width);
    line_height = text_size.height + baseline;
    total_height += line_height + 5; // 5 pixels spacing between lines
  }

  // Draw background rectangle
  cv::Rect bg_rect(10, 10, max_width + 20, total_height + 10);
  cv::rectangle(frame, bg_rect, bg_color, -1);
  cv::rectangle(frame, bg_rect, text_color, 1);

  // Draw text lines
  int y_offset = 30;
  for (const auto &text_line : lines) {
    cv::putText(frame, text_line, cv::Point(20, y_offset), font_face,
                font_scale, text_color, thickness);
    y_offset += line_height + 5;
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<VideoManager>();

    RCLCPP_INFO(node->get_logger(), "Starting VideoManager node...");
    rclcpp::spin(node);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("video_manager"), "Exception in main: %s",
                 e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}