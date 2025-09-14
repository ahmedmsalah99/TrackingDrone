#include "video_manager/video_manager.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

VideoManager::VideoManager(const rclcpp::NodeOptions& options)
    : Node("video_manager", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing VideoManager node");
    
    // Create subscriptions to the stream manager topics
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    
    current_frame_subscription_ = this->create_subscription<shm_msgs::msg::Image1m>(
        "/stream_manager/current_frame", 
        qos,
        std::bind(&VideoManager::currentFrameCallback, this, std::placeholders::_1)
    );
    
    delayed_frame_subscription_ = this->create_subscription<shm_msgs::msg::Image1m>(
        "/stream_manager/delayed_frame", 
        qos,
        std::bind(&VideoManager::delayedFrameCallback, this, std::placeholders::_1)
    );
    
    // Create timer for display updates (20ms as requested)
    display_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(TIMER_PERIOD_MS)),
        std::bind(&VideoManager::displayTimerCallback, this)
    );
    
    // Initialize OpenCV windows
    cv::namedWindow("Current Frame", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Delayed Frame", cv::WINDOW_AUTOSIZE);
    
    RCLCPP_INFO(this->get_logger(), "VideoManager node initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Subscribed to:");
    RCLCPP_INFO(this->get_logger(), "  - /stream_manager/current_frame");
    RCLCPP_INFO(this->get_logger(), "  - /stream_manager/delayed_frame");
    RCLCPP_INFO(this->get_logger(), "Display timer running at %.1f Hz", 1000.0 / TIMER_PERIOD_MS);
}

VideoManager::~VideoManager()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down VideoManager node");
    cv::destroyAllWindows();
}

void VideoManager::currentFrameCallback(const shm_msgs::msg::Image1m::SharedPtr msg)
{
    try {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        current_frame_ = shm_msgs::toCvShare(msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Received current frame: %dx%d, encoding: %s", 
                    msg->width, msg->height, msg->encoding.data.data());
        
        // Calculate transport time
        auto now = this->get_clock()->now();
        auto transport_time_ns = (now - current_frame_->header.stamp).nanoseconds();
        auto transport_time_ms = transport_time_ns / 1000000.0;
        
        RCLCPP_DEBUG(this->get_logger(), "Current frame transport time: %.3f ms", transport_time_ms);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing current frame: %s", e.what());
    }
}

void VideoManager::delayedFrameCallback(const shm_msgs::msg::Image1m::SharedPtr msg)
{
    try {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        delayed_frame_ = shm_msgs::toCvShare(msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Received delayed frame: %dx%d, encoding: %s", 
                    msg->width, msg->height, msg->encoding.data.data());
        
        // Calculate transport time
        auto now = this->get_clock()->now();
        auto transport_time_ns = (now - delayed_frame_->header.stamp).nanoseconds();
        auto transport_time_ms = transport_time_ns / 1000000.0;
        
        RCLCPP_DEBUG(this->get_logger(), "Delayed frame transport time: %.3f ms", transport_time_ms);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing delayed frame: %s", e.what());
    }
}

void VideoManager::displayTimerCallback()
{
    std::lock_guard<std::mutex> lock(frame_mutex_);
    
    // Display current frame if available
    if (current_frame_ && !current_frame_->image.empty()) {
        displayFrameWithMetadata(current_frame_->image, "Current Frame", 
                               current_frame_->header, "CURRENT");
    }
    
    // Display delayed frame if available
    if (delayed_frame_ && !delayed_frame_->image.empty()) {
        displayFrameWithMetadata(delayed_frame_->image, "Delayed Frame", 
                               delayed_frame_->header, "DELAYED");
    }
    
    // Process OpenCV events
    cv::waitKey(1);
}

void VideoManager::displayFrameWithMetadata(const cv::Mat& frame, const std::string& window_name, 
                                          const std_msgs::msg::Header& header, const std::string& frame_type)
{
    if (frame.empty()) {
        return;
    }
    
    // Create a copy of the frame to overlay metadata
    cv::Mat display_frame = frame.clone();
    
    // Resize frame if needed for display
    if (display_frame.cols > DISPLAY_WIDTH || display_frame.rows > DISPLAY_HEIGHT) {
        double scale = std::min(static_cast<double>(DISPLAY_WIDTH) / display_frame.cols,
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
    metadata_ss << "Age: " << std::fixed << std::setprecision(1) << age_ms << " ms\n";
    
    // Add current time
    auto current_time = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(current_time);
    metadata_ss << "Display Time: " << std::put_time(std::localtime(&time_t), "%H:%M:%S");
    
    // Overlay metadata on frame
    overlayMetadata(display_frame, metadata_ss.str());
    
    // Display the frame
    cv::imshow(window_name, display_frame);
}

std::string VideoManager::formatTimestamp(const builtin_interfaces::msg::Time& timestamp)
{
    // Convert ROS time to system time for display
    auto time_point = std::chrono::system_clock::time_point(
        std::chrono::seconds(timestamp.sec) + std::chrono::nanoseconds(timestamp.nanosec)
    );
    auto time_t = std::chrono::system_clock::to_time_t(time_point);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%H:%M:%S");
    
    // Add milliseconds
    auto ms = (timestamp.nanosec / 1000000) % 1000;
    ss << "." << std::setfill('0') << std::setw(3) << ms;
    
    return ss.str();
}

void VideoManager::overlayMetadata(cv::Mat& frame, const std::string& metadata_text)
{
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
    
    for (const auto& text_line : lines) {
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(text_line, font_face, font_scale, thickness, &baseline);
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
    for (const auto& text_line : lines) {
        cv::putText(frame, text_line, cv::Point(20, y_offset), 
                   font_face, font_scale, text_color, thickness);
        y_offset += line_height + 5;
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<VideoManager>();
        
        RCLCPP_INFO(node->get_logger(), "Starting VideoManager node...");
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("video_manager"), "Exception in main: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}