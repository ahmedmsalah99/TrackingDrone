#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
// #include <cv_bridge/cv_bridge.h>
#include "shm_msgs/msg/image.hpp"
#include "shm_msgs/opencv_conversions.hpp"
#include <common_msgs/msg/detections.hpp>
#include <common_msgs/msg/change_target.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <memory>
#include <vector>
#include <detection_and_mot/nanoflann.hpp>
#include <detection_and_mot/YOLO.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace nanoflann;

class DetectionAndMOTNode : public rclcpp::Node {
public:
    DetectionAndMOTNode() : Node("detection_and_mot_node") {
        // Initialize parameters
        std::string shared_path = ament_index_cpp::get_package_share_directory("detection_and_mot");
        declare_parameter<std::string>("model_path", shared_path + "/resources/yolo_nas_s.onnx");
        declare_parameter<std::string>("labels_path", shared_path + "/resources/coco.names");
        declare_parameter<bool>("use_gpu", false);
        declare_parameter<int>("cache_size", 30);
        declare_parameter<double>("detection_fps", 2.0);
        declare_parameter<double>("tracking_fps", 30.0);

        // Get parameters
        std::string model_path = get_parameter("model_path").as_string();
        std::string labels_path = get_parameter("labels_path").as_string();
        bool use_gpu = get_parameter("use_gpu").as_bool();
        cache_size_ = get_parameter("cache_size").as_int();
        detection_fps_ = get_parameter("detection_fps").as_double();
        tracking_fps_ = get_parameter("tracking_fps").as_double();

        // Initialize YOLO detector
        detector_ = std::make_unique<YOLODetector>(model_path, labels_path, use_gpu);

        // Initialize K-D tree
        // kd_tree_ = std::make_unique<KDTree>();

        // Setup subscriptions and publishers
        setupCommunications();

        // Setup timers
        detection_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / detection_fps_)),
            std::bind(&DetectionAndMOTNode::detectionCallback, this));

        tracking_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / tracking_fps_)),
            std::bind(&DetectionAndMOTNode::trackingCallback, this));

        RCLCPP_INFO(get_logger(), "DetectionAndMOTNode initialized");
    }

private:
    void setupCommunications() {
        // Subscriber for current frame with cache
        frame_sub_ = std::make_shared<message_filters::Subscriber<shm_msgs::msg::Image1m>>(this, "/stream_manager/current_frame");
        frame_cache_ = std::make_shared<message_filters::Cache<shm_msgs::msg::Image1m>>(*frame_sub_, cache_size_);
        frame_cache_->registerCallback(std::bind(&DetectionAndMOTNode::frameCallback, this, std::placeholders::_1));

        // Publisher for detections
        detections_pub_ = create_publisher<common_msgs::msg::Detections>("detections", 10);

        // Subscriber for target change commands
        target_change_sub_ = create_subscription<common_msgs::msg::ChangeTarget>(
            "change_target", 10,
            std::bind(&DetectionAndMOTNode::targetChangeCallback, this, std::placeholders::_1));
    }

    void frameCallback(const shm_msgs::msg::Image1m::ConstSharedPtr& msg) {
        // Store previous frame for tracking
        if (current_frame_ && !current_frame_->image.empty()) {
            cv::cvtColor(current_frame_->image, prev_frame_gray_, cv::COLOR_BGR2GRAY);
        }
        current_frame_ = shm_msgs::toCvShare(msg);
        current_stamp_ = msg->header.stamp;
    }

    void detectionCallback() {
        if (!current_frame_ || current_frame_->image.empty()) {
            return;
        }
        // Store detection frame and points
        detection_frame_ = current_frame_->image.clone();
        detection_stamp_ = current_stamp_;
        // Run detection
        std::vector<Detection> detections = detector_->detect(current_frame_->image);

        auto msg = std::make_unique<common_msgs::msg::Detections>();
        msg->stamp = get_clock()->now();
        msg->current_target_id = current_target_id_;

        for (const auto& det : detections) {
            common_msgs::msg::Detection det_msg;
            det_msg.x = det.box.x;
            det_msg.y = det.box.y;
            det_msg.width = det.box.width;
            det_msg.height = det.box.height;
            det_msg.conf = det.conf;
            det_msg.class_id = det.classId;
            msg->detections.push_back(det_msg);
        }

        detections_pub_->publish(std::move(msg));

        detection_points_.clear();
        for (const auto& det : detections) {
            cv::Point2f center(det.box.x + det.box.width / 2.0f, det.box.y + det.box.height / 2.0f);
            detection_points_.push_back(center);
        }

        // Track detections to present
        // trackDetectionsToPresent(detections);
    }

    void trackDetectionsToPresent(std::vector<Detection> detections){
        if (detections.empty()) {
            tracked_detections_.clear();
            return;
        }

        // Get messages from cache after detection_stamp
        rclcpp::Time now = get_clock()->now();
        std::vector<shm_msgs::msg::Image1m::ConstSharedPtr> msgs = frame_cache_->getInterval(detection_stamp_, now);

        // Start with detection frame
        cv::Mat prev_gray;
        cv::cvtColor(detection_frame_, prev_gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> prev_points = detection_points_;
        std::vector<Detection> current_detections = detections;

        // Track through each frame in the interval
        for (const auto& msg : msgs) {
            rclcpp::Time msg_time(msg->header.stamp);
            rclcpp::Time det_time(detection_stamp_);
            if (msg_time <= det_time) continue; // Skip frames before or at detection

            shm_msgs::CvImageConstPtr cv_img = shm_msgs::toCvShare(msg);
            if (cv_img->image.empty()) continue;

            cv::Mat curr_gray;
            cv::cvtColor(cv_img->image, curr_gray, cv::COLOR_BGR2GRAY);

            std::vector<cv::Point2f> curr_points;
            std::vector<uchar> status;
            std::vector<float> err;
            cv::calcOpticalFlowPyrLK(prev_gray, curr_gray, prev_points, curr_points, status, err);

            // Update detection positions
            for (size_t i = 0; i < current_detections.size() && i < curr_points.size(); ++i) {
                if (status[i]) {
                    cv::Point2f delta = curr_points[i] - prev_points[i];
                    current_detections[i].box.x += delta.x;
                    current_detections[i].box.y += delta.y;
                }
            }

            // Update for next iteration
            prev_gray = curr_gray.clone();
            prev_points = curr_points;
        }

        // Set tracked detections
        tracked_detections_ = current_detections;

        // Initialize real-time tracking
        prev_frame_gray_ = prev_gray.clone();
        prev_points_ = prev_points;
        last_detection_time_ = now;
    }

    void trackingCallback() {
        if (prev_frame_gray_.empty() || current_frame_->image.empty() || prev_points_.empty()) {
            return;
        }

        // Convert current frame to gray
        cv::Mat curr_gray;
        cv::cvtColor(current_frame_->image, curr_gray, cv::COLOR_BGR2GRAY);

        // Track points using Lucas-Kanade
        std::vector<cv::Point2f> curr_points;
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_frame_gray_, curr_gray, prev_points_, curr_points, status, err);

        // Update tracked detections based on point movements
        for (size_t i = 0; i < tracked_detections_.size() && i < curr_points.size(); ++i) {
            if (status[i]) {
                cv::Point2f delta = curr_points[i] - prev_points_[i];
                tracked_detections_[i].box.x += delta.x;
                tracked_detections_[i].box.y += delta.y;
            }
        }

        // Update for next iteration
        prev_frame_gray_ = curr_gray.clone();
        prev_points_ = curr_points;

        // Publish updated detections
        publishTrackings();
    }

    void targetChangeCallback(const common_msgs::msg::ChangeTarget::SharedPtr msg) {
        // Find new target based on direction
        // int new_target_id = findNewTarget(msg->vertical, msg->horizontal);
        // current_target_id_ = new_target_id;

        RCLCPP_INFO(get_logger(), "Changed target to ID: %d", current_target_id_);
    }

    void publishTrackings() {
        auto msg = std::make_unique<common_msgs::msg::Detections>();
        msg->stamp = get_clock()->now();
        msg->current_target_id = current_target_id_;

        for (const auto& det : tracked_detections_) {
            common_msgs::msg::Detection det_msg;
            det_msg.x = det.box.x;
            det_msg.y = det.box.y;
            det_msg.width = det.box.width;
            det_msg.height = det.box.height;
            det_msg.conf = det.conf;
            det_msg.class_id = det.classId;
            msg->detections.push_back(det_msg);
        }

        // detections_pub_->publish(std::move(msg));
    }

    // void updateKDTree(const std::vector<Detection>& detections) {
    //     kd_tree_->clear();
    //     for (size_t i = 0; i < detections.size(); ++i) {
    //         const auto& det = detections[i];
    //         float center_x = det.box.x + det.box.width / 2.0f;
    //         float center_y = det.box.y + det.box.height / 2.0f;
    //         kd_tree_->addPoint({center_x, center_y}, i);
    //     }
    //     kd_tree_->buildIndex();
    // }

    // void updateKDTreeWithTracks(const std::vector<std::vector<float>>& tracks) {
    //     kd_tree_->clear();
    //     for (size_t i = 0; i < tracks.size(); ++i) {
    //         const auto& track = tracks[i];
    //         kd_tree_->addPoint({track[0] + track[2]/2.0f, track[1] + track[3]/2.0f}, i);
    //     }
    //     kd_tree_->buildIndex();
    // }

    // int findNewTarget(int8_t vertical, int8_t horizontal) {
    //     if (kd_tree_->empty()) {
    //         return -1;
    //     }

    //     // Get current target position
    //     std::vector<float> current_pos;
    //     if (current_target_id_ >= 0 && current_target_id_ < static_cast<int>(kd_tree_->size())) {
    //         current_pos = kd_tree_->getPoint(current_target_id_);
    //     } else {
    //         // Default to center if no current target
    //         current_pos = {static_cast<float>(current_frame_->image.cols / 2), static_cast<float>(current_frame_->image.rows / 2)};
    //     }

    //     // Calculate direction vector
    //     float dx = horizontal * 50.0f;  // Adjust step size as needed
    //     float dy = vertical * 50.0f;
    //     std::vector<float> target_pos = {current_pos[0] + dx, current_pos[1] + dy};

    //     // Find nearest neighbor in the direction
    //     size_t nearest_idx;
    //     float dist_sq;
    //     kd_tree_->knnSearch(target_pos.data(), 1, &nearest_idx, &dist_sq);

    //     return static_cast<int>(nearest_idx);
    // }

    // K-D Tree implementation using nanoflann
//     class KDTree {
//     public:
//         struct Point {
//             float x, y;
//         };

//         void addPoint(const Point& p, size_t id) {
//             points_.push_back(p);
//             ids_.push_back(id);
//         }

//         void buildIndex() {
//             if (!points_.empty()) {
//                 index_ = std::make_unique<KDTreeIndex>(2, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10));
//                 index_->buildIndex();
//             }
//         }

//         void knnSearch(const float* query, size_t num_results, size_t* indices, float* dists_sq) {
//             if (index_) {
//                 index_->knnSearch(query, num_results, indices, dists_sq);
//             }
//         }

//         Point getPoint(size_t idx) const {
//             return points_[idx];
//         }

//         size_t size() const {
//             return points_.size();
//         }

//         bool empty() const {
//             return points_.empty();
//         }

//         void clear() {
//             points_.clear();
//             ids_.clear();
//             index_.reset();
//         }

//     private:
//         struct KDTreeIndex : public KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, KDTree>, KDTree, 2> {
//             KDTreeIndex(size_t n_dims, KDTree& tree, const KDTreeSingleIndexAdaptorParams& params)
//                 : KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, KDTree>, KDTree, 2>(n_dims, tree, params) {}
//         };

//         std::vector<Point> points_;
//         std::vector<size_t> ids_;
//         std::unique_ptr<KDTreeIndex> index_;

//         inline size_t kdtree_get_point_count() const { return points_.size(); }
//         inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
//             return dim == 0 ? points_[idx].x : points_[idx].y;
//         }
//         template <class BBOX>
//         bool kdtree_get_bbox(BBOX&) const { return false; }
//     };

    // Members
    std::unique_ptr<YOLODetector> detector_;
    // std::unique_ptr<KDTree> kd_tree_;
    shm_msgs::CvImageConstPtr current_frame_;
    int current_target_id_ = -1;
    size_t cache_size_;
    double detection_fps_;
    double tracking_fps_;

    // Lucas-Kanade tracking members
    cv::Mat prev_frame_gray_;
    std::vector<cv::Point2f> prev_points_;
    std::vector<Detection> tracked_detections_;
    rclcpp::Time last_detection_time_;
    builtin_interfaces::msg::Time current_stamp_;
    cv::Mat detection_frame_;
    std::vector<cv::Point2f> detection_points_;
    builtin_interfaces::msg::Time detection_stamp_;

    // ROS2 interfaces
    std::shared_ptr<message_filters::Subscriber<shm_msgs::msg::Image1m>> frame_sub_;
    std::shared_ptr<message_filters::Cache<shm_msgs::msg::Image1m>> frame_cache_;
    rclcpp::Publisher<common_msgs::msg::Detections>::SharedPtr detections_pub_;
    rclcpp::Subscription<common_msgs::msg::ChangeTarget>::SharedPtr target_change_sub_;
    rclcpp::TimerBase::SharedPtr detection_timer_;
    rclcpp::TimerBase::SharedPtr tracking_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DetectionAndMOTNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
