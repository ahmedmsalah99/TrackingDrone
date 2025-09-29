#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
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
#include <Eigen/Dense>
#include <ocsort/OCSort.hpp>

using namespace nanoflann;


class DetectionAndMOTNode : public rclcpp::Node {
public:
    DetectionAndMOTNode() : Node("detection_and_mot_node") {
        // Initialize parameters
        std::string shared_path = ament_index_cpp::get_package_share_directory("detection_and_mot");
        declare_parameter<std::string>("model_path", shared_path + "/resources/best3_nano.onnx");
        declare_parameter<std::string>("labels_path", shared_path + "/resources/coco_mod.names");
        declare_parameter<bool>("use_gpu", false);
        declare_parameter<int>("cache_size", 30);
        declare_parameter<double>("detection_fps", 10.0);
        declare_parameter<double>("delay_time", 0.25);

        // Get parameters
        std::string model_path = get_parameter("model_path").as_string();
        std::string labels_path = get_parameter("labels_path").as_string();
        bool use_gpu = get_parameter("use_gpu").as_bool();
        cache_size_ = get_parameter("cache_size").as_int();
        detection_fps_ = get_parameter("detection_fps").as_double();
        delay_time = rclcpp::Duration::from_seconds(get_parameter("delay_time").as_double());

        // Initialize YOLO detector
        detector_ = std::make_unique<YOLODetector>(model_path, labels_path, use_gpu);

        // Initialize K-D tree
        // kd_tree_ = std::make_unique<KDTree>();
        detection_timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        frame_sub_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // Setup subscriptions and publishers
        setupCommunications();

        // Setup timers
        detection_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / detection_fps_)),
            std::bind(&DetectionAndMOTNode::detectionCallback, this),
            detection_timer_group_);

        
        // Initialize OCSort tracker
        // min_hits_ is minimum number of consecutive detections needed before a track is considered valid.
        // delta_t_ is max number of frames back the tracker looks when trying to re-associate a detection with an existing track.
        // Higher inertia → track relies more on past motion when associating
        // Whether to use ByteTrack-style association
        //GIoU = Generalized IoU, an improvement over IoU that accounts for cases where boxes don’t overlap.
        oc_tracker_ = std::make_shared<ocsort::OCSort>(0.2, 10, 3, 0.3, 10, "giou", 0.2, true);

        RCLCPP_INFO(get_logger(), "DetectionAndMOTNode initialized");
    }

private:
    void setupCommunications() {
        // Subscriber for current frame with cache
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = frame_sub_group_;
        frame_sub_ = std::make_shared<message_filters::Subscriber<shm_msgs::msg::Image1m>>(this, "/stream_manager/current_frame",rmw_qos_profile_default,sub_options);
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
        // update current frame
        current_frame_ = shm_msgs::toCvShare(msg);
        current_stamp_ = rclcpp::Time(msg->header.stamp);
        // track the frame 
        bool new_det = false;
        // check if a detection is available and consume it
        if(current_detections_stamp_.nanoseconds() != 0 && new_dets_available_){
            rclcpp::Time time_now = get_clock()->now() - delay_time;
            // if the current detection's time is In past and in the delay time window
            if(time_now > current_detections_stamp_ && (time_now - current_detections_stamp_) < delay_time){
                // new detection available - update our tracked detection
                new_det = true;
                {
                    std::lock_guard<std::mutex> lock(det_mutex);
                    tracked_detections_ = current_detections_;
                    new_dets_available_ = false;
                }
            }
        }
        
        if(new_det){
            // make a new message from the detection and update the MOT tracker
            Eigen::Matrix<float, Eigen::Dynamic, 6> data(tracked_detections_.size(),6);
            data = detectionsToMatrix(tracked_detections_);
            std::vector<Eigen::RowVectorXf> res = oc_tracker_->update(data);
            detections_msg = tracksToMsg(res);
        }
        // always publish new message
        detections_msg.stamp = get_clock()->now() - delay_time;  
        detections_pub_->publish(detections_msg);
    }
    Eigen::Matrix<float, Eigen::Dynamic, 6> detectionsToMatrix(
        const std::vector<Detection> &detections)
    {
        Eigen::Matrix<float, Eigen::Dynamic, 6> data(detections.size(), 6);

        for (size_t i = 0; i < detections.size(); i++)
        {
            const auto &det = detections[i];
            data(i, 0) = static_cast<float>(det.box.x);
            data(i, 1) = static_cast<float>(det.box.y);
            data(i, 2) = static_cast<float>(det.box.x + det.box.width);
            data(i, 3) = static_cast<float>(det.box.y + det.box.height);
            data(i, 4) = det.conf;
            data(i, 5) = static_cast<float>(det.classId);
        }
        return data;
    }

    common_msgs::msg::Detections tracksToMsg(
        const std::vector<Eigen::RowVectorXf> &tracks)
    {
        common_msgs::msg::Detections msg;
        // Optionally set current_target_id (e.g., first active track)
        msg.current_target_id = -1;
        for (const auto &t : tracks)
        {
            common_msgs::msg::Detection det;
            int x1 = static_cast<int>(t(0));
            int y1 = static_cast<int>(t(1));
            int x2 = static_cast<int>(t(2));
            int y2 = static_cast<int>(t(3));

            det.x = x1;
            det.y = y1;
            det.width = x2 - x1;
            det.height = y2 - y1;
            det.conf = t(6);
            det.class_id = static_cast<int>(t(5));
            
            msg.detections.push_back(det);
        }
        return msg;
    }

    void detectionCallback() {
        if (!current_frame_ || current_frame_->image.empty()) {
            return;
        }
        // Store detection frame and points
        cv::Mat detection_frame = current_frame_->image.clone();
        rclcpp::Time detection_stamp(current_frame_->header.stamp);
        // Run detection
        std::vector<Detection> detections = detector_->detect(detection_frame,0.35);
        std::lock_guard<std::mutex> lock(det_mutex);
        detection_frame_ = detection_frame.clone();

        // assign member variables
        current_detections_ = detections;
        current_detections_stamp_ = get_clock()->now() - delay_time;
        // current_detections_stamp_ = detection_stamp;
        new_dets_available_ = true;
    }
    

    void targetChangeCallback(const common_msgs::msg::ChangeTarget::SharedPtr msg) {
        // Find new target based on direction
        // int new_target_id = findNewTarget(msg->vertical, msg->horizontal);
        // current_target_id_ = new_target_id;

        RCLCPP_INFO(get_logger(), "Changed target to ID: %d", current_target_id_);
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

    // configs
    size_t cache_size_;
    double detection_fps_;
    double tracking_fps_;
    rclcpp::Duration delay_time{0,0};
    // detection params
    std::unique_ptr<YOLODetector> detector_;
    cv::Mat detection_frame_;
    rclcpp::Time current_detections_stamp_;
    bool new_dets_available_ = false;
    std::vector<Detection> tracked_detections_;
    std::vector<Detection> current_detections_;
    common_msgs::msg::Detections detections_msg;
    // current state
    shm_msgs::CvImageConstPtr current_frame_;
    int current_target_id_ = -1;
    rclcpp::Time current_stamp_;
    
    // Tracker
    std::shared_ptr<ocsort::OCSort> oc_tracker_;

    // ROS2 interfaces
    std::shared_ptr<message_filters::Subscriber<shm_msgs::msg::Image1m>> frame_sub_;
    std::shared_ptr<message_filters::Cache<shm_msgs::msg::Image1m>> frame_cache_;
    rclcpp::Publisher<common_msgs::msg::Detections>::SharedPtr detections_pub_;
    rclcpp::Subscription<common_msgs::msg::ChangeTarget>::SharedPtr target_change_sub_;
    rclcpp::TimerBase::SharedPtr detection_timer_;

    // Synchronization
    rclcpp::CallbackGroup::SharedPtr detection_timer_group_;
    rclcpp::CallbackGroup::SharedPtr frame_sub_group_;
    std::mutex det_mutex;

    // std::unique_ptr<KDTree> kd_tree_;
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
