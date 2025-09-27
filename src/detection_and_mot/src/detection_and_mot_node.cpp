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
#include <Eigen/Dense>
#include <ocsort/OCSort.hpp>

using namespace nanoflann;

struct FloatBoundingBox
{
    float x;
    float y;
    float width;
    float height;

    FloatBoundingBox() : x(0), y(0), width(0), height(0) {}
    FloatBoundingBox(float x_, float y_, float width_, float height_)
        : x(x_), y(y_), width(width_), height(height_) {}
};

/**
 * @brief Struct to represent a detection.
 */
struct FloatDetection
{
    FloatBoundingBox box;
    float conf{};
    int classId{};
    FloatDetection(FloatBoundingBox box, float conf, int classId)
        : box(box), classId(classId), conf(conf) {}
    FloatDetection(const Detection& d)
    {
        box.x = static_cast<float>(d.box.x);
        box.y = static_cast<float>(d.box.y);
        box.width = static_cast<float>(d.box.width);
        box.height = static_cast<float>(d.box.height);
        conf = d.conf;
        classId = d.classId;
    }
};


class DetectionAndMOTNode : public rclcpp::Node {
public:
    DetectionAndMOTNode() : Node("detection_and_mot_node") {
        // Initialize parameters
        std::string shared_path = ament_index_cpp::get_package_share_directory("detection_and_mot");
        declare_parameter<std::string>("model_path", shared_path + "/resources/best3.onnx");
        declare_parameter<std::string>("labels_path", shared_path + "/resources/coco_mod.names");
        declare_parameter<bool>("use_gpu", false);
        declare_parameter<int>("cache_size", 30);
        declare_parameter<double>("detection_fps", 2.0);
        declare_parameter<double>("tracking_fps", 70.0);

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
        detection_timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        frame_sub_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
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
        oc_tracker_ = std::make_shared<ocsort::OCSort>(0.2, 25, 10, 0.3, 25, "giou", 0.2, false);
        
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
        // Store previous frame for tracking before updating it
        // if (current_frame_ && !current_frame_->image.empty()) {
        //     cv::cvtColor(current_frame_->image, prev_frame_gray_, cv::COLOR_BGR2GRAY);
        // }

        // update current frame
        current_frame_ = shm_msgs::toCvShare(msg);
        current_stamp_ = msg->header.stamp;
        // track the frame 
        std::lock_guard<std::mutex> lock(tracking_mutex);
        trackFrame();
        if(!tracked_detections_.empty()){
            // std::cout << "publishing tracked frames" << std::endl;
            Eigen::Matrix<float, Eigen::Dynamic, 6> data(tracked_detections_.size(),6);
            data = detectionsToMatrix(tracked_detections_);
            

            std::vector<Eigen::RowVectorXf> res = oc_tracker_->update(data);
            // for (const auto& row : res) {
            //     std::cout << row << std::endl;
            // }
            // std::cout << " " << std::endl;
            common_msgs::msg::Detections detections_msg = tracksToMsg(res);
            detections_pub_->publish(detections_msg);
        }else{
            std::cout << "tracked frames are empty for some reason" << std::endl;
        }
    }
    Eigen::Matrix<float, Eigen::Dynamic, 6> detectionsToMatrix(
        const std::vector<FloatDetection>& detections)
    {
        Eigen::Matrix<float, Eigen::Dynamic, 6> data(detections.size(), 6);

        for (size_t i = 0; i < detections.size(); i++)
        {
            const auto& det = detections[i];
            data(i, 0) = det.box.x;
            data(i, 1) = det.box.y;
            data(i, 2) = det.box.x + det.box.width;
            data(i, 3) = det.box.y + det.box.height;
            data(i, 4) = det.conf;
            data(i, 5) = static_cast<float>(det.classId);
        }
        return data;
    }

    common_msgs::msg::Detections tracksToMsg(
        const std::vector<Eigen::RowVectorXf>& tracks)
    {
        common_msgs::msg::Detections msg;
        msg.stamp = get_clock()->now();  

        // Optionally set current_target_id (e.g., first active track)
        msg.current_target_id = -1;

        for (const auto& t : tracks)
        {
            common_msgs::msg::Detection det;
            int x1 = static_cast<int>(t(0));
            int y1 = static_cast<int>(t(1));
            int x2 = static_cast<int>(t(2));
            int y2 = static_cast<int>(t(3));

            det.x       = x1;
            det.y       = y1;
            det.width   = x2 - x1;
            det.height  = y2 - y1;
            det.conf    = t(6);
            det.class_id = static_cast<int>(t(5));

            msg.detections.push_back(det);
        }

        return msg;
    }



    void detectionCallback() {
        if (!current_frame_ || current_frame_->image.empty()) {
            std::cout << "current_frame_->image.empty() " << current_frame_->image.empty() << std::endl;
            return;
        }
        // Store detection frame and points
        cv::Mat detection_frame_ = current_frame_->image.clone();
        rclcpp::Time detection_stamp(current_frame_->header.stamp);
        // Run detection
        rclcpp::Time now = get_clock()->now();
        auto oldest = frame_cache_->getOldestTime();
        auto latest = frame_cache_->getLatestTime();
        auto start  = detection_stamp + rclcpp::Duration::from_nanoseconds(10);
        // std::cout << "11Cache oldest time " << oldest.seconds() << "s "
        //   << oldest.nanoseconds() % 1000000000 << "ns "
        //   << " latest time " << latest.seconds() << "s "
        //   << latest.nanoseconds() % 1000000000 << "ns "
        //   << " my interval start " << start.seconds() << "s "
        //   << start.nanoseconds() % 1000000000 << "ns "
        //   << " and ends " << now.seconds() << "s "
        //   << now.nanoseconds() % 1000000000 << "ns"
        //   << std::endl;

        // saveTrackedDetections(detection_frame_
        //     ,
        //     std::vector<Detection>(),
        //     std::vector<cv::Point2f>(),
        //     "/home/stark/stuff/Projects/TrackingDrone/ros2_ws/results"
        // );
        std::vector<Detection> detections = detector_->detect(detection_frame_,0.35);

        // Track detections to present
        trackDetectionsToPresent(detections,detection_frame_ ,detection_stamp);
    }
    void trackDetectionsToPresent(std::vector<Detection>& detections,cv::Mat& detection_frame_, rclcpp::Time detection_stamp_){
        if (detections.empty()) {
            RCLCPP_WARN(get_logger(), "Tracking detections to present found no detections ");
            return;
        }
        RCLCPP_INFO(get_logger(), "Started tracking to present with %ld detections.",detections.size());
        rclcpp::Time now = get_clock()->now();
        // Get messages from cache after detection_stamp
        std::vector<shm_msgs::msg::Image1m::ConstSharedPtr> msgs = 
                                        frame_cache_->getInterval(detection_stamp_+rclcpp::Duration::from_nanoseconds(10), now);
        auto oldest = frame_cache_->getOldestTime();
        auto latest = frame_cache_->getLatestTime();
        auto start  = detection_stamp_ + rclcpp::Duration::from_nanoseconds(10);
        // std::cout << "Cache oldest time " << oldest.seconds() << "s "
        //   << oldest.nanoseconds() % 1000000000 << "ns "
        //   << " latest time " << latest.seconds() << "s "
        //   << latest.nanoseconds() % 1000000000 << "ns "
        //   << " my interval start " << start.seconds() << "s "
        //   << start.nanoseconds() % 1000000000 << "ns "
        //   << " and ends " << now.seconds() << "s "
        //   << now.nanoseconds() % 1000000000 << "ns"
        //   << std::endl;

        
        // std::cout << "period between " << (now - detection_stamp_).seconds() << std::endl;
        // std::cout << "there are " << msgs.size() << " right now" << std::endl;
        std::vector<cv::Mat> images;
        images.push_back(detection_frame_);
        
        // Track through each frame in the interval
        for (const auto& msg : msgs) {
            shm_msgs::CvImageConstPtr cv_img = shm_msgs::toCvShare(msg);
            if (cv_img->image.empty()) continue;
            images.push_back(cv_img->image);
        }
        std::lock_guard<std::mutex> lock(tracking_mutex);
        // Set tracked detections
        trackFrames(images,detections,prev_frame_gray_,prev_points_);
        // saveTrackedDetections(images, detections, prev_points_, "/home/stark/stuff/Projects/TrackingDrone/ros2_ws/results");
        tracked_detections_.clear();
        tracked_detections_.reserve(detections.size());
        

        std::transform(
            detections.begin(), detections.end(),
            std::back_inserter(tracked_detections_),
            [](const Detection& d){ return FloatDetection(d); });
        RCLCPP_INFO(get_logger(), "Tracking to present on %ld images found %ld detections.",images.size(),tracked_detections_.size());
    }
// Helper: generate timestamp string
std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S")
        << "_" << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
}
void saveTrackedDetections(
    const cv::Mat& img,
    const std::vector<Detection>& detections,
    std::vector<cv::Point2f> points,
    const std::string& out_dir
) {

    cv::Mat final_img = img.clone();
    for (const auto& det : detections) {
        // Convert your FloatBoundingBox into cv::Rect
        cv::Rect rect(
            static_cast<int>(det.box.x),
            static_cast<int>(det.box.y),
            static_cast<int>(det.box.width),
            static_cast<int>(det.box.height)
        );

        // Draw bounding box
        cv::rectangle(final_img, rect, cv::Scalar(0, 255, 0), 2);
        for (const auto& pt : points) {
            cv::circle(final_img, pt, 3, cv::Scalar(0, 0, 255), -1); // red dots
        }
        // Optional: put text (classId and confidence)
        std::ostringstream label;
        label << "ID:" << det.classId << " " << std::fixed << std::setprecision(2) << det.conf;
        cv::putText(final_img, label.str(),
                    cv::Point(rect.x, rect.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 0), 1);
    }

    // Build filename with timestamp
    std::string filename = out_dir + "/tracked_" + getTimestamp() + ".png";
    cv::imwrite(filename, final_img);
}
    void trackFrames(const std::vector<cv::Mat>& images,std::vector<Detection>& detections,cv::Mat& final_grey,std::vector<cv::Point2f>& final_points){
        if(images.size()<2 || detections.size() ==0 ){
            final_points.clear();
            for (const auto& det : detections) {
                // cv::Point2f center(det.box.x + det.box.width / 2.0f, det.box.y + det.box.height / 2.0f);
                cv::Point2f center(det.box.x, det.box.y);
                final_points.push_back(center);
            }
            cv::cvtColor(images[0], final_grey, cv::COLOR_BGR2GRAY);
            RCLCPP_INFO(get_logger(),"No sufficient data for present tracking. There are %ld images and %ld detections.",
            images.size(),detections.size());
            return;
        }
        RCLCPP_INFO(get_logger(),"Sufficient data for present tracking. There are %ld images and %ld detections.",
            images.size(),detections.size());
        
        std::vector<cv::Point2f> prev_points;
        cv::Mat first_gray;
        cv::cvtColor(images[0], first_gray, cv::COLOR_BGR2GRAY);
        for (const auto& det : detections) {
            // cv::Point2f center(det.box.x + det.box.width / 2.0f, det.box.y + det.box.height / 2.0f);
            cv::Point2f center(det.box.x, det.box.y);
            prev_points.push_back(center);
        }
        cv::Mat prev_gray;
        std::vector<uchar> status;
        std::vector<float> err;

        for(size_t i=1;i<images.size();i++){
            cv::cvtColor(images[i-1], prev_gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(images[i], final_grey, cv::COLOR_BGR2GRAY);

            cv::calcOpticalFlowPyrLK(prev_gray, final_grey, prev_points, final_points, status, err);
            
            // Update detection positions
            for (size_t i = 0; i < detections.size() && i < final_points.size(); ++i) {
                if (status[i]) {
                    cv::Point2f delta = final_points[i] - prev_points[i];
                    // detections[i].box.x += delta.x;
                    // detections[i].box.y += delta.y;
                    detections[i].box.x = final_points[i].x;
                    detections[i].box.y = final_points[i].y;

                }
            }
            saveTrackedDetections(images[i], detections, final_points, "/home/stark/stuff/Projects/TrackingDrone/ros2_ws/results");
            prev_points = final_points;
        }
    }
    
    void trackFrame() {
        if (prev_frame_gray_.empty() || current_frame_->image.empty() || prev_points_.empty()) {
            std::cout << "can't track frames"<< std::endl;
            std::cout << "prev_frame_gray_.empty() " << prev_frame_gray_.empty() << " current_frame_->image.empty() " <<  current_frame_->image.empty() << " prev_points_.empty() " << prev_points_.empty() <<std::endl;
            return;
        }

        // Convert current frame to gray
        cv::Mat curr_gray;
        cv::cvtColor(current_frame_->image, curr_gray, cv::COLOR_BGR2GRAY);
        builtin_interfaces::msg::Time tracking_frame_stamp = current_frame_->header.stamp;
        // Track points using Lucas-Kanade
        std::vector<cv::Point2f> curr_points;
        std::vector<uchar> status;
        std::vector<float> err;

        
        cv::calcOpticalFlowPyrLK(prev_frame_gray_, curr_gray, prev_points_, curr_points, status, err);
        // Update tracked detections based on point movements
        for (size_t i = 0; i < tracked_detections_.size() && i < curr_points.size(); ++i) {
            if (status[i]) {
                cv::Point2f delta = curr_points[i] - prev_points_[i];
                // tracked_detections_[i].box.x += delta.x;
                // tracked_detections_[i].box.y += delta.y;
                tracked_detections_[i].box.x = curr_points[i].x;
                tracked_detections_[i].box.y = curr_points[i].y;
            }
        }

        // Update for next iteration
        prev_frame_gray_ = curr_gray.clone();
        prev_points_ = curr_points;

        // Publish updated detections
        // publishTrackings(tracking_frame_stamp);
    }

    void targetChangeCallback(const common_msgs::msg::ChangeTarget::SharedPtr msg) {
        // Find new target based on direction
        // int new_target_id = findNewTarget(msg->vertical, msg->horizontal);
        // current_target_id_ = new_target_id;

        RCLCPP_INFO(get_logger(), "Changed target to ID: %d", current_target_id_);
    }

    void publishTrackings(builtin_interfaces::msg::Time tracking_frame_stamp) {
        auto msg = std::make_unique<common_msgs::msg::Detections>();
        msg->stamp = tracking_frame_stamp;
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

        detections_pub_->publish(std::move(msg));
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
    std::vector<FloatDetection> tracked_detections_;
    builtin_interfaces::msg::Time current_stamp_;

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
    std::mutex tracking_mutex;
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
