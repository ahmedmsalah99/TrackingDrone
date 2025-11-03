#include "shm_msgs/msg/image.hpp"
#include "shm_msgs/opencv_conversions.hpp"
#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <common_msgs/msg/detections.hpp>
#include <common_msgs/srv/change_target.hpp>
#include <deque>
#include <detection_and_mot/YOLO.hpp>
#include <detection_and_mot/nanoflann.hpp>
#include <detection_and_mot/point.hpp>
#include <memory>
#include <ocsort/OCSort.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, DetPointCloud>, DetPointCloud,
    2 /* dim */
    >;

class DetectionAndMOTNode : public rclcpp::Node {
public:
  DetectionAndMOTNode() : Node("detection_and_mot_node"), current_target_(6) {
    // Initialize parameters
    std::string shared_path =
        ament_index_cpp::get_package_share_directory("detection_and_mot") + "/resources/";
    declare_parameter<std::string>("model_path","best3_nano.onnx");
    declare_parameter<std::string>("labels_path","coco_mod.names");
    declare_parameter<bool>("use_gpu", false);
    declare_parameter<double>("detection_fps", 10.0);
    declare_parameter<double>("delay_time", 0.25);

    // Get parameters
    std::string model_path = shared_path + get_parameter("model_path").as_string();
    std::string labels_path = shared_path + get_parameter("labels_path").as_string();
    bool use_gpu = get_parameter("use_gpu").as_bool();
    detection_fps_ = get_parameter("detection_fps").as_double();
    delay_time =
        rclcpp::Duration::from_seconds(get_parameter("delay_time").as_double());

    // Initialize YOLO detector
    detector_ =
        std::make_unique<YOLODetector>(model_path, labels_path, use_gpu);

    // Initialize K-D tree
    // Detection example_det(BoundingBox(),0,0);
    detection_tree_ = std::make_unique<my_kd_tree_t>(
        2, curr_det_point_cloud_,
        nanoflann::KDTreeSingleIndexAdaptorParams(100));
    detection_timer_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    frame_sub_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    // Setup subscriptions and publishers
    setupCommunications();

    // Setup timers
    auto period = rclcpp::Duration::from_seconds(1.0 / detection_fps_);

    detection_timer_ = rclcpp::create_timer(
        this->get_node_base_interface(),         // node_base
        this->get_node_timers_interface(),       // node_timers
        this->get_clock(),                       // clock (so it can follow /clock)
        period,                                  // rclcpp::Duration
        std::bind(&DetectionAndMOTNode::detectionCallback, this),
        detection_timer_group_                   // optional callback group
    );

    current_target_ << -1, -1, -1, -1, -1, -1;
    // Initialize OCSort tracker
    // min_hits_ is minimum number of consecutive detections needed before a
    // track is considered valid. delta_t_ is max number of frames back the
    // tracker looks when trying to re-associate a detection with an existing
    // track. Higher inertia → track relies more on past motion when associating
    // Whether to use ByteTrack-style association
    // GIoU = Generalized IoU, an improvement over IoU that accounts for cases
    // where boxes don’t overlap.
    oc_tracker_ = std::make_shared<ocsort::OCSort>(0.2, 10, 3, 0.3, 10, "giou",
                                                   0.2, true);
    RCLCPP_INFO(get_logger(), "DetectionAndMOTNode initialized");
  }

private:
  void setupCommunications() {
    // Subscriber for current frame with cache
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = frame_sub_group_;
    // frame_sub_ =
    //     std::make_shared<message_filters::Subscriber<shm_msgs::msg::Image1m>>(
    //         this, "/stream_manager/current_frame", rmw_qos_profile_default,
    //         sub_options);
    // frame_cache_ =
    //     std::make_shared<message_filters::Cache<shm_msgs::msg::Image1m>>(
    //         *frame_sub_, cache_size_);
    // frame_cache_->registerCallback(std::bind(
    //     &DetectionAndMOTNode::frameCallback, this, std::placeholders::_1));

    frame_sub_ = create_subscription<shm_msgs::msg::Image1m>(
        "/stream_manager/current_frame", 10,
        [this](const shm_msgs::msg::Image1m::ConstSharedPtr msg) {
          this->frameCallback(msg);
        },sub_options);
    // Publisher for detections
    detections_pub_ =
        create_publisher<common_msgs::msg::Detections>("detections", 10);

    // Subscriber for target change commands
    target_change_srv_ = create_service<common_msgs::srv::ChangeTarget>(
        "change_target",
        std::bind(&DetectionAndMOTNode::targetChangeService, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

  void frameCallback(const shm_msgs::msg::Image1m::ConstSharedPtr &msg) {
    // update current frame
    current_frame_ = shm_msgs::toCvShare(msg);
    current_stamp_ = rclcpp::Time(msg->header.stamp);
    if (!current_frame_ || current_frame_->image.empty()) {
      return;
    }
    center_x = current_frame_->image.cols / 2;
    center_y = current_frame_->image.rows / 2;
    // track the frame
    // check if a detection is available and consume it
    if (current_detections_stamp_.nanoseconds() != 0 && new_dets_available_) {
      rclcpp::Time time_now = get_clock()->now() - delay_time;
      // if the current detection's time is In past and in the delay time window
      if (time_now > current_detections_stamp_ &&
          (time_now - current_detections_stamp_) < delay_time) {
        // new detection available - update our tracked detection
        Eigen::Matrix<float, Eigen::Dynamic, 6> data(current_detections_.size(),
                                                     6);
        {
          std::lock_guard<std::mutex> lock(det_mutex);
          new_dets_available_ = false;
          // make a new message from the detection and update the MOT tracker
          data = detectionsToMatrix(current_detections_);
        }
        tracked_detections_ = oc_tracker_->update(data);
        updateCurrentTarget();
        detections_msg = tracksToMsg(tracked_detections_);
      }
    }

    // always publish new message
    detections_msg.stamp = get_clock()->now() - delay_time;
    detections_pub_->publish(detections_msg);
  }
  Eigen::Matrix<float, Eigen::Dynamic, 6>
  detectionsToMatrix(const std::vector<Detection> &detections) {
    Eigen::Matrix<float, Eigen::Dynamic, 6> data(detections.size(), 6);

    for (size_t i = 0; i < detections.size(); i++) {
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

  common_msgs::msg::Detections
  tracksToMsg(const std::vector<Eigen::RowVectorXf> &tracks) {
    common_msgs::msg::Detections msg;
    // Optionally set current_target_id (e.g., first active track)
    msg.current_target_id = current_target_(4);
    for (const auto &t : tracks) {
      common_msgs::msg::Detection det;
      int x1 = static_cast<int>(t(0));
      int y1 = static_cast<int>(t(1));
      int x2 = static_cast<int>(t(2));
      int y2 = static_cast<int>(t(3));
      int id = t(4);

      det.x = x1;
      det.y = y1;
      det.width = x2 - x1;
      det.height = y2 - y1;
      det.id = id;
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
    cv::Mat detection_frame = current_frame_->image;
    rclcpp::Time detection_stamp(current_frame_->header.stamp);
    // Run detection
    std::vector<Detection> detections =
        detector_->detect(detection_frame, 0.35);
    std::lock_guard<std::mutex> lock(det_mutex);
    detection_frame_ = detection_frame;

    // assign member variables
    current_detections_ = detections;
    current_detections_stamp_ = get_clock()->now() - delay_time;
    // current_detections_stamp_ = detection_stamp;
    new_dets_available_ = true;
  }

  void targetChangeService(
      const std::shared_ptr<common_msgs::srv::ChangeTarget::Request> request,
      std::shared_ptr<common_msgs::srv::ChangeTarget::Response> response) {
    // Find new target based on direction
    if (tracked_detections_.empty() || current_target_(4) == -1) {
      response->success = false;
      response->message = "No valid current target or empty detections.";
      return;
    }
    updateKDTree();
    // find closest 5 targets
    int num_results = 5;
    std::vector<uint32_t> ret_index(num_results);
    std::vector<float> out_dist_sqr(num_results);
    searchClosestTargets(current_target_, num_results, ret_index, out_dist_sqr);

    if (ret_index.empty()) {
      // failed to update (unexpected state)
      response->success = false;
      response->message = "Unexpected result when trying to change target. No "
                          "near targets found.";
      return;
    }
    // check if the requested direction is empty
    Eigen::Vector2f dir(request->horizontal, request->vertical);
    if (dir.isZero()) {
      response->success = false;
      response->message = "Direction vector is zero.";
      return;
    }
    // check if we need to move horizontal or vertical
    int dir_idx = -1;
    if (dir(0) != 0) {
      dir_idx = 0;
    } else if (dir(1) != 0) {
      dir_idx = 1;
    }
    // determine the direction of the request
    int sign = 1;
    if (dir(dir_idx) < 0) {
      sign = -1;
    }
    if (dir_idx == 1) {
      sign = -sign;
    }

    float diff;
    float least_dist = 100000.0f;
    int best_idx = -1;

    for (auto idx : ret_index) {
      const auto &det = tracked_detections_[idx];
      // filter detections that are in the opposite way than the requested
      diff = sign * (det(dir_idx) - current_target_(dir_idx));
      if (diff < 0 || diff == 0) {
        continue;
      }
      // closer detection
      if (diff < least_dist) {
        least_dist = diff;
        best_idx = idx;
      }
    }

    if (best_idx == -1) {
      response->success = false;
      response->message = "No target found in requested direction.";
      return;
    }
    current_target_ = tracked_detections_[best_idx];
    response->success = true;
    response->message = "Target changed successfully.";
  }

  void updateKDTree() {
    curr_det_point_cloud_.pts = tracked_detections_;
    detection_tree_->buildIndex();
  }
  bool initCurrentTarget() {
    if (tracked_detections_.empty() || center_x == -1 || center_y == -1) {
      return false;
    }
    // search for a target near the center
    Eigen::RowVector<float, 6> dummy_target(center_x, center_y, 0, 0, 0, 0);
    int num_results = 1;
    std::vector<uint32_t> ret_index(num_results);
    std::vector<float> out_dist_sqr(num_results);
    searchClosestTargets(dummy_target, num_results, ret_index, out_dist_sqr);
    // If nothing found then there's an error in logic
    if (ret_index.size() == 0) {
      RCLCPP_ERROR(get_logger(),
                   "Tried to reset current target but no results were found.");
      return false;
    }

    current_target_ = tracked_detections_[ret_index[0]];
    return true;
  }
  void updateCurrentTarget() {
    // If no detections then no target
    if (tracked_detections_.empty()) {
      current_target_(4) = -1;
      return;
    }
    if (current_target_.size() > 4 && current_target_(4) == -1) {
      // There's no current target
      // fall into default logic for finding a target
      updateKDTree();
      initCurrentTarget();
      return;
    }
    // Current Target is in tracked_detections_
    for (const auto &target : tracked_detections_) {
      if (target(4) == current_target_(4)) {
        current_target_ = target;
        return;
      }
    }

    // Current target is missing
    // Find closest target to last known "current_target_"
    updateKDTree();
    int num_results = 1;
    std::vector<uint32_t> ret_index(num_results);
    std::vector<float> out_dist_sqr(num_results);
    searchClosestTargets(current_target_, num_results, ret_index, out_dist_sqr);

    if (ret_index.size() == 0) {
      // failed to update (unexpected state)
      //
      RCLCPP_ERROR(get_logger(), "Unexpected result when trying to update "
                                 "current target. No near targets found.");
      current_target_(4) = -1;
      return;
    }

    current_target_ = tracked_detections_[ret_index[0]];
  }
  void searchClosestTargets(const Eigen::RowVectorXf &query_det,
                            int &num_results, std::vector<uint32_t> &ret_index,
                            std::vector<float> &out_dist_sqr) {
    float query_pt[2] = {query_det(0) + (query_det(2) - query_det(0)) / 2.0f,
                         query_det(1) + (query_det(3) - query_det(1)) / 2.0f};
    num_results = detection_tree_->knnSearch(&query_pt[0], num_results,
                                             &ret_index[0], &out_dist_sqr[0]);
    // In case of less points in the tree than requested:
    ret_index.resize(num_results);
    out_dist_sqr.resize(num_results);
  }

  // int findTarget(int8_t vertical, int8_t horizontal) {
  //     if (kd_tree_->empty()) {
  //         return -1;
  //     }

  //     // Get current target position
  //     std::vector<float> current_pos;
  //     if (current_target_id_ >= 0 && current_target_id_ <
  //     static_cast<int>(kd_tree_->size())) {
  //         current_pos = kd_tree_->getPoint(current_target_id_);
  //     } else {
  //         // Default to center if no current target
  //         current_pos = {static_cast<float>(current_frame_->image.cols / 2),
  //         static_cast<float>(current_frame_->image.rows / 2)};
  //     }

  //     // Calculate direction vector
  //     float dx = horizontal * 50.0f;  // Adjust step size as needed
  //     float dy = vertical * 50.0f;
  //     std::vector<float> target_pos = {current_pos[0] + dx, current_pos[1] +
  //     dy};

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
  //                 index_ = std::make_unique<KDTreeIndex>(2, *this,
  //                 nanoflann::KDTreeSingleIndexAdaptorParams(10));
  //                 index_->buildIndex();
  //             }
  //         }

  //         void knnSearch(const float* query, size_t num_results, size_t*
  //         indices, float* dists_sq) {
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

private:
  // struct KDTreeIndex : public
  // KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, KDTree>, KDTree, 2> {
  //     KDTreeIndex(size_t n_dims, KDTree& tree, const
  //     KDTreeSingleIndexAdaptorParams& params)
  //         : KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, KDTree>,
  //         KDTree, 2>(n_dims, tree, params) {}
  // };

  // std::vector<Point> points_;
  // std::vector<size_t> ids_;
  DetPointCloud curr_det_point_cloud_;
  std::unique_ptr<my_kd_tree_t> detection_tree_;
  Eigen::RowVectorXf current_target_;
  float center_x = -1;
  float center_y = -1;
  // inline size_t kdtree_get_point_count() const { return points_.size(); }
  // inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
  //     return dim == 0 ? points_[idx].x : points_[idx].y;
  // }
  // template <class BBOX>
  // bool kdtree_get_bbox(BBOX&) const { return false; }
  // };

  // configs
  double detection_fps_;
  double tracking_fps_;
  rclcpp::Duration delay_time{0, 0};
  // detection params
  std::unique_ptr<YOLODetector> detector_;
  cv::Mat detection_frame_;
  rclcpp::Time current_detections_stamp_;
  bool new_dets_available_ = false;
  std::vector<Eigen::RowVectorXf> tracked_detections_;
  std::vector<Detection> current_detections_;
  common_msgs::msg::Detections detections_msg;
  // current state
  shm_msgs::CvImageConstPtr current_frame_;
  rclcpp::Time current_stamp_;

  // Tracker
  std::shared_ptr<ocsort::OCSort> oc_tracker_;

  // ROS2 interfaces
  rclcpp::Subscription<shm_msgs::msg::Image1m>::SharedPtr frame_sub_;
  rclcpp::Publisher<common_msgs::msg::Detections>::SharedPtr detections_pub_;
  rclcpp::Service<common_msgs::srv::ChangeTarget>::SharedPtr target_change_srv_;
  rclcpp::TimerBase::SharedPtr detection_timer_;

  // Synchronization
  rclcpp::CallbackGroup::SharedPtr detection_timer_group_;
  rclcpp::CallbackGroup::SharedPtr frame_sub_group_;
  std::mutex det_mutex;

  // std::unique_ptr<KDTree> kd_tree_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DetectionAndMOTNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
