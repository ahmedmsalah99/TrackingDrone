#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
// #include <cv_bridge/cv_bridge.h>
#include "shm_msgs/msg/image.hpp"
#include <opencv2/tracking.hpp>
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
        declare_parameter<std::string>("model_path", shared_path + "/resources/best3_nano.onnx");
        declare_parameter<std::string>("labels_path", shared_path + "/resources/coco_mod.names");
        declare_parameter<bool>("use_gpu", false);
        declare_parameter<int>("cache_size", 30);
        declare_parameter<double>("detection_fps", 10.0);
        declare_parameter<double>("tracking_fps", 70.0);

        // Get parameters
        std::string model_path = get_parameter("model_path").as_string();
        std::string labels_path = get_parameter("labels_path").as_string();
        bool use_gpu = get_parameter("use_gpu").as_bool();
        cache_size_ = get_parameter("cache_size").as_int();
        detection_fps_ = get_parameter("detection_fps").as_double();
        tracking_fps_ = get_parameter("tracking_fps").as_double();

        delay_time = rclcpp::Duration::from_seconds(0.25);
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

        // params.resize = true;
        // params.desc_npca = cv::TrackerKCF::GRAY;  // fallback non-compressed
        // params.desc_pca  = cv::TrackerKCF::CN;    // 10 channels
        // params.compress_feature = true;
        // params.compressed_size = 2;               // ≤ 10
        params.output_sigma_factor = 1.5;
        // params.detect_thresh = 0.2;


        // params.desc_npca = cv::TrackerKCF::GRAY;  // fallback non-compressed
        // params.desc_pca  = cv::TrackerKCF::CN;    // 10 channels
        // params.compress_feature = false;
        // params.compressed_size = 1;               // ≤ 10
        // trackers_.resize(50);
        // for(int i=0;i<50;i++){
        //     trackers_[i] = cv::TrackerKCF::create(params);
        // }
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
        // std::lock_guard<std::mutex> lock(tracking_mutex);
        // trackFrame();
        bool new_det = false;
        if(current_detections_stamp_.nanoseconds() != 0 && !detection_frame_.empty()){
            rclcpp::Time time_now = get_clock()->now() - delay_time;
            if(time_now > current_detections_stamp_ && (time_now - current_detections_stamp_) < delay_time){
                // new detection available - update our tracked detection
                new_det = true;
                std::lock_guard<std::mutex> lock(det_mutex);
                tracked_detections_.clear();
                tracked_detections_.reserve(current_detections_.size());
                std::transform(
                    current_detections_.begin(), current_detections_.end(),
                    std::back_inserter(tracked_detections_),
                    [](const Detection& d){ return FloatDetection(d); });
                std::cout << "tracked_detections_.size() " << tracked_detections_.size() << std::endl;
                current_detections_stamp_ = rclcpp::Time(0);
            }
        }
        if(!tracked_detections_.empty()){
            // std::cout << "publishing tracked frames" << std::endl;
            Eigen::Matrix<float, Eigen::Dynamic, 6> data(tracked_detections_.size(),6);
            if(new_det){
                data = detectionsToMatrix(tracked_detections_);
                std::vector<Eigen::RowVectorXf> res = oc_tracker_->update(data);
                detections_msg = tracksToMsg(res);
            }
            
            // std::cout << "res is " << std::endl;
            // for (const auto& row : res) {
            //     std::cout << row << std::endl;
            // }
            // std::cout << " " << std::endl;
            
            detections_msg.stamp = get_clock()->now() - delay_time;  
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
            // std::cout << "current_frame_->image.empty() " << current_frame_->image.empty() << std::endl;
            return;
        }
        // Store detection frame and points
        cv::Mat detection_frame = current_frame_->image.clone();
        rclcpp::Time detection_stamp(current_frame_->header.stamp);
        // Run detection
        rclcpp::Time now = get_clock()->now() - delay_time;
        auto oldest = frame_cache_->getOldestTime();
        auto latest = frame_cache_->getLatestTime();
        auto start  = detection_stamp + rclcpp::Duration::from_nanoseconds(5);
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

        std::vector<Detection> detections = detector_->detect(detection_frame,0.35);
        std::lock_guard<std::mutex> lock(det_mutex);
        detection_frame_ = detection_frame.clone();
        // Track detections to present
        trackDetectionsToPresent(detections,detection_frame_ ,detection_stamp);
    }
    void trackDetectionsToPresent(std::vector<Detection>& detections,const cv::Mat& detection_frame, rclcpp::Time detection_stamp_){
        if (detections.empty()) {
            RCLCPP_WARN(get_logger(), "Tracking detections to present found no detections ");
            return;
        }
        // RCLCPP_INFO(get_logger(), "Started tracking to present with %ld detections.",detections.size());
        rclcpp::Time now = get_clock()->now() - delay_time;
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
        images.push_back(detection_frame);
        
        // Track through each frame in the interval
        // for (const auto& msg : msgs) {
        //     shm_msgs::CvImageConstPtr cv_img = shm_msgs::toCvShare(msg);
        //     if (cv_img->image.empty()) continue;
        //     images.push_back(cv_img->image);
        // }
        std::lock_guard<std::mutex> lock(tracking_mutex);
        // Set tracked detections
        trackFrames(images,detections,prev_frame_gray_,prev_points_);
        current_detections_ = detections;
        current_detections_stamp_ = now;
        // RCLCPP_INFO(get_logger(), "Tracking to present on %ld images found %ld detections.",images.size(),tracked_detections_.size());
        // saveTrackedDetections(images, detections, prev_points_, "/home/stark/stuff/Projects/TrackingDrone/ros2_ws/results");
    }
    cv::Point2f meanPoint(const std::vector<cv::Point2f>& pts) { 
        if (pts.empty()) return cv::Point2f(0.f, 0.f);
            cv::Point2f sum(0.f, 0.f); 
            for (const auto &p : pts) 
            { 
            sum.x += p.x; sum.y += p.y; 
            } 
            float inv = 1.0f / static_cast<float>(pts.size());
            return cv::Point2f(sum.x * inv, sum.y * inv); 
    }
    std::vector<std::vector<cv::Point2f>> detsToPoints(const std::vector<Detection>& detections, const cv::Mat & frame) const
    { 
        cv::Mat first_gray;
        std::vector<std::vector<cv::Point2f>> points;
        cv::cvtColor(frame, first_gray, cv::COLOR_BGR2GRAY);
        float margin = 2.0f;
        for (const auto& det : detections) 
        { // cv::Rect roi(det.box); 
            cv::Rect roi(det.box.x + margin, det.box.y + margin, det.box.width - margin, det.box.height - margin); 
            cv::Mat roi_img = first_gray(roi); 
            std::vector<cv::Point2f> roi_points; 
            cv::goodFeaturesToTrack(roi_img, roi_points, 10, 0.01, 5);
            // KLT features 
            // Shift ROI points to full image coordinates 
            if (roi_points.empty()){
                roi_points.push_back(cv::Point2f(det.box.x, det.box.y));
            }else{
                for (auto& p : roi_points) {
                    p.x += roi.x; 
                    p.y += roi.y; 
                } 
            }
            // std::cout << "roi_points size " << roi_points.size() << std::endl;
            // prev_points.insert(prev_points.end(), roi_points.begin(), roi_points.end());
            points.push_back(roi_points); 
        } 
        return points; 
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
    const cv::Mat& prev_img,
    const cv::Mat& curr_img,
    const std::vector<FloatDetection>& prev_dets,
    const std::vector<FloatDetection>& curr_dets,
    const std::string& out_dir,
    const std::string& counter
) {
    auto drawDetections = [&](const cv::Mat& img,
                              const std::vector<FloatDetection>& detections) {
        cv::Mat final_img = img.clone();
        for (size_t i = 0; i < detections.size(); ++i) {
            const auto& det = detections[i];
            cv::Rect rect(
                static_cast<int>(det.box.x),
                static_cast<int>(det.box.y),
                static_cast<int>(det.box.width),
                static_cast<int>(det.box.height)
            );
            cv::rectangle(final_img, rect, cv::Scalar(0, 255, 0), 2);

            // if (i < pts.size()) {
            //     for (const auto& pt : pts[i]) {
            //         cv::circle(final_img, pt, 1, cv::Scalar(0, 0, 255), -1);
            //     }
            // }

            std::ostringstream label;
            label << "ID:" << det.classId << " "
                  << std::fixed << std::setprecision(2) << det.conf;
            cv::putText(final_img, label.str(),
                        cv::Point(rect.x, rect.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 0), 1);
        }
        return final_img;
    };



    // Build filenames
    std::string base_name = out_dir + "/" + counter;
    std::string prev_file = base_name + "_prev.png";
    std::string curr_file = base_name + "_curr.png";
    std::string txt_file  = base_name + "_detections.txt";
    cv::Mat prev_out = drawDetections(prev_img, prev_dets);
    cv::Mat curr_out = drawDetections(curr_img, curr_dets);
    // Save images
    cv::imwrite(prev_file, prev_out);
    cv::imwrite(curr_file, curr_out);

    // Save detections to text file
    std::ofstream ofs(txt_file);
    if (ofs.is_open()) {
        ofs << "Prev detections:\n";
        for (const auto& det : prev_dets) {
            ofs << det.classId << " "
                << det.conf << " "
                << det.box.x << " " << det.box.y << " "
                << det.box.width << " " << det.box.height << "\n";
        }

        ofs << "\nCurr detections:\n";
        for (const auto& det : curr_dets) {
            ofs << det.classId << " "
                << det.conf << " "
                << det.box.x << " " << det.box.y << " "
                << det.box.width << " " << det.box.height << "\n";
        }
    }
}
// void saveTrackedDetections(
//     const cv::Mat& img,
//     const std::vector<FloatDetection>& detections,
//     std::vector<std::vector<cv::Point2f>> points,
//     const std::string& out_dir
// ) {

//     cv::Mat final_img = img.clone();
//     for (const auto& det : detections) {
//         // Convert your FloatBoundingBox into cv::Rect
//         cv::Rect rect(
//             static_cast<int>(det.box.x),
//             static_cast<int>(det.box.y),
//             static_cast<int>(det.box.width),
//             static_cast<int>(det.box.height)
//         );

//         // Draw bounding box
//         cv::rectangle(final_img, rect, cv::Scalar(0, 255, 0), 2);
//         for (const auto& pts : points){
//             for (const auto& pt : pts) {
//                 cv::circle(final_img, pt, 1, cv::Scalar(0, 0, 255), -1); // red dots
//             }
//         }
//         // Optional: put text (classId and confidence)
//         std::ostringstream label;
//         label << "ID:" << det.classId << " " << std::fixed << std::setprecision(2) << det.conf;
//         cv::putText(final_img, label.str(),
//                     cv::Point(rect.x, rect.y - 5),
//                     cv::FONT_HERSHEY_SIMPLEX, 0.5,
//                     cv::Scalar(0, 255, 0), 1);
//     }

//     // Build filename with timestamp
//     std::string filename = out_dir + "/tracked_" + getTimestamp() + ".png";
//     cv::imwrite(filename, final_img);
// }
    void trackFrames(const std::vector<cv::Mat>& images,std::vector<Detection>& detections,cv::Mat& final_grey,std::vector<std::vector<cv::Point2f>>& final_points){
        if(images.size()<2 || detections.size() ==0 ){
            // final_points = detsToPoints(detections,images[0]);
            cv::cvtColor(images[0], final_grey, cv::COLOR_BGR2GRAY);
            // RCLCPP_INFO(get_logger(),"No sufficient data for present tracking. There are %ld images and %ld detections.",
            // images.size(),detections.size());
            return;
        }
        // RCLCPP_INFO(get_logger(),"Sufficient data for present tracking. There are %ld images and %ld detections.",
            // images.size(),detections.size());
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<std::vector<cv::Point2f>> prev_points = detsToPoints(detections,images[0]);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        std::cout << "dets to points Time taken: " << elapsed.count() << " ms" << std::endl;

        cv::Mat prev_gray;
        final_points.resize(prev_points.size());
        start = std::chrono::high_resolution_clock::now();
        // initialize delta_x and delta_y vectors
        
        std::vector<float> delta_x(prev_points.size(),0);
        std::vector<float> delta_y(prev_points.size(),0);

        cv::cvtColor(images[0], final_grey, cv::COLOR_BGR2GRAY);
        for(size_t i=1;i<images.size();i++){
            // convert the images to grey
            prev_gray = final_grey;
            cv::cvtColor(images[i], final_grey, cv::COLOR_BGR2GRAY);
            
            for (int j = prev_points.size()-1;j>-1;j--)
            { 
                // for each detection in the image

                // calculate the optical flow
                std::vector<uchar> status;
                std::vector<float> err;
                cv::calcOpticalFlowPyrLK(prev_gray, final_grey, prev_points[j], final_points[j], status, err);
                // Filter invalid tracking points
                for(int k=final_points[j].size()-1;k>-1;k--){
                    if(!status[k]){
                        final_points[j].erase(final_points[j].begin()+k);
                        prev_points[j].erase(prev_points[j].begin()+k);
                    }
                }
                // Get delta movement and further filter points that don't align with the others
                cv::Point2f delta;
                filterAndComputeMedianDelta(prev_points[j], final_points[j],delta);
                // If no valid points left, remove the detection
                if(final_points[j].empty()){
                    detections.erase(detections.begin()+j);
                    delta_x.erase(delta_x.begin()+j);
                    delta_y.erase(delta_y.begin()+j);
                    final_points.erase(final_points.begin()+j);
                    prev_points.erase(prev_points.begin()+j);
                    continue;
                }
                // Update the deltas
                delta_x[j] += delta.x;
                delta_y[j] += delta.y;
            }
            // Update the detections with the new deltas
            for(int j = detections.size()-1;j>-1;j--){
                detections[j].box.x += static_cast<int>(std::round(delta_x[j]));
                detections[j].box.y += static_cast<int>(std::round(delta_y[j]));
            }

            // saveTrackedDetections(images[i], detections, final_points, "/home/stark/stuff/Projects/TrackingDrone/ros2_ws/results");
            prev_points = final_points;
        }
        end = std::chrono::high_resolution_clock::now();
        elapsed = end - start;
        std::cout << "Time taken: " << elapsed.count() << " ms" << std::endl;
    }
    
    void trackFrame() {
        auto start = std::chrono::high_resolution_clock::now();
        bool new_det_received = false;
        if(current_detections_stamp_.nanoseconds() != 0 && !detection_frame_.empty()){
            rclcpp::Time time_now = get_clock()->now() - delay_time;
            if(time_now > current_detections_stamp_ && (time_now - current_detections_stamp_) < delay_time){
                // new detection available - update our tracked detection
                std::lock_guard<std::mutex> lock(det_mutex);
                new_det_received = true;
                tracked_detections_.clear();
                tracked_detections_.reserve(current_detections_.size());
                std::transform(
                    current_detections_.begin(), current_detections_.end(),
                    std::back_inserter(tracked_detections_),
                    [](const Detection& d){ return FloatDetection(d); });
                std::cout << "tracked_detections_.size() " << tracked_detections_.size() << std::endl;
                current_detections_stamp_ = rclcpp::Time(0);
            }
        }
        if (current_frame_->image.empty() || tracked_detections_.empty()) {
            std::cout << "current_frame_->image.empty() " << current_frame_->image.empty() << std::endl;
            std::cout << "trackers_.empty() " << trackers_.empty() << std::endl;
            std::cout << "tracked_detections_.empty() " << tracked_detections_.empty() << std::endl;

            std::cout << "can't track frames: empty image or no trackers/detections" << std::endl;
            return;
        }
        
        // Convert current frame to gray
        rclcpp::Time now = get_clock()->now() - delay_time;
        shm_msgs::msg::Image1m::ConstSharedPtr msg = 
                                        frame_cache_->getElemAfterTime(now - delay_time);
        if(!msg){
            std::cout << "No delayed frame yet" << std::endl;
            return;
        }
        shm_msgs::CvImageConstPtr cv_img = shm_msgs::toCvShare(msg);
        builtin_interfaces::msg::Time tracking_frame_stamp = cv_img->header.stamp;
        if(new_det_received){
            
            // int trackers_needed = tracked_detections_.size() - trackers_.size();
            int trackers_needed = tracked_detections_.size();
            trackers_.clear();
            trackers_.resize(tracked_detections_.size());
            if (trackers_needed > 0){
                for(int i = 0; i<trackers_needed;i++){
                    // trackers_.push_back(cv::TrackerKCF::create(params));
                    trackers_[i] = cv::TrackerKCF::create(params);
                }
            }
            for (int i = static_cast<int>(tracked_detections_.size()) - 1; i >= 0; --i) {
                const auto det = tracked_detections_[i];
                cv::Rect bbox(static_cast<int>(det.box.x), static_cast<int>(det.box.y), static_cast<int>(det.box.width), static_cast<int>(det.box.height));
                trackers_[i]->init(detection_frame_, bbox);
                // std::cout << "detection_frame_.empty() " << detection_frame_.empty() << std::endl;
                // std::cout << "bbox " << bbox << std::endl;
                // std::cout << "Frame size: " 
                // << detection_frame_.cols << " x " << detection_frame_.rows
                // << " | Channels: " << detection_frame_.channels()
                // << " | Type: " << detection_frame_.type()
                // << std::endl;
                // trackers_[i]->update(detection_frame_, bbox);
            }
            prev_frame_gray_ = detection_frame_;
        }

        if(trackers_.size() == 0){
            return;
        }
        const std::vector<FloatDetection> prev_dets = tracked_detections_;
        for (int i = static_cast<int>(tracked_detections_.size()) - 1; i >= 0; --i) {
            cv::Rect bbox;
            bool success = trackers_[i]->update(cv_img->image, bbox);
            if (success) {
                tracked_detections_[i].box.x = static_cast<float>(bbox.x);
                tracked_detections_[i].box.y = static_cast<float>(bbox.y);
                tracked_detections_[i].box.width = static_cast<float>(bbox.width);
                tracked_detections_[i].box.height = static_cast<float>(bbox.height);
            } else {
                // Remove failed tracker and detection
                trackers_.erase(trackers_.begin() + i);
                tracked_detections_.erase(tracked_detections_.begin() + i);
            }
        }
        
        saveTrackedDetections(prev_frame_gray_,cv_img->image,prev_dets, tracked_detections_, "/home/stark/stuff/Projects/TrackingDrone/ros2_ws/results",std::to_string(counter));
        counter ++;
        prev_frame_gray_ = cv_img->image.clone();
        // // Update for next iteration
        // prev_frame_gray_ = curr_gray.clone();
        // prev_points_ = curr_points;

        // Publish updated detections
        // publishTrackings(tracking_frame_stamp);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli>  elapsed = end - start;
        std::cout << "Time taken: " << elapsed.count() << " ms" << std::endl;
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
    // Compute median delta between two sets of matched points
        void filterAndComputeMedianDelta(
        const std::vector<cv::Point2f>& prev_pts,
        std::vector<cv::Point2f>& curr_pts,
        cv::Point2f& median_delta,
        float threshold = 5.0f // pixels
    ) {
        if (prev_pts.empty() || curr_pts.empty() || prev_pts.size() != curr_pts.size()) {
            median_delta = cv::Point2f(0.f, 0.f);
            return;
        }

        std::vector<float> dxs, dys;
        dxs.reserve(prev_pts.size());
        dys.reserve(prev_pts.size());

        for (size_t i = 0; i < prev_pts.size(); ++i) {
            dxs.push_back(curr_pts[i].x - prev_pts[i].x);
            dys.push_back(curr_pts[i].y - prev_pts[i].y);
        }

        auto median = [](std::vector<float>& v) {
            size_t n = v.size();
            std::nth_element(v.begin(), v.begin() + n/2, v.end());
            return v[n/2];
        };

        float mdx = median(dxs);
        float mdy = median(dys);
        median_delta = cv::Point2f(mdx, mdy);

        // --- erase outliers in-place ---
        for (int i = (int)prev_pts.size() - 1; i >= 0; --i) {
            float dx = curr_pts[i].x - prev_pts[i].x;
            float dy = curr_pts[i].y - prev_pts[i].y;

            if (std::abs(dx - mdx) > threshold || std::abs(dy - mdy) > threshold) {
                curr_pts.erase(curr_pts.begin() + i);
            }
        }
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
    int counter = 0;
    cv::Mat detection_frame_;
    std::unique_ptr<YOLODetector> detector_;
    // std::unique_ptr<KDTree> kd_tree_;
    shm_msgs::CvImageConstPtr current_frame_;
    int current_target_id_ = -1;
    size_t cache_size_;
    double detection_fps_;
    double tracking_fps_;
    rclcpp::Duration delay_time{0,0};
    // Lucas-Kanade tracking members
    cv::Mat prev_frame_gray_;
    std::vector<std::vector<cv::Point2f>> prev_points_;
    std::vector<FloatDetection> tracked_detections_;
    std::vector<Detection> current_detections_;
    rclcpp::Time current_detections_stamp_;
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
    std::mutex det_mutex;

    std::vector<cv::Ptr<cv::Tracker>> trackers_;
    cv::TrackerKCF::Params params;
    common_msgs::msg::Detections detections_msg;
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
