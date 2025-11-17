#ifndef TRACK_CONTROLLER_HPP_
#define TRACK_CONTROLLER_HPP_

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>    // for close()
#include <cstring>     // for memset, memcpy, etc.
#include <common/mavlink.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <common_msgs/action/track.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ardupilot_msgs/msg/status.hpp>
#include <chrono>
#include <tuple>
#include <Eigen/Dense>

#define UDP_PORT_OUT 14552    // usually ArduPilot SITL listens on this
#define UDP_PORT_IN  14551    // our local receiving port
#define TARGET_IP    "127.0.0.1"


class TrackController : public rclcpp::Node
{
public:
  using Track = common_msgs::action::Track;
  using GoalHandleTrack = rclcpp_action::ServerGoalHandle<Track>;

  TrackController();

private:
  // Action server
  rclcpp_action::Server<Track>::SharedPtr action_server_;
  std::chrono::steady_clock::time_point start_time;
  // Parameters
  double max_alt_;
  double max_dist_;
  int required_mode_ = 4;
  double vis_tracking_timeout_;
  double required_alt_default_;
  double required_dist_default_;
  double required_speed_;
  double gz_ground_level_ = 42;
  // State variables
  int curr_mode_;
  bool tracking_lost_;
  rclcpp::Time vis_tracking_timestamp_;

  // Pose variables
  geometry_msgs::msg::PoseStamped car_pose_gt_;
  geometry_msgs::msg::Pose iris_pose_gt_;



  
  Eigen::Matrix3d b_R_bi_;  // rotation from body to iris frame
  Eigen::Matrix3d bi_R_w_;   // rotation from iris frame to world frame
  // local NED w/F
  Eigen::Vector3d w_O_gw_;  // position of the goal with respect to world
  Eigen::Vector3d w_O_bw_;  // position of the goal with respect to world
  Eigen::Vector3d w_O_fw_;  // origin of the EKF with respect to world
  Eigen::Matrix3d w_R_f_;   // rotation from EKF to world frame
  // Eigen variables
  Eigen::Vector3d f_O_bf_;  // position of the drone with respect to the EKF origin
  Eigen::Matrix3d b_R_f_;   // rotation from the EKF to the body frame
  Eigen::Matrix3d b_R_c_;   // rotation from camera frame to body frame
  Eigen::Vector3d b_O_cb_;  // position of the camera with respect to BODY FRAME

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr car_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr iris_pose_sub_;
  rclcpp::Subscription<ardupilot_msgs::msg::Status>::SharedPtr status_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr velocity_timer_;

  // Callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Track::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTrack> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleTrack> goal_handle);

  void execute(const std::shared_ptr<GoalHandleTrack> goal_handle);

  bool isTrackingValid(std::string &msg);

  // Subscriber callbacks
  void carPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void irisPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void statusCallback(const ardupilot_msgs::msg::Status::SharedPtr msg);
  void initTransformations();
  // Velocity calculation
  std::tuple<double, double, double> calculateVelocity();
  double getYawError();
  Eigen::Vector3d getGoal(double required_alt, double required_dist);


  // Mavlink communication
  uint8_t target_system = 1;
  uint8_t target_component = 1;
  sockaddr_in remote_addr{};
  int sock;
  int configureSocket();
  
  void sendSetPointCmd(Eigen::Vector3d goal, double goal_yaw);
  // void sendYawRateCmd(double yaw_rate);
  // bool yaw_mode_active = false;
  // double yaw_error_low = 5.0;
  // double yaw_error_high = 10.0;
};

#endif  // TRACK_CONTROLLER_HPP_
