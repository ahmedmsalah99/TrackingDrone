#ifndef TRACK_CONTROLLER_HPP_
#define TRACK_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <common_msgs/action/track.hpp>

class TrackController : public rclcpp::Node
{
public:
  using Track = common_msgs::action::Track;
  using GoalHandleTrack = rclcpp_action::ServerGoalHandle<Track>;

  TrackController();

private:
  // Action server
  rclcpp_action::Server<Track>::SharedPtr action_server_;

  // Parameters
  double max_alt_;
  double max_dist_;
  std::string required_mode_;
  double vis_tracking_timeout_;
  double required_alt_default_;
  double required_dist_default_;

  // State variables
  std::string curr_mode_;
  bool tracking_lost_;
  rclcpp::Time vis_tracking_timestamp_;

  // Callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Track::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTrack> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleTrack> goal_handle);

  void execute(const std::shared_ptr<GoalHandleTrack> goal_handle);

  bool isTrackingValid(std::string &msg);
};

#endif  // TRACK_CONTROLLER_HPP_
