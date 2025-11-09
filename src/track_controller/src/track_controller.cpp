#include "track_controller/track_controller.hpp"

TrackController::TrackController()
: Node("track_controller"),
  curr_mode_("offboard"),
  tracking_lost_(false)
{
  // Declare parameters
  this->declare_parameter<double>("max_alt", 10.0);
  this->declare_parameter<double>("max_dist", 5.0);
  this->declare_parameter<std::string>("required_mode", "guided");
  this->declare_parameter<double>("vis_tracking_timeout", 1.0);
  this->declare_parameter<double>("required_alt", 5.0);
  this->declare_parameter<double>("required_dist", 2.0);

  // Get parameters
  this->get_parameter("max_alt", max_alt_);
  this->get_parameter("max_dist", max_dist_);
  this->get_parameter("required_mode", required_mode_);
  this->get_parameter("vis_tracking_timeout", vis_tracking_timeout_);
  this->get_parameter("required_alt", required_alt_default_);
  this->get_parameter("required_dist", required_dist_default_);

  // Create action server
  action_server_ = rclcpp_action::create_server<Track>(
    this,
    "track",
    std::bind(&TrackController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TrackController::handle_cancel, this, std::placeholders::_1),
    std::bind(&TrackController::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse TrackController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Track::Goal> goal)
{
  (void)uuid;
  // Validate goal
  if (goal->required_alt <= 0.0  ||
      goal->required_dist <= 0.0)
  {
    RCLCPP_WARN(this->get_logger(), "Invalid goal: alt=%.2f, dist=%.2f", goal->required_alt, goal->required_dist);
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Accepting goal: alt=%.2f, dist=%.2f", goal->required_alt, goal->required_dist);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrackController::handle_cancel(
  const std::shared_ptr<GoalHandleTrack> goal_handle)
{
  (void) goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrackController::handle_accepted(const std::shared_ptr<GoalHandleTrack> goal_handle)
{
  // Execute the goal
  std::thread{std::bind(&TrackController::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void TrackController::execute(const std::shared_ptr<GoalHandleTrack> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Track::Result>();

  // Use defaults if goal values are zero
  double required_alt = (goal->required_alt > 0.0) ?  goal->required_alt : required_alt_default_;
  double required_dist = (goal->required_dist > 0.0) ? goal->required_dist : required_dist_default_;

  required_alt = std::min(required_alt,max_alt_); 
  required_dist = std::min(required_dist,max_dist_); 
  RCLCPP_INFO(this->get_logger(), "Executing goal with alt=%.2f, dist=%.2f", required_alt, required_dist);

  rclcpp::Rate loop_rate(20.0);  // 20 Hz
  std::string msg;
  while (rclcpp::ok() && !goal_handle->is_canceling())
  {
    if (!isTrackingValid(msg))
    {
      result->result = msg;
      goal_handle->abort(result);
      RCLCPP_INFO(this->get_logger(), "Tracking lost, aborting");
      return;
    }

    // Publish feedback
    auto feedback = std::make_shared<Track::Feedback>();
    feedback->current_alt = required_alt;  // Placeholder, should be actual current alt
    feedback->current_dist = required_dist;  // Placeholder, should be actual current dist
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  if (goal_handle->is_canceling())
  {
    result->result = "canceled";
    goal_handle->canceled(result);
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
  }
  else
  {
    result->result = "aborted";
    goal_handle->abort(result);
    RCLCPP_INFO(this->get_logger(), "Goal aborted");
  }
}

bool TrackController::isTrackingValid(std::string& msg)
{
  // Check if mode is correct
  if (curr_mode_ != required_mode_)
  {
    msg = "Wrong Mode";
    std::cout << "got mode " << curr_mode_ << " required " << required_mode_ << std::endl;
    return false;
  }

  // Check timeout
  auto now = this->get_clock()->now();
  vis_tracking_timestamp_ = this->get_clock()->now();
  if ((now - vis_tracking_timestamp_).seconds() > vis_tracking_timeout_)
  {
    msg = "Tracking Lost";
    return false;
  }

  return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
