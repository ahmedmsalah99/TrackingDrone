#include "track_controller/track_controller.hpp"

TrackController::TrackController()
: Node("track_controller"),
  curr_mode_(4),
  tracking_lost_(false)
{
  start_time = std::chrono::steady_clock::now();
  // Declare parameters
  this->declare_parameter<double>("max_alt", 50.0);
  this->declare_parameter<double>("max_dist", 100.0);
  this->declare_parameter<double>("vis_tracking_timeout", 1.0);
  this->declare_parameter<double>("required_alt", 30.0);
  this->declare_parameter<double>("required_dist", 30.0);
  this->declare_parameter<double>("required_speed", 5.0);

  // Get parameters
  this->get_parameter("max_alt", max_alt_);
  this->get_parameter("max_dist", max_dist_);
  this->get_parameter("vis_tracking_timeout", vis_tracking_timeout_);
  this->get_parameter("required_alt", required_alt_default_);
  this->get_parameter("required_dist", required_dist_default_);
  this->get_parameter("required_speed", required_speed_);
  initTransformations();
  configureSocket();
  // Create action server
  action_server_ = rclcpp_action::create_server<Track>(
    this,
    "track",
    std::bind(&TrackController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TrackController::handle_cancel, this, std::placeholders::_1),
    std::bind(&TrackController::handle_accepted, this, std::placeholders::_1));

  // Create subscribers
  car_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/uav1/moving_box/pose", 10,
    std::bind(&TrackController::carPoseCallback, this, std::placeholders::_1));

  iris_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/uav1/iris/pose", 10,
    std::bind(&TrackController::irisPoseCallback, this, std::placeholders::_1));

  // Create subscriber for status
  status_sub_ = this->create_subscription<ardupilot_msgs::msg::Status>(
    "/ap/status", 10,
    std::bind(&TrackController::statusCallback, this, std::placeholders::_1));

  // Create publisher for cmd_vel
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/ap/cmd_vel", 10);

  // Create timer for velocity calculation
  velocity_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 10 Hz
    std::bind(&TrackController::calculateVelocity, this));
}



void TrackController::initTransformations(){
  b_R_c_ << 
         0.00,  -0.175,  0.985,
        -1.000, -0.001, -0.000,
         0.002, -0.985, -0.175;
  b_O_cb_ << 0.150,-0.010,-0.125;
  w_R_f_ << 
        0,  1, 0,
        1,  0,  0,
        0, 0,  -1;
  b_R_bi_ << 
        0,  1, 0,
        1,  0,  0,
        0, 0,  -1;
}




rclcpp_action::GoalResponse TrackController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Track::Goal> goal)
{
  (void)uuid;
  // Validate goal
  if (goal->required_alt <= 0.0)
  {
    RCLCPP_WARN(this->get_logger(), "Invalid goal: alt=%.2f", goal->required_alt);
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Accepting goal: alt=%.2f", goal->required_alt);
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
  double required_dist = (goal->required_dist > 0.0) ?  goal->required_dist : required_dist_default_;
  required_dist = std::min(required_dist,max_dist_); 
  required_alt = std::min(required_alt,max_alt_); 
  RCLCPP_INFO(this->get_logger(), "Executing goal with alt=%.2f and dist=%.2f", required_alt, required_dist);

  rclcpp::Rate loop_rate(50.0);  // 50 Hz
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

    // Calculate and publish velocity
    // auto [vx, vy, vz] = calculateVelocity();
    // auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    // twist_msg->header.stamp = this->get_clock()->now();
    // twist_msg->header.frame_id = "map";
    // twist_msg->twist.linear.x = vx;
    // twist_msg->twist.linear.y = vy;
    // twist_msg->twist.linear.z = vz;
    // cmd_vel_pub_->publish(*twist_msg);

    // Publish feedback
    // auto feedback = std::make_shared<Track::Feedback>();
    // feedback->current_alt = required_alt;  // Placeholder, should be actual current alt
    // feedback->current_dist = required_dist;  // Placeholder, should be actual current dist
    // goal_handle->publish_feedback(feedback);
    Eigen::Vector3d goal = getGoal(required_alt, required_dist);
    double goal_yaw = getYawError();
    std::cout << "goal is " << goal << "goal_yaw is " << goal_yaw << std::endl;
    Eigen::Vector3d dummy_goal;
    dummy_goal << 1,1,0;
    sendSetPointCmd(goal,goal_yaw);
    // if(yaw_mode_active)
    // {
    //   sendYawRateCmd(yaw_rate);
    //   if(abs(yaw_rate)*180/M_PI < yaw_error_low){
    //     yaw_mode_active = false;
    //   }
    // }
    // else
    // {
    //   sendSetPointCmd(goal);
    //   if(abs(yaw_rate)*180/M_PI >= yaw_error_high){
    //     yaw_mode_active = true;
    //   }
    // }
    // std::cout << "yaw_mode_active " << yaw_mode_active << " current yaw error " << abs(yaw_rate)*180/M_PI <<std::endl;
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

void TrackController::carPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (msg->header.frame_id == "map")
  {
    w_O_gw_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    car_pose_gt_ = *msg;
  }
}

void TrackController::irisPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (!msg->poses.empty())
  {
    iris_pose_gt_ = msg->poses[0];  // Assuming first pose is the one we want
    // Set bi_R_w_ from orientation quaternion
    Eigen::Quaterniond q(iris_pose_gt_.orientation.w, iris_pose_gt_.orientation.x,
                         iris_pose_gt_.orientation.y, iris_pose_gt_.orientation.z);
    // Initialize w_O_fw_ if not set
    if (w_O_fw_.isZero())
    {
      w_O_fw_ = Eigen::Vector3d(iris_pose_gt_.position.x, iris_pose_gt_.position.y, iris_pose_gt_.position.z);
    }
    w_O_bw_ = Eigen::Vector3d(iris_pose_gt_.position.x, iris_pose_gt_.position.y, iris_pose_gt_.position.z);
    bi_R_w_ = q.toRotationMatrix();
    b_R_f_ = bi_R_w_ * w_R_f_;
    // Set f_O_bf_ as position relative to EKF origin
    f_O_bf_ = w_R_f_.transpose()*(Eigen::Vector3d(iris_pose_gt_.position.x, iris_pose_gt_.position.y, iris_pose_gt_.position.z) - w_O_fw_);
  }
}

void TrackController::statusCallback(const ardupilot_msgs::msg::Status::SharedPtr msg)
{
  curr_mode_ = msg->mode;
}


Eigen::Vector3d TrackController::getGoal(double required_alt, double required_dist){
  // double d = required_alt/tan(30 * M_PI / 180.0);
  // Eigen::Vector3d w_O_gb =  w_O_gw_ - w_O_bw_;
  // Eigen::Vector3d w_O_lw = w_O_bw_ + w_O_gb + Eigen::Vector3d{0,0,required_alt} - Eigen::Vector3d{w_O_gb.x(),w_O_gb.y(),0}.normalized()*d;
  // Eigen::Vector3d b_O_lb = b_R_bi_*bi_R_w_*(w_O_lw - w_O_bw_);
  // return b_O_lb;
  Eigen::Vector3d w_O_gb =  w_O_gw_ - w_O_bw_;
  Eigen::Vector3d w_O_gf =  w_O_gw_ - w_O_fw_;
  Eigen::Vector3d w_O_lf = w_O_gf + Eigen::Vector3d{0,0,required_alt} - Eigen::Vector3d{w_O_gb.x(),w_O_gb.y(),0}.normalized()*required_dist;
  Eigen::Vector3d f_O_lf = w_R_f_.transpose() * w_O_lf;
  return f_O_lf;
}

double TrackController::getYawError(){
  Eigen::Vector3d f_O_gb =  w_R_f_.transpose()*(w_O_gw_ - w_O_bw_);
  // 1. Body forward in world frame
  Eigen::Vector3d w_forward;
  w_forward << 1,0,0;

  // 2. Project to 2D
  Eigen::Vector2d f2d(w_forward.x(), w_forward.y());
  Eigen::Vector2d g2d(f_O_gb.x(), f_O_gb.y());

  // 3. Normalize
  f2d.normalize();
  g2d.normalize();

  // 4. Compute angle difference
  double angle_f = std::atan2(f2d.y(), f2d.x());
  double angle_g = std::atan2(g2d.y(), g2d.x());
  double yaw_error_rad = std::atan2(std::sin(angle_g - angle_f), std::cos(angle_g - angle_f));

  return yaw_error_rad;
}


std::tuple<double, double, double> TrackController::calculateVelocity()
{
  // Calculate vector from iris to car
  double dx = car_pose_gt_.pose.position.x - iris_pose_gt_.position.x;
  double dy = car_pose_gt_.pose.position.y - iris_pose_gt_.position.y;
  double dz = car_pose_gt_.pose.position.z - iris_pose_gt_.position.z;

  // Calculate distance
  double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

  // Desired altitude difference (iris should be at required_alt above car)
  double alt_diff = (required_alt_default_ + gz_ground_level_) - iris_pose_gt_.position.z;  // Assuming required_alt is altitude above car

  // Normalize direction
  double norm_x = dx / dist;
  double norm_y = dy / dist;
  double norm_z = alt_diff / std::abs(alt_diff);  // Up or down

  // Calculate velocity
  double vel_x = norm_x * required_speed_;
  double vel_y = norm_y * required_speed_;
  double vel_z = norm_z * required_speed_;

  // Log the velocity
  // RCLCPP_INFO(this->get_logger(), "Calculated velocity: vx=%.2f, vy=%.2f, vz=%.2f, dist=%.2f, alt_diff=%.2f x %.2f y %.2f z %.2f x %.2f y %.2f z %.2f",
  //             vel_x, vel_y, vel_z, dist, alt_diff, iris_pose_gt_.position.x, iris_pose_gt_.position.y,iris_pose_gt_.position.z,
  //             car_pose_gt_.pose.position.x,car_pose_gt_.pose.position.y, car_pose_gt_.pose.position.z);

  return std::make_tuple(vel_x, vel_y, vel_z);
}


int TrackController::configureSocket(){
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket creation failed");
        return -1;
    }
    // 2️⃣ Bind local port (to receive messages)
    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(UDP_PORT_IN);
    if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        perror("bind failed");
        return -1;
    }


    // 3️⃣ Configure remote endpoint (ArduPilot)
    
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(UDP_PORT_OUT);
    inet_pton(AF_INET, TARGET_IP, &remote_addr.sin_addr);

    std::cout << "Waiting for heartbeat from ArduPilot...\n";

    // 4️⃣ Receive HEARTBEAT to get target system/component
    uint8_t buf[2048];
    socklen_t addrlen = sizeof(remote_addr);
    mavlink_message_t msg;
    mavlink_status_t status;
    while (true) {
        ssize_t len = recvfrom(sock, buf, sizeof(buf), 0,
                               (struct sockaddr*)&remote_addr, &addrlen);
        if (len > 0) {
            for (ssize_t i = 0; i < len; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        target_system = msg.sysid;
                        target_component = msg.compid;
                        std::cout << "Got heartbeat from sys=" << (int)target_system
                                  << " comp=" << (int)target_component << "\n";
                        return 0;
                        }
                }
            }
        }
    }
    return 0;
    
  }
void TrackController::sendSetPointCmd(
    Eigen::Vector3d goal, double goal_yaw)
{
    mavlink_message_t msg;
    auto now = std::chrono::steady_clock::now();
    uint32_t time_boot_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
    // Create the SET_POSITION_TARGET_LOCAL_NED message
    mavlink_msg_set_position_target_local_ned_pack(
        255,                          // sender system ID
        190,                        // sender component ID (e.g. companion computer)
        &msg,
        0,                          // time_boot_ms, set to 0 (autopilot fills this in)
        target_system,              // target system
        target_component,            // target component
        MAV_FRAME_LOCAL_NED,            // MAV_FRAME_LOCAL_NED, etc.
        2552,                   // bits for which fields to ignore
        goal.x(), goal.y(), goal.z(),                     // Position
        0.0, 0.0, 0.0,                  // Velocity
        0.0f, 0.0f, 0.0f,            // Acceleration (ignored if masked)
        goal_yaw,                         // yaw (rad)
        0                     // yaw rate (rad/s)
    );

    // Send the message (replace mavlink_channel_t with your actual function)
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    // 7️⃣ Send message to ArduPilot
    ssize_t sent = sendto(sock, buffer, len, 0,
                          (struct sockaddr*)&remote_addr, sizeof(remote_addr));
    if (sent < 0)
        perror("sendto failed");
    else
        std::cout << "Sent SET_POSITION_TARGET_LOCAL_NED (velocity)\n";
}

// void TrackController::sendYawRateCmd(double yaw_rate)
// {
//     mavlink_message_t msg;
//     auto now = std::chrono::steady_clock::now();
//     uint32_t time_boot_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
//     // Create the SET_POSITION_TARGET_LOCAL_NED message
//     mavlink_msg_set_position_target_local_ned_pack(
//         255,                          // sender system ID
//         190,                        // sender component ID (e.g. companion computer)
//         &msg,
//         0,                          // time_boot_ms, set to 0 (autopilot fills this in)
//         target_system,              // target system
//         target_component,            // target component
//         MAV_FRAME_LOCAL_NED,            // MAV_FRAME_LOCAL_NED, etc.
//         1535,                   // bits for which fields to ignore
//         0, 0, 0,                     // Position
//         0.0, 0.0, 0.0,                  // Velocity
//         0.0f, 0.0f, 0.0f,            // Acceleration (ignored if masked)
//         0,                         // yaw (rad)
//         yaw_rate                     // yaw rate (rad/s)
//     );

//     // Send the message (replace mavlink_channel_t with your actual function)
//     uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
//     uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
//     // 7️⃣ Send message to ArduPilot
//     ssize_t sent = sendto(sock, buffer, len, 0,
//                           (struct sockaddr*)&remote_addr, sizeof(remote_addr));
//     if (sent < 0)
//         perror("sendto failed");
//     else
//         std::cout << "Sent SET_POSITION_TARGET_LOCAL_NED (velocity)\n";
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
