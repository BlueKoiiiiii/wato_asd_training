#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
    // Subscribers and Publishers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, 
        [this](const nav_msgs::msg::Path::SharedPtr msg) { 
            current_path_ = msg; 
            RCLCPP_INFO(this->get_logger(), "Received new path with %zu poses", msg->poses.size());
        });
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, 
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

    point_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, 
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { 
            point_stamp = msg; 
            goal_reached_ = false; // Reset goal reached when new goal received
            RCLCPP_INFO(this->get_logger(), "Received new goal point: (%f, %f)", 
                      msg->point.x, msg->point.y);
        });
    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Parameters
    this->declare_parameter("lookahead_distance", 1.0);
    this->declare_parameter("min_lookahead_distance", 0.3);  // Add minimum lookahead
    this->declare_parameter("max_linear_velocity", 1.5);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("goal_tolerance", 1.0);
    this->declare_parameter("path_reset_threshold", 3.0);    // Add path reset threshold
    
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    min_lookahead_distance_ = this->get_parameter("min_lookahead_distance").as_double();
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    path_reset_threshold_ = this->get_parameter("path_reset_threshold").as_double();
    
    // Timer
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
        [this]() { controlLoop(); });
    
    // Initialize goal reached flag
    goal_reached_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Control node initialized with parameters:");
    RCLCPP_INFO(this->get_logger(), "  lookahead_distance: %f", lookahead_distance_);
    RCLCPP_INFO(this->get_logger(), "  min_lookahead_distance: %f", min_lookahead_distance_);
    RCLCPP_INFO(this->get_logger(), "  max_linear_velocity: %f", max_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "  max_angular_velocity: %f", max_angular_velocity_);
    RCLCPP_INFO(this->get_logger(), "  goal_tolerance: %f", goal_tolerance_);
    RCLCPP_INFO(this->get_logger(), "  path_reset_threshold: %f", path_reset_threshold_);
}

void ControlNode::controlLoop() {
    // Skip control if no path or odometry data is available
    if (!current_path_ || !robot_odom_ || !point_stamp) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                           "Missing path, odometry, or goal data");
        return;
    }
    
    // Check if we have reached the goal
    if (checkGoalReached()) {
        if (!goal_reached_) {
            RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping robot.");
            goal_reached_ = true;
            
            // Send a zero velocity command to stop the robot
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(stop_cmd);
        }
        return;
    }
    
    // Reset goal reached flag if we're moving to a new goal
    goal_reached_ = false;
    
    // Find the lookahead point
    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No valid lookahead point found");
        
        // Emergency stop if no valid lookahead point
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }
    
    // Compute velocity command
    auto cmd_vel = computeVelocity(*lookahead_point);
    
    // Publish the velocity command
    cmd_vel_pub_->publish(cmd_vel);
}

bool ControlNode::checkGoalReached() {
    if (!current_path_ || current_path_->poses.empty() || !robot_odom_ || !point_stamp) {
        return false;
    }
    
    // Get the final goal point from the path
    const auto& goal_point = point_stamp->point;
    const auto& robot_pos = robot_odom_->pose.pose.position;
    
    // Calculate distance to goal
    double distance_to_goal = computeDistance(robot_pos, goal_point);
    
    // Check if we're close enough to the goal
    return distance_to_goal <= goal_tolerance_;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    if (!current_path_ || current_path_->poses.empty()) {
        return std::nullopt;
    }
    
    // Get robot position from odometry
    const auto& robot_pos = robot_odom_->pose.pose.position;
    
    // Find the closest point on the path
    size_t closest_idx = 0;
    double min_distance = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < current_path_->poses.size(); ++i) {
        double dist = computeDistance(robot_pos, current_path_->poses[i].pose.position);
        if (dist < min_distance) {
            min_distance = dist;
            closest_idx = i;
        }
    }
    
    // Check if we're too far from the path
    if (min_distance > path_reset_threshold_) {
        RCLCPP_WARN(this->get_logger(), "Robot is too far from the path (%.2f m). Waiting for new path.", 
                  min_distance);
        return std::nullopt;
    }
    
    // Debug info
    RCLCPP_DEBUG(this->get_logger(), "Closest point index: %zu, distance: %.2f", closest_idx, min_distance);
    
    // Dynamic lookahead distance - adjust based on current speed
    double current_speed = std::sqrt(
        std::pow(robot_odom_->twist.twist.linear.x, 2) + 
        std::pow(robot_odom_->twist.twist.linear.y, 2));
    
    // Adjust lookahead distance based on speed (between min and max)
    double adjusted_lookahead = std::max(min_lookahead_distance_, 
                                       std::min(lookahead_distance_, 
                                              min_lookahead_distance_ + current_speed));
    
    RCLCPP_DEBUG(this->get_logger(), "Using adjusted lookahead distance: %.2f", adjusted_lookahead);
    
    // First, try looking ahead from the closest point
    for (size_t i = closest_idx; i < current_path_->poses.size(); ++i) {
        double dist = computeDistance(robot_pos, current_path_->poses[i].pose.position);
        if (dist >= adjusted_lookahead) {
            RCLCPP_DEBUG(this->get_logger(), "Found lookahead point ahead at index %zu, distance %.2f", 
                       i, dist);
            return current_path_->poses[i];
        }
    }
    
    // If we didn't find a point ahead that's far enough, use the end of the path
    if (closest_idx < current_path_->poses.size() - 1) {
        RCLCPP_DEBUG(this->get_logger(), "Using end of path as lookahead point");
        return current_path_->poses.back();
    }
    
    // If the robot is close to the end of the path, return the end point
    if (min_distance < adjusted_lookahead && !current_path_->poses.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Robot close to end of path, using end point");
        return current_path_->poses.back();
    }
    
    RCLCPP_WARN(this->get_logger(), "Could not find a valid lookahead point");
    return std::nullopt;
}

geometry_msgs::msg::Twist
ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target)
{
    geometry_msgs::msg::Twist cmd;

    /* robot pose */
    const auto &p     = robot_odom_->pose.pose.position;
    const auto &q     = robot_odom_->pose.pose.orientation;
    double yaw_robot  = extractYaw(q);

    /* vector to look-ahead target in world frame */
    double dx = target.pose.position.x - p.x;
    double dy = target.pose.position.y - p.y;
    
    // Calculate distance to target point
    double distance_to_target = std::sqrt(dx*dx + dy*dy);

    /* transform that vector into the robot frame */
    double x_r =  dx * std::cos(-yaw_robot) - dy * std::sin(-yaw_robot);
    double y_r =  dx * std::sin(-yaw_robot) + dy * std::cos(-yaw_robot);

    /* ------------- pure-pursuit steering law -------------------- */
    double L2 = x_r * x_r + y_r * y_r;              // (look-ahead distance)²
    if (L2 < 1e-6) {                               // protection against /0
        RCLCPP_WARN(this->get_logger(), "Lookahead distance too small, stopping robot");
        return cmd;
    }

    double curvature = 2.0 * y_r / L2;              // ①: uses x_r / y_r ONLY
    /* ------------------------------------------------------------ */

    /* ------------- dynamic speed scaling ----------------------- */
    constexpr double k_speed = 2.0;                 // ③ tune 1-3 (higher ⇒ slower in bends)
    
    // Base speed scaled by distance to target
    double base_speed = max_linear_velocity_;
    
    // Slow down when approaching the goal
    if (distance_to_target < lookahead_distance_) {
        base_speed *= (distance_to_target / lookahead_distance_);
    }
    
    // Slow down in curves
    double v = base_speed / (1.0 + k_speed * std::abs(curvature));
    
    // Add minimum velocity to prevent getting stuck
    v = std::max(v, 0.1);
    
    // Calculate angular velocity
    double w = curvature * v;
    /* ------------------------------------------------------------ */

    /* saturate angular speed & re-scale v so you respect both limits */
    if (std::abs(w) > max_angular_velocity_) {
        double scale = max_angular_velocity_ / std::abs(w);
        w *= scale;
        v *= scale;
    }

    // Handle path behind the robot - x_r can be negative when target is behind
    if (x_r < 0) {
        RCLCPP_DEBUG(this->get_logger(), "Target is behind robot, adjusting velocity");
        // Slow down more when target is behind
        v *= 0.5;
        // Ensure minimum velocity
        v = std::max(v, 0.05);
    }

    cmd.linear.x  = v;
    cmd.angular.z = w;
    
    RCLCPP_DEBUG(this->get_logger(), "Computed velocity: linear=%.2f, angular=%.2f, target: (%.2f, %.2f), curvature: %.2f", 
               v, w, target.pose.position.x, target.pose.position.y, curvature);
    
    return cmd;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    
    return std::sqrt(dx*dx + dy*dy);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &q) {
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}