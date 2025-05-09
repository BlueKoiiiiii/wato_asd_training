#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
    // Subscribers and Publishers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, 
        [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, 
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

    point_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, 
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { point_stamp = msg; });

    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Parameters
    this->declare_parameter("lookahead_distance", 1.0);
    this->declare_parameter("max_linear_velocity", 0.5);
    this->declare_parameter("max_angular_velocity", 1.0);
    this->declare_parameter("goal_tolerance", 1.0);  // Add a goal tolerance parameter
    
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();  // Get the tolerance
    
    // Timer
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
        [this]() { controlLoop(); });
    
    // Initialize goal reached flag
    goal_reached_ = false;
}

void ControlNode::controlLoop() {
    // Skip control if no path or odometry data is available
    if (!current_path_ || !robot_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Missing path or odometry data");
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
        return; // No valid lookahead point found
    }
    
    // Compute velocity command
    auto cmd_vel = computeVelocity(*lookahead_point);
    
    // Publish the velocity command
    cmd_vel_pub_->publish(cmd_vel);
}

bool ControlNode::checkGoalReached() {
    if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
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
    if (current_path_->poses.empty()) {
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
    
    // Look for a point that is approximately lookahead_distance_ away
    for (size_t i = closest_idx; i < current_path_->poses.size(); ++i) {
        double dist = computeDistance(robot_pos, current_path_->poses[i].pose.position);
        if (dist >= lookahead_distance_) {
            return current_path_->poses[i];
        }
    }
    
    // If we can't find a point far enough, use the last point
    if (!current_path_->poses.empty()) {
        return current_path_->poses.back();
    }
    
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

    /* transform that vector into the robot frame */
    double x_r =  dx * std::cos(-yaw_robot) - dy * std::sin(-yaw_robot);
    double y_r =  dx * std::sin(-yaw_robot) + dy * std::cos(-yaw_robot);

    /* ------------- pure-pursuit steering law -------------------- */
    double L2 = x_r * x_r + y_r * y_r;              // (look-ahead distance)²
    if (L2 < 1e-6)                                  // protection against /0
        return cmd;

    double curvature = 2.0 * y_r / L2;              // ①: uses x_r / y_r ONLY
    /* ------------------------------------------------------------ */

    /* ------------- dynamic speed scaling ----------------------- */
    constexpr double k_speed = 2.0;                 // ③ tune 1-3 (higher ⇒ slower in bends)
    double v = max_linear_velocity_ / (1.0 + k_speed * std::abs(curvature));
    double w = curvature * v;
    /* ------------------------------------------------------------ */

    /* saturate angular speed & re-scale v so you respect both limits */
    if (std::abs(w) > max_angular_velocity_) {
        double scale = max_angular_velocity_ / std::abs(w);
        w *= scale;
        v *= scale;
    }

    cmd.linear.x  = v;
    cmd.angular.z = w;
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