#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <optional>
#include <cmath>
#include <limits>

namespace robot {
    class ControlCore {
    public:
        explicit ControlCore(rclcpp::Logger logger) : logger_(logger) {}
        
    private:
        rclcpp::Logger logger_;
    };
    
}

class ControlNode : public rclcpp::Node {
public:
    ControlNode();

private:
    double goal_tolerance_;
    bool goal_reached_;

    bool checkGoalReached();
    void controlLoop();
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target);
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &q);
    
    // Core controller
    robot::ControlCore control_;
    
    // Parameters
    double lookahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    
    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Data
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;
    geometry_msgs::msg::PointStamped::SharedPtr point_stamp;

};

#endif // CONTROL_NODE_HPP