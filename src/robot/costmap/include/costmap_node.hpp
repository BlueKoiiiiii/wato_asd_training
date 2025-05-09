#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "costmap_core.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <iostream>
#include <vector>

// #include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

// #include <tf2/LinearMath/Matrix3x3.h>
// #include <geometry_msgs/msg/quaternion.hpp>
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    double orientation = 0; 
    double getYawFromOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishMessage();
    int OccupancyGrid[10][10] = {};
    void lidartogrid(const std::vector<float>& ranges, float angle_min, float angle_max, float angle_increment); 
    void inflateObstacles();
 
  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; 
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr laser_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    // sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};


 
#endif 