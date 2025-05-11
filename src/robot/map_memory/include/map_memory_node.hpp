#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include<vector>
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    void costmapcallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg); 
    void odomcallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    double last_x = 0; 
    double last_y = 0; 
    int distance_threshold = 5; 
    nav_msgs::msg::OccupancyGrid latest_costmap_info; 
    int latestcostmap[10][10] = {};
    bool costmap_updated = false;
    bool should_update_map = false; 
    nav_msgs::msg::OccupancyGrid global_map_;



    void updatemap(); 
    double global_x = 0; 
    double orientation = 0;
    double global_y = 0; 
    bool map_initialized = false; 
    void integrateCostmap(); 
    std::vector<std::vector<int>> map; 

  private:
    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub; 
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

};

#endif 
