#ifndef PLANNER_NODE_HPP
#define PLANNER_NODE_HPP

#include <functional>
#include <memory>
#include <string>
#include <queue>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot {
  class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger &logger) : logger_(logger) {}
  private:
    rclcpp::Logger logger_;
  };
}

// Helper for A* search
struct astarnode {
  int x;
  int y;
  double g_cost; // cost from start to current
  double h_cost; // heuristic cost from current to goal
  double f_cost; // g_cost + h_cost
  astarnode* parent;

  astarnode(int x_, int y_) : x(x_), y(y_), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}
};

// Simple struct for map cell indices
struct CellIndex {
  int x;
  int y;
  CellIndex() : x(0), y(0) {}
  CellIndex(int x_, int y_) : x(x_), y(y_) {}
};

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

private:
  void mapcallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomcallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pointcallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void timeout();
  
  void planpath(double start_world_x, double start_world_y, double goal_x, double goal_y);
  bool goalReached();
  
  // Convert between world coordinates and map coordinates
  void toMap(double x, double y, CellIndex &output);
  void toWorld(int x, int y, double &world_x, double &world_y); // New method to convert back to world coords
  
  robot::PlannerCore planner_;
  
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub;
  
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::Pose robot_pose_;
  geometry_msgs::msg::PointStamped goal_;
  
  bool goal_received_;
  bool state_is_waiting_for_goal;
};

#endif // PLANNER_NODE_HPP