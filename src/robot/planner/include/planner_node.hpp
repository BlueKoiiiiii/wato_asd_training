#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>
#include <queue>
#include <map>
#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
public:
  struct astarnode {
    int x, y;
    double g_cost; // cost from start to current
    double h_cost; // heuristic (estimated cost from current to goal)
    double f_cost; // f = g + h
    astarnode* parent;
    
    astarnode(int _x, int _y) : x(_x), y(_y), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}
    
    // For priority queue comparison
    bool operator>(const astarnode& other) const {
      return f_cost > other.f_cost;
    }
  };

  struct CellIndex {
    int x;
    int y;
    
    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}
    
    bool operator==(const CellIndex &other) const {
      return (x == other.x && y == other.y);
    }
    
    bool operator!=(const CellIndex &other) const {
      return (x != other.x || y != other.y);
    }
  };

  // Hash function for CellIndex so it can be used in std::unordered_map
  struct CellIndexHash {
    std::size_t operator()(const CellIndex &idx) const {
      // A simple hash combining x and y
      return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
  };

  // Structure representing a node in the A* open set
  struct AStarNode {
    CellIndex index;
    double f_score; // f = g + h
    
    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
  };

  // Comparator for the priority queue (min-heap by f_score)
  struct CompareF {
    bool operator()(const AStarNode &a, const AStarNode &b) {
      // We want the node with the smallest f_score on top
      return a.f_score > b.f_score;
    }
  };

  PlannerNode();
  
  void mapcallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomcallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pointcallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void timeout();
  bool goalReached();
  void toMap(double x, double y, CellIndex &output);
  void planpath(double start_world_x, double start_world_y, double goal_x, double goal_y);

  bool goal_received_ = false;
  bool state_is_waiting_for_goal = true;
  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PointStamped goal_;
  geometry_msgs::msg::Pose robot_pose_;
  
private:
  robot::PlannerCore planner_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // PLANNER_NODE_HPP_