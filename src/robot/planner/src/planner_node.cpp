#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapcallback, this, std::placeholders::_1));
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomcallback, this, std::placeholders::_1));
  point_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::pointcallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&PlannerNode::timeout, this));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
}

void PlannerNode::mapcallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  current_map_ = *msg;
  if (state_is_waiting_for_goal) {
    double start_x = robot_pose_.position.x;
    double start_y = robot_pose_.position.y;
    double goal_x = goal_.point.x;
    double goal_y = goal_.point.y;
    planpath(start_x, start_y, goal_x, goal_y);
  }
}

void PlannerNode::odomcallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::pointcallback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
  goal_ = *msg;
  goal_received_ = true;
  state_is_waiting_for_goal = true;
  double start_x = robot_pose_.position.x;
  double start_y = robot_pose_.position.y;
  double goal_x = goal_.point.x;
  double goal_y = goal_.point.y;
  planpath(start_x, start_y, goal_x, goal_y);
}

void PlannerNode::timeout(){
  if (state_is_waiting_for_goal) {
    if (goalReached()) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        state_is_waiting_for_goal = false; 
    } else {
        RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
        double start_x = robot_pose_.position.x;
        double start_y = robot_pose_.position.y;
        double goal_x = goal_.point.x;
        double goal_y = goal_.point.y;
        planpath(start_x, start_y, goal_x, goal_y);
    }
  }
}

void PlannerNode::toMap(double x, double y, CellIndex &output) {
  double origin_x = current_map_.info.origin.position.x;
  double origin_y = current_map_.info.origin.position.y;
  double res = current_map_.info.resolution;

  double cell_x = (x - origin_x) / res;
  double cell_y = (y - origin_y) / res;

  output.x = static_cast<int>(std::round(cell_x));
  output.y = static_cast<int>(std::round(cell_y));
}

// A* helpers
double heuristic(int x1, int y1, int x2, int y2) {
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

bool isValid(int x, int y, int width, int height, const std::vector<int8_t>& map)
{
  if (x < 0 || y < 0 || x >= width || y >= height)
  return false;

  int index = y * width + x;
  int8_t v  = map[index];

  /* keep out of unknown or occupied */
  return v >= 0 && v < 70;   // 0-49 free, ≥50 occupied, -1 unknown
}
void PlannerNode::planpath(double start_world_x, double start_world_y, double goal_x, double goal_y){
  if (!goal_received_ || current_map_.data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
      return;
  }

  // Convert world coordinates to map coordinates
  CellIndex start_cell, goal_cell;
  toMap(start_world_x, start_world_y, start_cell);
  toMap(goal_x, goal_y, goal_cell);

  // run a star
  int width  = current_map_.info.width;
  int height = current_map_.info.height;

  struct NodeCmp {
    bool operator()(const astarnode* a, const astarnode* b) const
    {   return a->f_cost > b->f_cost; }   // ‘>’  because std::priority_queue
};                                        // pops the *largest* element

std::priority_queue<astarnode*, std::vector<astarnode*>, NodeCmp> openList;
  std::vector<std::vector<bool>> closedList(width, std::vector<bool>(height, false)); // closed

  astarnode* startNode = new astarnode(start_cell.x, start_cell.y);
  startNode->g_cost = 0;
  startNode->h_cost = heuristic(start_cell.x, start_cell.y, goal_cell.x, goal_cell.y);
  startNode->f_cost = startNode->g_cost + startNode->h_cost;

  openList.push(startNode);

  int dx[] = {0, 1, 1, 1, 0, -1, -1, -1};
  int dy[] = {-1, -1, 0, 1, 1, 1, 0, -1};

  std::map<std::pair<int, int>, astarnode*> allNodes;
  allNodes[{start_cell.x, start_cell.y}] = startNode;
  std::vector<CellIndex> ppath;

  //actual algorithm
  while (!openList.empty()) {
    // Get the node with lowest f_cost
    astarnode* current = openList.top();
    openList.pop();
    
    //end condition
    if (current->x == goal_cell.x && current->y == goal_cell.y) {
        // Reconstruct the path
        std::vector<CellIndex> path;
        while (current != nullptr) {
            CellIndex cell(current->x, current->y);
            path.push_back(cell);
            current = current->parent;
        }
        
        // Reverse to get path from start to goal
        std::reverse(path.begin(), path.end());
        
        // Clean up nodes and save path
        for (auto& node_pair : allNodes) {
            delete node_pair.second;
        }
        ppath = path;
        break; // Exit the loop as we found the path
    }

    closedList[current->x][current->y] = true;
    
    for (int i = 0; i < 8; i++) {
      int newX = current->x + dx[i];
      int newY = current->y + dy[i];
      const auto& mapData = current_map_.data; 
      
      // If this cell is valid and not visited
      if (isValid(newX, newY, width, height, mapData) && !closedList[newX][newY]) {
          // Calculate new costs
          double gNew = current->g_cost + ((i % 2 == 0) ? 1.0 : 1.414); // Diagonal moves cost more
          double hNew = heuristic(newX, newY, goal_cell.x, goal_cell.y);
          double fNew = gNew + hNew;
          
          // Check if this node is already in open list and has a better path
          bool found = false;
          astarnode* neighbor = nullptr;
          
          auto it = allNodes.find({newX, newY});
          if (it != allNodes.end()) {
              neighbor = it->second;
              found = true;
              
              // If we found a better path
              if (gNew < neighbor->g_cost) {
                  neighbor->parent = current;
                  neighbor->g_cost = gNew;
                  neighbor->f_cost = fNew;
              }
          } else {
              // Create new node and add to open list
              neighbor = new astarnode(newX, newY);
              neighbor->g_cost = gNew;
              neighbor->h_cost = hNew;
              neighbor->f_cost = fNew;
              neighbor->parent = current;
              
              allNodes[{newX, newY}] = neighbor;
              openList.push(neighbor);
          }
      }
    }
  }

  // Clean up any remaining nodes if path wasn't found
  if (ppath.empty()) {
    for (auto& node_pair : allNodes) {
      delete node_pair.second;
    }
    RCLCPP_WARN(this->get_logger(), "Path planning failed: no path found.");
    return;
  }

  // Publish the path
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "sim_world";
  
  for (const auto& cell : ppath) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    
    // Convert from grid coordinates back to world coordinates
    pose.pose.position.x = cell.x * current_map_.info.resolution + current_map_.info.origin.position.x;
    pose.pose.position.y = cell.y * current_map_.info.resolution + current_map_.info.origin.position.y;
    pose.pose.position.z = 0.0;
    
    // Set orientation to identity quaternion
    pose.pose.orientation.w = 1.0;
    
    path.poses.push_back(pose);
  }
  
  path_pub_->publish(path);
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}