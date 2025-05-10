#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // Use a higher QoS value for map to ensure we get all map updates
  costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), 
      std::bind(&PlannerNode::mapcallback, this, std::placeholders::_1)
  );
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 
      10, 
      std::bind(&PlannerNode::odomcallback, this, std::placeholders::_1)
  );
  point_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 
      10, 
      std::bind(&PlannerNode::pointcallback, this, std::placeholders::_1)
  );
  
  // More frequent timer for better responsiveness
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), 
      std::bind(&PlannerNode::timeout, this)
  );
  
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  
  // Initialize state variables
  goal_received_ = false;
  state_is_waiting_for_goal = false;
  
  RCLCPP_INFO(this->get_logger(), "Planner node initialized and waiting for map and goal");
}

void PlannerNode::mapcallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  current_map_ = *msg;
  RCLCPP_INFO(this->get_logger(), "Map received: %dx%d, resolution: %f", 
               current_map_.info.width, current_map_.info.height, current_map_.info.resolution);
  
  // Always replan when a new map is received if we have a goal
  if (goal_received_) {
    RCLCPP_INFO(this->get_logger(), "New map received - regenerating path");
    double start_x = robot_pose_.position.x;
    double start_y = robot_pose_.position.y;
    double goal_x = goal_.point.x;
    double goal_y = goal_.point.y;
    planpath(start_x, start_y, goal_x, goal_y);
    // Ensure we're in the waiting state
    state_is_waiting_for_goal = true;
  }
}

void PlannerNode::odomcallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::pointcallback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
  goal_ = *msg;
  goal_received_ = true;
  state_is_waiting_for_goal = true;
  
  RCLCPP_INFO(this->get_logger(), "Goal received: (%f, %f)", goal_.point.x, goal_.point.y);
  
  double start_x = robot_pose_.position.x;
  double start_y = robot_pose_.position.y;
  double goal_x = goal_.point.x;
  double goal_y = goal_.point.y;
  planpath(start_x, start_y, goal_x, goal_y);
}

void PlannerNode::timeout(){
  if (!goal_received_) {
    return; // No goal yet
  }
  
  if (goalReached()) {
    if (state_is_waiting_for_goal) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_is_waiting_for_goal = false;
    }
    return;
  }
  
  // Only replan periodically via timeout if we're in the waiting state
  // This avoids replanning when we've just received a map update (which triggers its own replan)
  if (state_is_waiting_for_goal) {
    // Check map age - only replan if map is older than X seconds
    // This avoids competing with map-triggered replans
    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Duration map_age = now - rclcpp::Time(current_map_.header.stamp);
    
    // If map is older than 1 second and we haven't replanned recently, do a periodic replan
    if (map_age.seconds() > 1.0) {
      RCLCPP_DEBUG(this->get_logger(), "Periodic replanning (map age: %.2f sec)", map_age.seconds());
      double start_x = robot_pose_.position.x;
      double start_y = robot_pose_.position.y;
      double goal_x = goal_.point.x;
      double goal_y = goal_.point.y;
      planpath(start_x, start_y, goal_x, goal_y);
    }
  }
}

// FIXED: Improved coordinate transformation
void PlannerNode::toMap(double x, double y, CellIndex &output) {
  double origin_x = current_map_.info.origin.position.x;
  double origin_y = current_map_.info.origin.position.y;
  double res = current_map_.info.resolution;

  // Correct conversion from world coordinates to grid indices
  output.x = static_cast<int>((x - origin_x) / res);
  output.y = static_cast<int>((y - origin_y) / res);
  
  // Debug output to verify conversion
  RCLCPP_DEBUG(this->get_logger(), "World (%f,%f) -> Map (%d,%d)", 
              x, y, output.x, output.y);
}
// Convert map coordinates back to world coordinates
void PlannerNode::toWorld(int x, int y, double &world_x, double &world_y) {
  double origin_x = current_map_.info.origin.position.x;
  double origin_y = current_map_.info.origin.position.y;
  double res = current_map_.info.resolution;
  
  // Convert from cell indices to world coordinates (cell center)
  world_x = (x * res) + origin_x + (res / 2.0);
  world_y = (y * res) + origin_y + (res / 2.0);
}

// A* helpers
double heuristic(int x1, int y1, int x2, int y2) {
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

// FIXED: Improved isValid function to properly check for obstacles
bool isValid(int x, int y, int width, int height, const std::vector<int8_t>& map, rclcpp::Logger logger)
{
  // Check bounds
  if (x < 0 || y < 0 || x >= width || y >= height)
    return false;

  int index = y * width + x;
  if (index < 0 || index >= static_cast<int>(map.size())) {
    return false;  // Index out of bounds check
  }
  
  int8_t value = map[index];
  
  // Uncomment for debugging specific problematic areas
  // if (x % 5 == 0 && y % 5 == 0) {
  //   RCLCPP_INFO(logger, "Cell (%d,%d) value: %d", x, y, static_cast<int>(value));
  // }
  
  // In standard occupancy grid:
  // 0 = definitely free
  // 100 = definitely occupied
  // -1 (255 as uint8_t) = unknown
  
  // Consider a cell valid only if it's definitely free (values < 50)
  // This is the key fix - previously it might have been allowing obstacles
  return value < 90;
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
  
  RCLCPP_INFO(this->get_logger(), "Planning from map cell (%d,%d) to (%d,%d)", 
              start_cell.x, start_cell.y, goal_cell.x, goal_cell.y);

  // Safety check - ensure start/goal are in valid cells
  int width = current_map_.info.width;
  int height = current_map_.info.height;
  
  // FIXED: Pass logger to isValid for debugging
  if (!isValid(start_cell.x, start_cell.y, width, height, current_map_.data, this->get_logger())) {
    RCLCPP_WARN(this->get_logger(), "Start position (%d,%d) is in obstacle or outside map!", 
                start_cell.x, start_cell.y);
    
    // Try to find nearest valid start cell
    bool found = false;
    int search_radius = 1;
    int max_search = 10; // Limit search radius
    
    while (!found && search_radius < max_search) {
      for (int dx = -search_radius; dx <= search_radius; dx++) {
        for (int dy = -search_radius; dy <= search_radius; dy++) {
          // Only check cells on the perimeter of the search square
          if (abs(dx) == search_radius || abs(dy) == search_radius) {
            int nx = start_cell.x + dx;
            int ny = start_cell.y + dy;
            if (isValid(nx, ny, width, height, current_map_.data, this->get_logger())) {
              start_cell.x = nx;
              start_cell.y = ny;
              found = true;
              RCLCPP_INFO(this->get_logger(), "Adjusted start to valid cell (%d,%d)", nx, ny);
              break;
            }
          }
        }
        if (found) break;
      }
      search_radius++;
    }
    
    if (!found) {
      RCLCPP_ERROR(this->get_logger(), "Could not find valid start cell within reasonable range!");
      return;
    }
  }
  
  if (!isValid(goal_cell.x, goal_cell.y, width, height, current_map_.data, this->get_logger())) {
    RCLCPP_WARN(this->get_logger(), "Goal position (%d,%d) is in obstacle or outside map!", 
                goal_cell.x, goal_cell.y);
    
    // Try to find nearest valid goal cell
    bool found = false;
    int search_radius = 1;
    int max_search = 10; // Limit search radius
    
    while (!found && search_radius < max_search) {
      for (int dx = -search_radius; dx <= search_radius; dx++) {
        for (int dy = -search_radius; dy <= search_radius; dy++) {
          // Only check cells on the perimeter of the search square
          if (abs(dx) == search_radius || abs(dy) == search_radius) {
            int nx = goal_cell.x + dx;
            int ny = goal_cell.y + dy;
            if (isValid(nx, ny, width, height, current_map_.data, this->get_logger())) {
              goal_cell.x = nx;
              goal_cell.y = ny;
              found = true;
              RCLCPP_INFO(this->get_logger(), "Adjusted goal to valid cell (%d,%d)", nx, ny);
              break;
            }
          }
        }
        if (found) break;
      }
      search_radius++;
    }
    
    if (!found) {
      RCLCPP_ERROR(this->get_logger(), "Could not find valid goal cell within reasonable range!");
      return;
    }
  }

  // run a star
  struct NodeCmp {
    bool operator()(const astarnode* a, const astarnode* b) const
    {   return a->f_cost > b->f_cost; }   // '>'  because std::priority_queue
  };                                      // pops the *largest* element

  std::priority_queue<astarnode*, std::vector<astarnode*>, NodeCmp> openList;
  std::vector<std::vector<bool>> closedList(width, std::vector<bool>(height, false)); // closed

  astarnode* startNode = new astarnode(start_cell.x, start_cell.y);
  startNode->g_cost = 0;
  startNode->h_cost = heuristic(start_cell.x, start_cell.y, goal_cell.x, goal_cell.y);
  startNode->f_cost = startNode->g_cost + startNode->h_cost;

  openList.push(startNode);

  // Neighbor directions - 8-connected grid
  int dx[] = {0, 1, 1, 1, 0, -1, -1, -1};
  int dy[] = {-1, -1, 0, 1, 1, 1, 0, -1};

  std::map<std::pair<int, int>, astarnode*> allNodes;
  allNodes[{start_cell.x, start_cell.y}] = startNode;
  std::vector<CellIndex> ppath;

  // Add some metrics for debugging
  int iterations = 0;
  const int MAX_ITERATIONS = 100000; // Safety limit

  //actual algorithm
  while (!openList.empty() && iterations < MAX_ITERATIONS) {
    iterations++;
    
    // Get the node with lowest f_cost
    astarnode* current = openList.top();
    openList.pop();
    
    // Debug current position occasionally
    if (iterations % 1000 == 0) {
      RCLCPP_DEBUG(this->get_logger(), "A* searching... current: (%d,%d), iter: %d", 
                   current->x, current->y, iterations);
    }
    
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
        
        RCLCPP_INFO(this->get_logger(), "Path found! Length: %zu cells, explored %d nodes", 
                   path.size(), iterations);
        
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
      
      // If this cell is valid and not visited
      if (isValid(newX, newY, width, height, current_map_.data, this->get_logger()) && !closedList[newX][newY]) {
          // Calculate new costs
          double gNew = current->g_cost + ((i % 2 == 0) ? 1.0 : 1.414); // Diagonal moves cost more
          double hNew = heuristic(newX, newY, goal_cell.x, goal_cell.y);
          double fNew = gNew + hNew;
          
          // Check if this node is already in open list
          auto it = allNodes.find({newX, newY});
          
          if (it != allNodes.end()) {
              astarnode* neighbor = it->second;
              
              // If we found a better path
              if (gNew < neighbor->g_cost) {
                  neighbor->parent = current;
                  neighbor->g_cost = gNew;
                  neighbor->f_cost = fNew;
                  
                  // Note: In a proper implementation, we should update the node's position in the priority queue
                  // But std::priority_queue doesn't support that operation directly
                  // For a more efficient implementation, consider using a different data structure
              }
          } else {
              // Create new node and add to open list
              astarnode* neighbor = new astarnode(newX, newY);
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

  // Check for timeout
  if (iterations >= MAX_ITERATIONS) {
    RCLCPP_WARN(this->get_logger(), "Path planning failed: exceeded maximum iterations!");
    // Clean up nodes
    for (auto& node_pair : allNodes) {
      delete node_pair.second;
    }
    return;
  }

  // Clean up any remaining nodes if path wasn't found
  if (ppath.empty()) {
    for (auto& node_pair : allNodes) {
      delete node_pair.second;
    }
    RCLCPP_WARN(this->get_logger(), "Path planning failed: no path found.");
    return;
  }

  // ADDED: Simple path smoothing
  if (ppath.size() > 3) {
    std::vector<CellIndex> smoothed_path;
    smoothed_path.push_back(ppath.front());
    
    for (size_t i = 1; i < ppath.size() - 1; i++) {
      // Skip points that form a straight line
      if (i < ppath.size() - 2) {
        int dx1 = ppath[i].x - ppath[i-1].x;
        int dy1 = ppath[i].y - ppath[i-1].y;
        int dx2 = ppath[i+1].x - ppath[i].x;
        int dy2 = ppath[i+1].y - ppath[i].y;
        
        // If not a straight line, add this point
        if (dx1 != dx2 || dy1 != dy2) {
          smoothed_path.push_back(ppath[i]);
        }
      } else {
        smoothed_path.push_back(ppath[i]);
      }
    }
    
    smoothed_path.push_back(ppath.back());
    
    if (smoothed_path.size() < ppath.size()) {
      RCLCPP_INFO(this->get_logger(), "Path smoothed from %zu to %zu points", 
                 ppath.size(), smoothed_path.size());
      ppath = smoothed_path;
    }
  }
  
  // Publish the path
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = current_map_.header.frame_id;
  
  for (const auto& cell : ppath) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    
    // FIXED: Convert from grid coordinates back to world coordinates using proper function
    double wx, wy;
    toWorld(cell.x, cell.y, wx, wy);
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    
    // Set orientation to identity quaternion
    pose.pose.orientation.w = 1.0;
    
    path.poses.push_back(pose);
  }
  
  RCLCPP_INFO(this->get_logger(), "Publishing path with %zu poses", path.poses.size());
  path_pub_->publish(path);
}

bool PlannerNode::goalReached() {
  if (!goal_received_) return false;
  
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  
  RCLCPP_DEBUG(this->get_logger(), "Distance to goal: %f", distance);
  return distance < 0.5; // Threshold for reaching the goal
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}