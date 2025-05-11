#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapcallback, this, std::placeholders::_1));
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomcallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updatemap, this));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
}


void MapMemoryNode::odomcallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  // RCLCPP_INFO(this->get_logger(), "recieved Odom");
  global_x = msg->pose.pose.position.x;
  global_y = msg->pose.pose.position.y;
  orientation = msg->pose.pose.orientation.w; 

  double distance = std::sqrt(std::pow(global_x - last_x, 2) + std::pow(global_y - last_y, 2));
  if (distance >= distance_threshold) {
    last_x = global_x;
    last_y = global_y;
    should_update_map = true;
    // RCLCPP_INFO(this->get_logger(), "distance threshold reached");

}
}

void MapMemoryNode::costmapcallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  // RCLCPP_INFO(this->get_logger(), "recieved costmap");

  for(int y = 9; y>=0; y--){
    for(int x = 9; x>=0; x--){
      int index = y * 10 + x;
      latestcostmap[x][y]=msg->data[index];
    }
  }
  latest_costmap_info = *msg; 
  costmap_updated = true;
}

void MapMemoryNode::updatemap(){
  if (should_update_map && costmap_updated) {
<<<<<<< Updated upstream
=======
    if(map_initialized == false){
    global_map_.data.resize(2500);
    std::fill(global_map_.data.begin(), global_map_.data.end(), -1);  // Properly initialize with unknown
    map_initialized = true; 
  }
>>>>>>> Stashed changes
    integrateCostmap();
    global_map_.header.frame_id = "map";
    global_map_.info.width = 50;
    global_map_.info.height = 50;
    global_map_.info.resolution = 0.6;  // Fixed missing semicolon
    global_map_.info.origin.position.x = -12.5; // Center the map (50*0.5/2 = 12.5)
    global_map_.info.origin.position.y = -12.5;
    global_map_.data.resize(50 * 50, -1); // Initialize with unknown (-1)
<<<<<<< Updated upstream

=======
    
>>>>>>> Stashed changes
    map_pub_->publish(global_map_);
  }
}
void MapMemoryNode::integrateCostmap(){
  double resolution = 0.6;

<<<<<<< Updated upstream
  if(map_initialized == false){
    global_map_.data.resize(2500);
    std::fill(global_map_.data.begin(), global_map_.data.end(), -1);  // Properly initialize with unknown
    map_initialized = true; 
  }
=======

>>>>>>> Stashed changes

  int costmap_width = 10;  // Based on your costmap array size
  int costmap_height = 10; // Based on your costmap array size

  // Debug info
  RCLCPP_INFO(this->get_logger(), "Integrating costmap at position (%f, %f)", global_x, global_y);

  for (int y = 0; y < costmap_height; y++){
    for (int x = 0; x < costmap_width; x++){
      // Calculate relative position from robot center
      double rel_x = (x - costmap_width/2.0) * 0.5;  // Assuming 0.5m per costmap cell
      double rel_y = (y - costmap_height/2.0) * 0.5; // Assuming 0.5m per costmap cell
      
      // Calculate global grid position (in the map frame)
      int global_grid_x = static_cast<int>((global_x + rel_x - global_map_.info.origin.position.x) / resolution);
      int global_grid_y = static_cast<int>((global_y + rel_y - global_map_.info.origin.position.y) / resolution);

      // Get the costmap value at this position
      int8_t costmap_value = latestcostmap[x][y];

      // Check if the calculated position is within the global map
      if (global_grid_x >= 0 && global_grid_x < static_cast<int>(global_map_.info.width) && 
          global_grid_y >= 0 && global_grid_y < static_cast<int>(global_map_.info.height)) {  
        
        int global_index = global_grid_y * global_map_.info.width + global_grid_x;
        
        // Only update if the current cell isn't already known to be an obstacle
        // This preserves obstacles in the map
        if (global_map_.data[global_index] != 100) {
          global_map_.data[global_index] = costmap_value;
          
          // Debug some values to verify integration
          if (costmap_value > 50) {
            RCLCPP_DEBUG(this->get_logger(), "Adding obstacle at map cell (%d,%d) value: %d", 
                         global_grid_x, global_grid_y, static_cast<int>(costmap_value));
          }
        }
      }
    }
  }
}








int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
