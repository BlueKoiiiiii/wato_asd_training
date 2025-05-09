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
    integrateCostmap();
    global_map_.header.frame_id = "map";
    global_map_.info.width = 50;
    global_map_.info.height = 50;
    global_map_.info.resolution = 0.5;
    global_map_.info.origin.position.x = -12.5; // Center the map (50*0.5/2 = 12.5)
    global_map_.info.origin.position.y = -12.5;
    global_map_.data.resize(50 * 50, -1); // Initialize with unknown (-1)


    map_pub_->publish(global_map_);
    should_update_map = false;
}
}

void MapMemoryNode::integrateCostmap(){
  double resolution = 0.5;


  if(map_initialized == false){
    global_map_.data.resize(2500);
    map_initialized = true; 
  }

  int costmap_width = latest_costmap_info.info.width;
  int costmap_height = latest_costmap_info.info.height;


  // RCLCPP_INFO(this->get_logger(), "Integrating costmap at position (%f, %f) with orientation %f degrees", global_x, global_y, orientation * 180.0 / 3.14);

  for (int y = 0; y<costmap_height; y++){
    for (int x = 0; x<costmap_width; x++){

      double rel_x = (x - costmap_width)*resolution;
      double rel_y = (y - costmap_height)*resolution;
      // double rotated_x = rel_x * std::cos(orientation) - rel_y * std::sin(orientation);
      // double rotated_y = rel_x * std::sin(orientation) + rel_y * std::cos(orientation);

      int global_grid_x = (global_x- global_map_.info.origin.position.x)/resolution + static_cast<int>(rel_x / resolution) + costmap_width;
      int global_grid_y = (global_y - global_map_.info.origin.position.y)/resolution + static_cast<int>(rel_y / resolution) + costmap_height;

      int costmap_index = y * costmap_width + x;
      int8_t costmap_value = latest_costmap_info.data[costmap_index];

      int map_x = x + global_x; 
      int map_y = y + global_y; 


      if (global_grid_x >= 0 && global_grid_x < static_cast<int>(global_map_.info.width) && global_grid_y >= 0 && global_grid_y < static_cast<int>(global_map_.info.height)) {  
        int global_index = global_grid_y * global_map_.info.width + global_grid_x;
        /* keep a cell once it is marked “100 = occupied” */
          if (global_map_.data[global_index] != 100)          // <-- new guard
          global_map_.data[global_index] = costmap_value;

        // global_map_.data[global_index] = costmap_value;
        // RCLCPP_INFO(this->get_logger(), "Global map updated with new costmap data");
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
