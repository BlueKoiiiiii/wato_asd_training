#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  laser_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&CostmapNode::OdomCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1));
}

void CostmapNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  const geometry_msgs::msg::Quaternion &q = msg->pose.pose.orientation;
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 mat(quat);
  double roll,pitch,yaw;
  mat.getRPY(roll,pitch,yaw);
  orientation = yaw; 
  // RCLCPP_INFO(this->get_logger(), "orientation%.6f", orientation);
}



void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // latest_scan_ = msg;
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      CostmapNode::OccupancyGrid[i][j] = 0;
    }
  }
  lidartogrid(msg->ranges, msg->angle_min, msg->angle_min, msg->angle_increment); 
  publishMessage(); 
  // RCLCPP_DEBUG(this->get_logger(), "processed laser scan");
}

void CostmapNode::lidartogrid(const std::vector<float>& ranges, float angle_min, float angle_max, float angle_increment){
  float resolution = 0.5;
  int robot_grid_x = 5; 
  int robot_grid_y = 5;

  for(size_t i = 0; i < ranges.size() && i < 256; i++){
    float angle = angle_min + i * angle_increment;
    float global_angle = angle + orientation;
    
    float x = ranges[i] * std::cos(global_angle);
    float y = ranges[i] * std::sin(global_angle);
    
    int grid_x = robot_grid_x + static_cast<int>(x / resolution);
    int grid_y = robot_grid_y + static_cast<int>(y / resolution);
    
    // Check if within grid bounds
    if (grid_x >= 0 && grid_x < 10 && grid_y >= 0 && grid_y < 10) {
      OccupancyGrid[grid_x][grid_y] = 100; // Mark as obstacle
    }
  }
  
  inflateObstacles();  
}

void CostmapNode::inflateObstacles(){
  int inflation_radius = 3/0.5; 
  for (int x = 0; x < 10; x++) {
    for (int y = 0; y < 10; y++) {
      if(OccupancyGrid[x][y]==100){
        for (int dx = -inflation_radius; dx <= inflation_radius; dx++) {
          for (int dy = -inflation_radius; dy <= inflation_radius; dy++) {
            int nx = x + dx;
            int ny = y + dy;

            if(dx!=0 and dy!=0 and nx<10 and ny<10 and nx>=0 and ny>=0){
              float distance = sqrt(dx*dx + dy*dy)*0.5; 
              int cost = (80 * (1.0 - distance/inflation_radius));
              if (cost > OccupancyGrid[nx][ny]) {
                OccupancyGrid[nx][ny] = cost;
                // RCLCPP_INFO(this->get_logger(), "Added inflation to occupancygrid:%d", cost);
            }
            }


      }


}
      }
    }
  }
}

// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  // if (latest_scan_ != nullptr) {
  //   RCLCPP_INFO(this->get_logger(), "Publishing LaserScan message");
  //   laser_pub_->publish(*latest_scan_);  // Republish the exact same message
  // } else {
  //   RCLCPP_INFO(this->get_logger(), "No LaserScan data received yet");
  // }
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.stamp = this->now();
  grid_msg.header.frame_id = "map";

  grid_msg.info.resolution = 0.5;
  grid_msg.info.width = 10;
  grid_msg.info.height = 10;
  grid_msg.info.origin.position.x = -5.0;
  grid_msg.info.origin.position.y = 5.0;


  grid_msg.data.resize(100);
  for (int y = 0; y < 10; y++) {
    for (int x = 0; x < 10; x++) {
      int index = y * 10 + x;
      int val = OccupancyGrid[x][y];
      grid_msg.data[index] = val;
    }
  }

  laser_pub_->publish(grid_msg);
  // laser_pub_->publish(OccupancyGrid);

}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}