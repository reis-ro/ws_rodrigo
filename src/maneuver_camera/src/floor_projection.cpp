
#include "maneuver_camera/floor_projection_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<maneuver_camera::FloorProjectionNode>());
  rclcpp::shutdown();
  return 0;
}