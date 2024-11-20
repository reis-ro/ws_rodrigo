
#include "maneuver_camera/maneuver_camera_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<maneuver_camera::ManeuverCameraNode>());
  rclcpp::shutdown();
  return 0;
}