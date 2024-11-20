#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

/*
Subscribes 4 camera topics and publishes a transformed image and the respective tf transforms
*/
namespace maneuver_camera{ 
class ManeuverCameraNode: public rclcpp::Node {

  public:

    // ManeuverCameraNode();
    ManeuverCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    void receive_callback( uint8_t num, const sensor_msgs::msg::Image::SharedPtr img);
    void publish_callback();
  private:
    std::string name;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgs_subs_[4];
    sensor_msgs::msg::Image::SharedPtr out_msg_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    sensor_msgs::msg::Image::SharedPtr imgs_[4];
};
}