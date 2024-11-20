#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

/*
Subscribes 4 camera topics and publishes a transformed image and the respective tf transforms
*/
namespace maneuver_camera{
class FloorProjectionNode: public rclcpp::Node {

  public:

    FloorProjectionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    void receive_callback(uint8_t num, const sensor_msgs::msg::PointCloud2::SharedPtr img);
    void publish_callback();
  private:
    bool received_[4];
    std::string name;
    pcl::ModelCoefficients::Ptr coefficients_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr clouds_subs_[4];
    sensor_msgs::msg::PointCloud2::SharedPtr clouds_cache_[4];
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    sensor_msgs::msg::PointCloud2::SharedPtr final_cloud_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};
}