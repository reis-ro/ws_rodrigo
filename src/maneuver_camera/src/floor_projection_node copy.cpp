
#include "maneuver_camera/floor_projection_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>

#include <cxxabi.h>
#include <functional>

const std::string img_topics[] = { "/carla/ego_vehicle/rgb_front/points2", "/carla/ego_vehicle/rgb_left/points2",
                                   "/carla/ego_vehicle/rgb_back/points2", "/carla/ego_vehicle/rgb_right/points2" };

using namespace std::chrono_literals;

struct FreeDeleter {
    void operator()(char* ptr) {
        free(ptr);
    }
};

std::string demangle(char const* name) {
    int status;
    std::unique_ptr<char, FreeDeleter> demangled_name(__cxxabiv1::__cxa_demangle(name, nullptr, nullptr, &status));
    switch (status) {
    case 0:
        return std::string(demangled_name.get());
    case -1:
        return "A memory allocation failure occurred.";
    case -2:
        return "name is not a valid name under the C++ ABI mangling rules.";
    case -3:
        return "One of the arguments is invalid.";
    default:
        return "Unknown error.";
    }
}

namespace maneuver_camera
{
FloorProjectionNode::FloorProjectionNode(const rclcpp::NodeOptions& options)
  : Node("FloorProjectionNode", options), coefficients_(new pcl::ModelCoefficients())
{
  RCLCPP_INFO_STREAM(this->get_logger(), "FloorProjectionNode init");

  for (uint8_t i = 0; i < 4; i++)
  {
    this->received_[i] = false;
    this->clouds_subs_[i] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        img_topics[i], 3,
        [i, this](const sensor_msgs::msg::PointCloud2::SharedPtr cloud) { return this->receive_callback(i, cloud); });
  }

  this->publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/composite_img/debug", 10);
  this->timer_ = this->create_wall_timer(1s, std::bind(&FloorProjectionNode::publish_callback, this));

  this->coefficients_->values.resize(4);
  this->coefficients_->values[0] = 0.0;
  this->coefficients_->values[1] = 1.0;
  this->coefficients_->values[2] = 0.0;
  this->coefficients_->values[3] = 0.0;
}

void FloorProjectionNode::receive_callback(uint8_t num, const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "receive_callback " << (int16_t) num);
  this->clouds_cache_[num] = cloud;
  this->received_[num] = true;
}

void FloorProjectionNode::publish_callback()
{
  if(!this->received_[0])
    return;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::fromROSMsg(*this->clouds_cache_[0], *cloud);

  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud);
  proj.setModelCoefficients(this->coefficients_);
  proj.filter(*cloud_projected);

  sensor_msgs::msg::Image::SharedPtr output_cloud_ = std::make_shared<sensor_msgs::msg::Image>();

  pcl::toROSMsg(*cloud_projected, *output_cloud_);

  output_cloud_->header = this->clouds_cache_[0]->header;

  this->publisher_->publish(*output_cloud_);
}
}  // namespace maneuver_camera
