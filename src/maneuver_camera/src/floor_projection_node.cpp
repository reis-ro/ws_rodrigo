
#include "maneuver_camera/floor_projection_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/transforms.hpp>

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

  this->publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/composite_img/debug", 10);
  this->timer_ = this->create_wall_timer(1s, std::bind(&FloorProjectionNode::publish_callback, this));

  this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

  this->final_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

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
  if(!this->received_[0] or !this->received_[1] or !this->received_[2] or !this->received_[3])
    return;

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);

  // pcl::fromROSMsg(*this->clouds_cache_[0], *cloud);

  this->final_cloud_.reset();
  this->final_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  sensor_msgs::msg::PointCloud2::SharedPtr tmp_[4];
  for(uint8_t i = 0; i < 4; i++){
    tmp_[i] = std::make_shared<sensor_msgs::msg::PointCloud2>();
    RCLCPP_INFO_STREAM(this->get_logger(), "transformPointCloud begin" << (int16_t) i);
    pcl_ros::transformPointCloud("ego_vehicle", *this->clouds_cache_[i], *tmp_[i], *this->tf_buffer_);
    RCLCPP_INFO_STREAM(this->get_logger(), "transformPointCloud end " << (int16_t) i);
  }

  sensor_msgs::msg::PointCloud2::SharedPtr mid_[2];

  mid_[0] = std::make_shared<sensor_msgs::msg::PointCloud2>();
  mid_[1] = std::make_shared<sensor_msgs::msg::PointCloud2>();

  pcl::concatenatePointCloud(*tmp_[0], *tmp_[1], *mid_[0]);
  pcl::concatenatePointCloud(*tmp_[2], *tmp_[3], *mid_[1]);

  pcl::concatenatePointCloud(*mid_[0], *mid_[1], *this->final_cloud_);

  RCLCPP_INFO_STREAM(this->get_logger(), "exit loop");

  // pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  // proj.setModelType(pcl::SACMODEL_PLANE);
  // proj.setInputCloud(cloud);
  // proj.setModelCoefficients(this->coefficients_);
  // proj.filter(*cloud_projected);

  // pcl::toROSMsg(*cloud_projected, *output_cloud_);

  final_cloud_->header = this->clouds_cache_[0]->header;
  final_cloud_->header.frame_id = "ego_vehicle";

  this->publisher_->publish(*this->final_cloud_);
}
}  // namespace maneuver_camera
