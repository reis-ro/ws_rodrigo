#ifndef CUSTOM_PARKING_PLANNER_HPP_
#define CUSTOM_PARKING_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <Eigen/Dense>


namespace custom_parking_planner
{
class CustomParkingPlanner : public nav2_core::GlobalPlanner
{
public:
  CustomParkingPlanner() = default;
  ~CustomParkingPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  nav2_util::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_, name_;

  // Parking points calculation method
  std::vector<geometry_msgs::msg::PoseStamped> generateParkingPoints(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double R);

  // Discretize the arc of the circle between two points
  std::vector<geometry_msgs::msg::PoseStamped> discretizeCircle(
    const Eigen::Vector2d & center, const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end, double radius, bool clockwise);

  // Function to calculate parking maneuver points
  void calculateParkingPoints(
    double R, const geometry_msgs::msg::PoseStamped & goal,
    const geometry_msgs::msg::PoseStamped & start, Eigen::Vector2d & c1,
    Eigen::Vector2d & c2, Eigen::Vector2d & transition_pt);
};

}  // namespace custom_parking_planner

#endif  // PARKING_PATH_PLANNER_HPP_
