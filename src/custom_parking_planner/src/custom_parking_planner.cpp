// // #include <cmath>
// // #include <string>
// // #include <memory>
// // #include <vector>
// // #include <Eigen/Core>  // Include Eigen for vector calculations
// // #include <geometry_msgs/msg/pose_stamped.hpp>
// // #include <nav_msgs/msg/path.hpp>
// // #include <pluginlib/class_list_macros.hpp>
// // #include "rclcpp/rclcpp.hpp"
// // #include "tf2_ros/buffer.h"
// // #include "nav2_core/global_planner.hpp"
// // #include "nav2_util/node_utils.hpp"
// // #include "nav2_costmap_2d/costmap_2d_ros.hpp"
// // #include "nav2_costmap_2d/costmap_2d.hpp"
// // #include "nav_msgs/msg/path.hpp"
// // #include "geometry_msgs/msg/pose_stamped.hpp"
// // #include "rclcpp_lifecycle/lifecycle_node.hpp"
// // #include <Eigen/Dense>
// // #include <tf2/LinearMath/Quaternion.h>
// // #include <tf2/convert.h>

// // // #include "custom_parking_planner/custom_parking_planner.hpp"

// // namespace custom_parking_planner
// // {

// // class CustomParkingPlanner : public nav2_core::GlobalPlanner
// // {
// // public:
// //   void configure(
// //     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
// //     std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
// //     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
// //   {
// //     node_ = parent.lock();
// //     name_ = name;
// //     tf_ = tf;
// //     costmap_ = costmap_ros->getCostmap();
// //     global_frame_ = costmap_ros->getGlobalFrameID();

// //     // Logger for configuration
// //     RCLCPP_INFO(node_->get_logger(), "Configuring plugin %s of type CustomParkingPlanner", name_.c_str());
// //   }

// //   void cleanup() override
// //   {
// //     RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s of type CustomParkingPlanner", name_.c_str());
// //   }

// //   void activate() override
// //   {
// //     RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type CustomParkingPlanner", name_.c_str());
// //   }

// //   void deactivate() override
// //   {
// //     RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type CustomParkingPlanner", name_.c_str());
// //   }

// //   nav_msgs::msg::Path createPlan(
// //     const geometry_msgs::msg::PoseStamped & start,
// //     const geometry_msgs::msg::PoseStamped & goal) override
// //   {
// //     nav_msgs::msg::Path global_path;
// //     double R = 3.1974802362597052;
// //     // Validate frames
// //     if (start.header.frame_id != global_frame_) {
// //       RCLCPP_ERROR(node_->get_logger(), "Planner only accepts start position from %s frame", global_frame_.c_str());
// //       return global_path;
// //     }

// //     if (goal.header.frame_id != global_frame_) {
// //       RCLCPP_ERROR(node_->get_logger(), "Planner only accepts goal position from %s frame", global_frame_.c_str());
// //       return global_path;
// //     }

// //     // Get the parking points (c1, c2, and transition points)
// //     Eigen::Vector2d c1, c2, transition_pt;
// //     calculateParkingPoints(R, goal, start, c1, c2, transition_pt);

// //     // Convert transition point into a PoseStamped
// //     geometry_msgs::msg::PoseStamped transition_pose = vectorToPoseStamped(transition_pt, c1, global_frame_, node_, false);
    

// //     // Discretize the circle arc from start -> transition around c1
// //     auto first_arc = discretizeCircle(c1, start, transition_pose, R, true);
// //     global_path.poses.insert(global_path.poses.end(), first_arc.begin(), first_arc.end());

// //     // Discretize the circle arc from transition -> goal around c2
// //     auto second_arc = discretizeCircle(c2, transition_pose, goal, R, false);
// //     global_path.poses.insert(global_path.poses.end(), second_arc.begin(), second_arc.end());

// //     // Set global path header
// //     global_path.header.stamp = node_->now();
// //     global_path.header.frame_id = global_frame_;

// //     return global_path;
// //   }

// // private:
// //   nav2_util::LifecycleNode::SharedPtr node_;
// //   std::string name_;
// //   std::shared_ptr<tf2_ros::Buffer> tf_;
// //   nav2_costmap_2d::Costmap2D * costmap_;
// //   std::string global_frame_;

// //   geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
// //     tf2::Quaternion quaternion;
// //     quaternion.setRPY(0, 0, yaw); // Roll = 0, Pitch = 0, Yaw = calculated yaw
// //     geometry_msgs::msg::Quaternion q;
// //     q = tf2::toMsg(quaternion);
// //     return q;
// //   }

// //   // Helper to calculate parking points (C1, C2, and transition)
// //   void calculateParkingPoints(
// //     double R, const geometry_msgs::msg::PoseStamped & goal,
// //     const geometry_msgs::msg::PoseStamped & start,
// //     Eigen::Vector2d & c1, Eigen::Vector2d & c2, Eigen::Vector2d & transition_pt)
// //   {
// //     c2 = Eigen::Vector2d(goal.pose.position.x + R, goal.pose.position.y);

// //     double xt = (start.pose.position.x + goal.pose.position.x) / 2;
// //     double yt = c2.y() - std::sqrt(R * R - (xt - c2.x()) * (xt - c2.x()));
// //     transition_pt = Eigen::Vector2d(xt, yt);

// //     c1 = Eigen::Vector2d(start.pose.position.x - R, 2 * transition_pt.y() - c2.y());
// //     RCLCPP_INFO(node_->get_logger(), "C1: [%f, %f], C2: [%f, %f], Transition: [%f, %f]",
// //       c1.x(), c1.y(), c2.x(), c2.y(), transition_pt.x(), transition_pt.y());
// //   }

// //   // Helper function to convert Eigen::Vector2d to PoseStamped with orientation
// //   geometry_msgs::msg::PoseStamped vectorToPoseStamped(
// //     const Eigen::Vector2d & point, const Eigen::Vector2d & center, const std::string & frame_id, 
// //     rclcpp_lifecycle::LifecycleNode::SharedPtr node, bool clockwise)
// //   {
// //     double car_offset = 0.8935655403137206;
// //     geometry_msgs::msg::PoseStamped pose;
// //     pose.header.stamp = node->now();
// //     pose.header.frame_id = frame_id;
    
// //     // Calculate the yaw angle for tangent orientation
// //     double yaw = std::atan2(point.y() - center.y(), point.x() - center.x());

// //     // Adjust yaw by +90 degrees for counterclockwise, -90 degrees for clockwise
// //     if (clockwise) {
// //       yaw -= M_PI_2;
// //     } else {
// //       yaw += M_PI_2;
// //     }

// //     // Set orientation using calculated yaw
// //     pose.pose.orientation = yawToQuaternion(yaw);

// //     // Calculate the tangent vector (perpendicular to the radius vector)
// //     Eigen::Vector2d tangent_vector;
// //     tangent_vector.x() = std::cos(yaw); // Tangent is perpendicular to the radius vector
// //     tangent_vector.y() = -std::sin(yaw);  // 90 degrees to the radius
    
// //     // Offset the car's center by car_offset along the tangent vector
// //     Eigen::Vector2d car_center = point + car_offset * tangent_vector;
    
// //     // Set the position with the offset
// //     pose.pose.position.x = car_center.x();
// //     pose.pose.position.y = car_center.y();
// //     pose.pose.position.z = 0.0;

// //     return pose;
// //   }

// //   // Discretize a circle between two points with a given radius and tangent orientation
// //   std::vector<geometry_msgs::msg::PoseStamped> discretizeCircle(
// //     const Eigen::Vector2d & center, const geometry_msgs::msg::PoseStamped & start,
// //     const geometry_msgs::msg::PoseStamped & end, double radius, bool clockwise)
// //   {
// //     std::vector<geometry_msgs::msg::PoseStamped> path;

// //     // Calculate the angles between start, center, and end
// //     double start_angle = atan2(start.pose.position.x - center.x(), start.pose.position.y - center.y());
// //     double end_angle = atan2(end.pose.position.x - center.x(), end.pose.position.y - center.y());

// //     // Ensure the angles are continuous (adjust for clockwise/counterclockwise)
// //     if (clockwise && end_angle > start_angle) {
// //       end_angle -= 2 * M_PI;
// //     } else if (!clockwise && end_angle < start_angle) {
// //       end_angle += 2 * M_PI;
// //     }

// //     // Discretize the arc into small steps
// //     int num_points = std::abs((end_angle - start_angle) / 0.1);  // Adjust step size as needed
// //     double angle_step = (end_angle - start_angle) / num_points;

// //     for (int i = 0; i <= num_points; ++i) {
// //       double theta = start_angle + i * angle_step;
// //       Eigen::Vector2d point_on_circle;
// //       point_on_circle.x() = center.x() + radius * std::sin(theta);
// //       point_on_circle.y() = center.y() + radius * std::cos(theta);

// //       // Convert the Eigen vector to PoseStamped with tangent orientation and add to the path
// //       path.push_back(vectorToPoseStamped(point_on_circle, center, global_frame_, node_, !clockwise));
// //     }

// //     return path;
// //   }

// // };

// // }  // namespace custom_parking_planner

// // // Register this planner as a plugin in the nav2_core library
// // PLUGINLIB_EXPORT_CLASS(custom_parking_planner::CustomParkingPlanner, nav2_core::GlobalPlanner)


// #include <cmath>
// #include <string>
// #include <memory>
// #include <vector>
// #include <Eigen/Core>  // Include Eigen for vector calculations
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <nav_msgs/msg/path.hpp>
// #include <pluginlib/class_list_macros.hpp>
// #include "rclcpp/rclcpp.hpp"
// #include "tf2_ros/buffer.h"
// #include "nav2_core/global_planner.hpp"
// #include "nav2_util/node_utils.hpp"
// #include "nav2_costmap_2d/costmap_2d_ros.hpp"
// #include "nav2_costmap_2d/costmap_2d.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include <Eigen/Dense>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/convert.h>
// #include "std_srvs/srv/trigger.hpp" // Include the service for triggering path generation

// namespace custom_parking_planner
// {

// class CustomParkingPlanner : public nav2_core::GlobalPlanner
// {
// public:
//   void configure(
//     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
//     std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
//     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
//   {
//     node_ = parent.lock();
//     name_ = name;
//     tf_ = tf;
//     costmap_ = costmap_ros->getCostmap();
//     global_frame_ = costmap_ros->getGlobalFrameID();

//     // Logger for configuration
//     RCLCPP_INFO(node_->get_logger(), "Configuring plugin %s of type CustomParkingPlanner", name_.c_str());

//     // Initialize service client for Python service
//     client_ = node_->create_client<std_srvs::srv::Trigger>("generate_parking_path");
//   }

//   void cleanup() override
//   {
//     RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s of type CustomParkingPlanner", name_.c_str());
//   }

//   void activate() override
//   {
//     RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type CustomParkingPlanner", name_.c_str());
//   }

//   void deactivate() override
//   {
//     RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type CustomParkingPlanner", name_.c_str());
//   }

//   nav_msgs::msg::Path createPlan(
//     const geometry_msgs::msg::PoseStamped & start,
//     const geometry_msgs::msg::PoseStamped & goal) override
//   {
//     nav_msgs::msg::Path global_path;

//     // Validate frames
//     if (start.header.frame_id != global_frame_) {
//       RCLCPP_ERROR(node_->get_logger(), "Planner only accepts start position from %s frame", global_frame_.c_str());
//       return global_path;
//     }

//     if (goal.header.frame_id != global_frame_) {
//       RCLCPP_ERROR(node_->get_logger(), "Planner only accepts goal position from %s frame", global_frame_.c_str());
//       return global_path;
//     }

//     // Call the Python service to generate the parking path
//     auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    
//     if (!client_->wait_for_service(std::chrono::seconds(2))) {
//       RCLCPP_ERROR(node_->get_logger(), "Service not available!");
//       return global_path;
//     }

//     auto future = client_->async_send_request(request);
//     try {
//       auto response = future.get();
//       if (response->success) {
//         RCLCPP_INFO(node_->get_logger(), "Successfully generated path from Python service");

//         // Check if the path contains valid poses
//         if (global_path.poses.empty()) {
//           RCLCPP_ERROR(node_->get_logger(), "Generated path is empty!");
//         } else {
//           RCLCPP_INFO(node_->get_logger(), "Generated path has %lu poses.", global_path.poses.size());
//         }
//       } else {
//         RCLCPP_ERROR(node_->get_logger(), "Failed to generate path using the Python service");
//       }
//     } catch (const std::exception &e) {
//       RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s", e.what());
//     }

//     auto future = client_->async_send_request(request);
//     try {
//       auto response = future.get();
//       if (response->success) {
//         RCLCPP_INFO(node_->get_logger(), "Successfully generated path from Python service");

//         // Check if the path returned is valid
//         if (response->path.empty()) {
//           RCLCPP_ERROR(node_->get_logger(), "Received an empty path from Python service!");
//         } else {
//           RCLCPP_INFO(node_->get_logger(), "Received path with %lu poses.", response->path.size());
//         }
//       } else {    auto future = client_->async_send_request(request);
//     try {
//       auto response = future.get();
//       if (response->success) {
//         RCLCPP_INFO(node_->get_logger(), "Successfully generated path from Python service");

//         // Check if the path returned is valid
//         if (response->path.empty()) {
//           RCLCPP_ERROR(node_->get_logger(), "Received an empty path from Python service!");
//         } else {
//           RCLCPP_INFO(node_->get_logger(), "Received path with %lu poses.", response->path.size());
//         }
//       } else {
//         RCLCPP_ERROR(node_->get_logger(), "Failed to generate path using Python service");
//       }
//     } catch (const std::exception &e) {
//       RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s", e.what());
//     }
//         RCLCPP_ERROR(node_->get_logger(), "Failed to generate path using Python service");
//       }
//     } catch (const std::exception &e) {
//       RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s", e.what());
//     }



//     // Populate the path with some sample poses for demonstration (replace this with actual path generation logic)
//     global_path.header.stamp = node_->now();
//     global_path.header.frame_id = global_frame_;

//     // You can add more logic here to process the received path from the Python service

//     return global_path;
//   }

// private:
//   nav2_util::LifecycleNode::SharedPtr node_;
//   std::string name_;
//   std::shared_ptr<tf2_ros::Buffer> tf_;
//   nav2_costmap_2d::Costmap2D * costmap_;
//   std::string global_frame_;

//   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;  // Client to call the Python path generation service
// };

// }  // namespace custom_parking_planner

// // Register this planner as a plugin in the nav2_core library
// PLUGINLIB_EXPORT_CLASS(custom_parking_planner::CustomParkingPlanner, nav2_core::GlobalPlanner)

// #include "rclcpp/rclcpp.hpp"
// #include "nav2_core/global_planner.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include <pybind11/embed.h>  // pybind11 header
// #include <pybind11/pybind11.h> 
// #include <pybind11/stl.h>
// #include <dlfcn.h>

// void *handle = dlopen("libpython3.5m.so.1.0", RTLD_LAZY | RTLD_GLOBAL);

// namespace py = pybind11;

// namespace custom_parking_planner
// {
// class CustomParkingPlanner : public nav2_core::GlobalPlanner
// {
// public:
//     CustomParkingPlanner() {
//         // Initialize Python interpreter
//         py::initialize_interpreter();
//         try {
//             // Load your Python module here
//             py::module sys = py::module::import("sys");
//             std::string module_path = "/home/gmsie/Documentos/Workspaces/ws_rodrigo/install/custom_parking_planner/lib/custom_parking_planner"; // Update this to your path
//             sys.attr("path").cast<py::list>().append(module_path);
//             path_generator = py::module::import("parking_path");  // Use py::module
//         } catch (py::error_already_set &e) {
//             RCLCPP_ERROR(rclcpp::get_logger("custom_global_planner"), "Error loading Python module: %s", e.what());
//         }
//     }

//     ~CustomParkingPlanner() {
//         py::finalize_interpreter();  // Finalize Python interpreter
//     }

//     void configure(
//       const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
//       std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
//       std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
//     {
//       node_ = parent.lock();
//       name_ = name;
//       tf_ = tf;
//       costmap_ = costmap_ros->getCostmap();
//       global_frame_ = costmap_ros->getGlobalFrameID();

//       // Logger for configuration
//       RCLCPP_INFO(node_->get_logger(), "Configuring plugin %s of type CustomParkingPlanner", name_.c_str());
//     }

//     void cleanup() override
//     {
//       RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s of type CustomParkingPlanner", name_.c_str());
//     }

//     void activate() override
//     {
//       RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type CustomParkingPlanner", name_.c_str());
//     }

//     void deactivate() override
//     {
//       RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type CustomParkingPlanner", name_.c_str());
//     }

//     nav_msgs::msg::Path createPlan(
//         const geometry_msgs::msg::PoseStamped &start,
//         const geometry_msgs::msg::PoseStamped &goal) override
//     {
//         nav_msgs::msg::Path path;
//         try {
//             // Call the Python function to generate the path
//             py::object py_path = path_generator.attr("generate_path")(start, goal);

//             // Convert the Python path to ROS2 Path message
//             for (auto &point : py_path) {
//                 geometry_msgs::msg::PoseStamped pose;
//                 pose.pose.position.x = point["x"].cast<double>();
//                 pose.pose.position.y = point["y"].cast<double>();
//                 path.poses.push_back(pose);
//             }

//         } catch (py::error_already_set &e) {
//             RCLCPP_ERROR(rclcpp::get_logger("custom_global_planner"), "Error in Python path generation: %s", e.what());
//         }
//         return path;
//     }

// private:
//     py::module path_generator;  // Use py::module here
//     nav2_util::LifecycleNode::SharedPtr node_;
//     std::string name_;
//     std::shared_ptr<tf2_ros::Buffer> tf_;
//     nav2_costmap_2d::Costmap2D * costmap_;
//     std::string global_frame_;
// };
// }

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(custom_parking_planner::CustomParkingPlanner, nav2_core::GlobalPlanner)


#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <vector>   
#include <tf2/LinearMath/Quaternion.h>

namespace custom_parking_planner
{

class CustomParkingPlanner : public nav2_core::GlobalPlanner
{
public:
    CustomParkingPlanner() = default;
    ~CustomParkingPlanner() = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
    {
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        RCLCPP_INFO(node_->get_logger(), "Configuring plugin %s of type CustomParkingPlanner", name_.c_str());
    }

    void cleanup() override {}
    void activate() override {}
    void deactivate() override {}

    // Create Plan function where we implement the parking trajectory logic
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) override
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_;
        path.header.stamp = rclcpp::Clock().now();

        double R = 5; //3.51; // The radius for the circles
        std::vector<geometry_msgs::msg::PoseStamped> full_path;

        // Calculate the centers and transition point
        auto [c1, c2, transition_pt] = calculateParkingPoints(R, start.pose.position, goal.pose.position);

        // Number of discretization points
        int num_points_straight = 20;  // Number of points for the straight segment
        int num_points_arc = 500;      // Number of points for the arcs

        // Generate straight line from start to c1's y position (only move along the x-axis)
        geometry_msgs::msg::Point straight_target;
        straight_target.x = start.pose.position.x;  // x remains the same
        straight_target.y = c1.y;                   // y is aligned with c1

        auto straight_line = generateStraightLine(start.pose.position, straight_target, num_points_straight);

        // Discretize the first arc (counterclockwise)
        auto arc1 = discretizeArc(c1, straight_target, transition_pt, R, num_points_arc, false); 

        // Discretize the second arc (clockwise)
        auto arc2 = discretizeArc(c2, transition_pt, goal.pose.position, R, num_points_arc, true); 

        // Combine the straight line and arcs to form the full path
        full_path.insert(full_path.end(), straight_line.begin(), straight_line.end());
        full_path.insert(full_path.end(), arc1.begin(), arc1.end());
        full_path.insert(full_path.end(), arc2.begin(), arc2.end());

        // Ensure the final pose's orientation matches the start pose's orientation
        if (!full_path.empty()) {
            full_path.back().pose.orientation = start.pose.orientation;
        }

        path.poses = full_path;

        return path;
    }


private:

    // Calculate centers of the circles and transition point
    std::tuple<geometry_msgs::msg::Point, geometry_msgs::msg::Point, geometry_msgs::msg::Point>
    calculateParkingPoints(double R, const geometry_msgs::msg::Point &start, const geometry_msgs::msg::Point &goal)
    {
        geometry_msgs::msg::Point c1, c2, transition_pt;

        // Circle center c2 is based on the goal
        c2.x = goal.x + R;
        c2.y = goal.y;

        // Transition point calculation (tangent point of both circles)
        double xt = (start.x + goal.x) / 2;
        double yt = c2.y - sqrt(R * R - pow(xt - c2.x, 2));
        transition_pt.x = xt;
        transition_pt.y = yt;

        // Circle center c1 is based on the start and transition point
        c1.x = start.x - R;
        c1.y = 2 * transition_pt.y - c2.y;

        // Log for debug purposes
        RCLCPP_INFO(node_->get_logger(), "C1: (%f, %f), C2: (%f, %f), Transition: (%f, %f)",
                    c1.x, c1.y, c2.x, c2.y, transition_pt.x, transition_pt.y);

        return std::make_tuple(c1, c2, transition_pt);
    }

    std::vector<geometry_msgs::msg::PoseStamped> generateStraightLine(
        const geometry_msgs::msg::Point &start,
        const geometry_msgs::msg::Point &target,
        int num_points)
    {
        std::vector<geometry_msgs::msg::PoseStamped> line;
        
        // Calculate the distance between the start and target along the x-axis
        double step_x = (target.x - start.x) / num_points;

        for (int i = 0; i <= num_points; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = start.x + i * step_x;
            pose.pose.position.y = start.y;  // Y remains constant since we're moving along the x-axis
            pose.pose.position.z = 0.0;      // Z is constant (ground level)

            // Set orientation (yaw = 0 for moving straight along x-axis)
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = -0.7;//q.z();
            pose.pose.orientation.w = 0.7;//q.w();

            line.push_back(pose);
        }

        return line;
    }



    // Discretize an arc between two points on a circle with a given radius
    double calculateYaw(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
    {
        return atan2(p2.y - p1.y, p2.x - p1.x);
    }

    std::vector<geometry_msgs::msg::PoseStamped> discretizeArc(
        const geometry_msgs::msg::Point &center,
        const geometry_msgs::msg::Point &start,
        const geometry_msgs::msg::Point &end,
        double radius, int num_points,
        bool clockwise = false)
    {
        std::vector<geometry_msgs::msg::PoseStamped> arc;

        // Calculate the start and end angles (in radians)
        double start_angle = atan2(start.y - center.y, start.x - center.x);
        double end_angle = atan2(end.y - center.y, end.x - center.x);

        // Handle clockwise motion
        if (clockwise) {
            if (start_angle < end_angle) {
                start_angle += 2 * M_PI;
            }
        } else {
            if (end_angle < start_angle) {
                end_angle += 2 * M_PI;
            }
        }

        geometry_msgs::msg::PoseStamped last_pose;

        // Discretize the arc
        for (int i = 0; i <= num_points; ++i) {
            double t = static_cast<double>(i) / num_points;
            double angle = start_angle + t * (end_angle - start_angle);

            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = center.x + radius * cos(angle);
            pose.pose.position.y = center.y + radius * sin(angle);

            // Calculate yaw based on the previous point (or forward direction if first point)
            if (i > 0) {
                double yaw = calculateYaw(last_pose.pose.position, pose.pose.position);
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                pose.pose.orientation.x = q.x();
                pose.pose.orientation.y = q.y();
                pose.pose.orientation.z = q.z();
                pose.pose.orientation.w = q.w();
            }

            arc.push_back(pose);
            last_pose = pose;
        }

        return arc;
    }




    nav2_util::LifecycleNode::SharedPtr node_;
    std::string name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_costmap_2d::Costmap2D *costmap_;
    std::string global_frame_;
};

}

// Export the plugin
PLUGINLIB_EXPORT_CLASS(custom_parking_planner::CustomParkingPlanner, nav2_core::GlobalPlanner)
