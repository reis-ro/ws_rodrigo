// #include <memory>
// #include <string>
// #include <cstring>
// #include <opencv2/opencv.hpp>
// #include <iostream>
// #include <stdio.h>
// #include <opencv2/imgproc.hpp>
// #include "rclcpp/rclcpp.hpp"
// #include <cv_bridge/cv_bridge.h>
// #include "sensor_msgs/msg/image.hpp"
// #include "sensor_msgs/msg/camera_info.hpp"
// #include "geometry_msgs/msg/pose_array.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "geometry_msgs/msg/pose.hpp"
// #include "image_transport/image_transport.hpp"
// #include "std_msgs/msg/header.hpp"
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// using std::placeholders::_1;
// using std::placeholders::_2;
// using std::placeholders::_3;
// using std::placeholders::_4;

// using namespace std;
// using namespace cv;

// static geometry_msgs::msg::PoseArray::SharedPtr detections_msg;

// class ConePoseEstimator : public rclcpp::Node
// {
// public:
//   ConePoseEstimator()
//       : Node("cone_pose_estimator"), buffer_(this->get_clock()), listener_(buffer_)
//   {
//     left_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_poses/left", 1); 
//     front_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_poses/front", 1); 
//     right_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_poses/right", 1); 
//     back_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_poses/back", 1); 
//     base_link_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_poses/base_link/all", 1); 

//     detections_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("/camera/cone_centers", 1, std::bind(&ConePoseEstimator::detections_callback, this, _1));

//     dist_thresh = 0.05;

//     RCLCPP_INFO(this->get_logger(), "Ready to receive images and detections!!");
//   }


//   void spin(){
//     if(  detections_msg != nullptr
//       // && front_img_msg != nullptr 
//       // && right_img_msg != nullptr 
//       // && left_img_msg != nullptr 
//       // && back_img_msg != nullptr 
//       ){
//       estimate_poses();

//       reset_msgs();
//     }
//   }

// private:
//   void reset_msgs(){
//     detections_msg = nullptr;
//   }

//   void estimate_poses()
//   {
//     RCLCPP_INFO(this->get_logger(), "I heard: bananas");
//     int h = 480;
//     int w = 640;
    
//     float f = 280;
//     float cone_height = 0.07; //m
//     float z_offset = 0.035;
//     float gain = 3.157;

//     geometry_msgs::msg::PoseArray left_detections;
//     geometry_msgs::msg::PoseArray front_detections;
//     geometry_msgs::msg::PoseArray right_detections;
//     geometry_msgs::msg::PoseArray back_detections;

//     geometry_msgs::msg::PoseArray all_detections;

//     for(geometry_msgs::msg::Pose pose: detections_msg->poses) {
//       RCLCPP_INFO(this->get_logger(), "Pose");
      
//       int image_row = pose.position.y / h;
//       int image_column = pose.position.x / w;
//       pose.position.x = (float)((int)pose.position.x % (int)w);
//       pose.position.y = (float)((int)pose.position.y % (int)h);

//       //align for Rviz visualization
//       pose.orientation.w = 0.707;
//       pose.orientation.z = -0.707;

//       pose.position.x -= w/2;
//       pose.position.y -= h/2;

//       float z = (f * (cone_height/2))/ (pose.position.y);
//       pose.position.x = gain * (pose.position.x * z)/ (f);
//       // pose.position.y = 0.137; //always on ground
//       // pose.position.z = z * gain + z_offset;
//       pose.position.y = z * gain + z_offset; 
//       pose.position.z = 0.137; //always on ground

//       geometry_msgs::msg::PoseStamped pose_from_cam;
//       // pose_from_cam.header.stamp = rclcpp::Node::now();
//       pose_from_cam.header.stamp = rclcpp::timestamp:time();
//       pose_from_cam.pose = pose;

//       switch (image_row)
//       {
//       case 0:
//         switch (image_column)
//         {
//           case 0:
//             // pose_from_cam.header.frame_id = "left_camera";
//             pose_from_cam.header.frame_id = "ego_vehicle";
//             left_detections.poses.push_back(pose);
//             break;

//           case 1:
//             pose_from_cam.header.frame_id = "front_camera";
//             front_detections.poses.push_back(pose);
//             break;
//         }
//         break;

//       case 1:
//         switch (image_column)
//         {
//           case 0:
//             pose_from_cam.header.frame_id = "right_camera";
//             right_detections.poses.push_back(pose);
//             break;
          
//           case 1:
//             pose_from_cam.header.frame_id = "back_camera";
//             back_detections.poses.push_back(pose);
//             break;
//         }
//         break;
      
//       default:
//         break;
//       }
//       geometry_msgs::msg::PoseStamped pose_from_base = buffer_.transform(pose_from_cam, "base_link");
//       all_detections.poses.push_back(pose_from_base.pose);
//     }

//     left_detections.header.stamp = rclcpp::Node::now(); // timestamp of creation of the msg
//     left_detections.header.frame_id = "left_camera";
//     left_det_pub->publish(left_detections);

//     front_detections.header.stamp = rclcpp::Node::now(); // timestamp of creation of the msg
//     front_detections.header.frame_id = "front_camera";
//     front_det_pub->publish(front_detections);

//     right_detections.header.stamp = rclcpp::Node::now(); // timestamp of creation of the msg
//     right_detections.header.frame_id = "right_camera";
//     right_det_pub->publish(right_detections);

//     back_detections.header.stamp = rclcpp::Node::now(); // timestamp of creation of the msg
//     back_detections.header.frame_id = "back_camera";
//     back_det_pub->publish(back_detections);

//     // bool found;
//     // for(auto old_pose: previous_all_detections.poses){
//     //   found = false;
//     //   for(int i = 0; i < size(all_detections.poses); i++){
//     //     auto new_pose = all_detections.poses[i];

//     //     float dist = sqrt(pow(old_pose.position.x - new_pose.position.x, 2) + pow(old_pose.position.y - new_pose.position.y, 2));
        
//     //     if(dist <= dist_thresh){
//     //       all_detections.poses[i].position.x = (new_pose.position.x + old_pose.position.x)/2;
//     //       all_detections.poses[i].position.y = (new_pose.position.y + old_pose.position.y)/2;
          
//     //       found = true;
//     //       break;
//     //     }
//     //   }

//     //   if (!found){
//     //     all_detections.poses.push_back(old_pose);
//     //   }
//     // }

//     all_detections.header.stamp = rclcpp::Node::now(); 
//     all_detections.header.frame_id = "base_link"; 
//     base_link_det_pub->publish(all_detections);

//     previous_all_detections = all_detections;

//     RCLCPP_INFO(this->get_logger(), "--------------------------------------");
//   }

//   void detections_callback(const geometry_msgs::msg::PoseArray::SharedPtr detections) const 
//   {
//     detections_msg = detections;
//   }

//   rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr detections_sub;

//   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr left_det_pub;
//   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr front_det_pub;
//   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr right_det_pub;
//   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr back_det_pub;
//   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr base_link_det_pub;
//   geometry_msgs::msg::PoseArray previous_all_detections;

//   tf2_ros::Buffer buffer_;
//   tf2_ros::TransformListener listener_;

//   float dist_thresh;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);

//   std::shared_ptr<ConePoseEstimator> node = std::make_shared<ConePoseEstimator>();

//   rclcpp::Rate rate(60);
//   while(rclcpp::ok()){
//     node->spin();
//     rclcpp::spin_some(node);
//     rate.sleep();
//   }

//   rclcpp::shutdown();

//   return 0;
// }


// ###########################################################################################################

// #include <memory>
// #include <string>
// #include <cstring>
// #include <opencv2/opencv.hpp>
// #include <iostream>
// #include <stdio.h>
// #include <opencv2/imgproc.hpp>
// #include "rclcpp/rclcpp.hpp"
// #include <cv_bridge/cv_bridge.h>
// #include "sensor_msgs/msg/image.hpp"
// #include "sensor_msgs/msg/camera_info.hpp"
// #include "geometry_msgs/msg/pose_array.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "geometry_msgs/msg/pose.hpp"
// #include "image_transport/image_transport.hpp"
// #include "std_msgs/msg/header.hpp"
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// using std::placeholders::_1;

// static geometry_msgs::msg::PoseArray::SharedPtr detections_msg;

// class ConePoseEstimator : public rclcpp::Node
// {
// public:
//   ConePoseEstimator()
//       : Node("cone_pose_estimator"), buffer_(this->get_clock()), listener_(buffer_)
//   {
//     // Enable simulation time
//     // this->declare_parameter("use_sim_time", rclcpp::ParameterValue(true));

//     // Publisher for front camera cone poses in base_link frame
//     base_link_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_poses/base_link/front", 1); 

//     // Subscription for cone detections from front camera
//     detections_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("/camera/cone_centers", 1, 
//                                                                               std::bind(&ConePoseEstimator::detections_callback, this, _1));

//     dist_thresh = 0.05;

//     RCLCPP_INFO(this->get_logger(), "Ready to receive images and detections for front camera!!");
//   }

//   void spin(){
//     if(detections_msg != nullptr){
//       estimate_poses();
//       reset_msgs();
//     }
//   }

// private:
//   void reset_msgs(){
//     detections_msg = nullptr;
//   }

//   void estimate_poses()
//   {
//     RCLCPP_INFO(this->get_logger(), "Estimating cone poses...");

//     int h = 480;
//     int w = 640;
    
//     float f = 280;
//     float cone_height = 0.07; // meters
//     float z_offset = 0.035;
//     float gain = 3.157;

//     geometry_msgs::msg::PoseArray all_detections;

//     for(auto &pose: detections_msg->poses) {
//       // int image_row = pose.position.y / h;
//       // int image_column = pose.position.x / w;
//       pose.position.x = static_cast<float>(static_cast<int>(pose.position.x) % w);
//       pose.position.y = static_cast<float>(static_cast<int>(pose.position.y) % h);

//       // Align for Rviz visualization
//       pose.orientation.w = 0.707;
//       pose.orientation.z = -0.707;

//       // Convert from image coordinates to 3D coordinates
//       pose.position.x -= w / 2;
//       pose.position.y -= h / 2;

//       float z = (f * (cone_height / 2)) / pose.position.y;
//       pose.position.x = gain * (pose.position.x * z) / f;
//       pose.position.y = 0.137; // on ground
//       pose.position.z = z * gain + z_offset;

//       geometry_msgs::msg::PoseStamped pose_from_cam;
//       pose_from_cam.header.stamp = this->get_clock()->now();
//       pose_from_cam.header.frame_id = "base_link";
//       pose_from_cam.pose = pose;

//       // Transform to base_link frame
//       geometry_msgs::msg::PoseStamped pose_from_base = buffer_.transform(pose_from_cam, "base_link");
//       all_detections.poses.push_back(pose_from_base.pose);
//     }

//     all_detections.header.stamp = this->get_clock()->now(); 
//     all_detections.header.frame_id = "base_link";
//     base_link_det_pub->publish(all_detections);

//     previous_all_detections = all_detections;
//     RCLCPP_INFO(this->get_logger(), "Published cone poses in base_link frame.");
//   }

//   void detections_callback(const geometry_msgs::msg::PoseArray::SharedPtr detections) const 
//   {
//     detections_msg = detections;
//   }

//   rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr detections_sub;
//   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr base_link_det_pub;
//   geometry_msgs::msg::PoseArray previous_all_detections;

//   tf2_ros::Buffer buffer_;
//   tf2_ros::TransformListener listener_;

//   float dist_thresh;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);

//   auto node = std::make_shared<ConePoseEstimator>();

//   node->set_parameter(rclcpp::Parameter("use_sim_time", true));

//   rclcpp::Rate rate(60);
//   while(rclcpp::ok()){
//     node->spin();
//     rclcpp::spin_some(node);
//     rate.sleep();
//   }

//   rclcpp::shutdown();
//   return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ConePoseEstimator : public rclcpp::Node
{
public:
    ConePoseEstimator()
        : Node("cone_pose_estimator"), buffer_(this->get_clock()), listener_(buffer_)
    {
        // Publishers and subscribers
        base_link_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_poses/base_link", 1);
        detections_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("/camera/cone_centers", 1, 
                             std::bind(&ConePoseEstimator::detections_callback, this, std::placeholders::_1));
        camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", 10, 
                             std::bind(&ConePoseEstimator::camera_info_callback, this, std::placeholders::_1));
        depth_image_sub = this->create_subscription<sensor_msgs::msg::Image>("/carla/ego_vehicle/depth_front/image", 10, 
                             std::bind(&ConePoseEstimator::depth_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Cone Pose Estimator node initialized");
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx = msg->k[0];
        fy = msg->k[4];
        cx = msg->k[2];
        cy = msg->k[5];
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert depth image to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            depth_image_ = cv_ptr->image;
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void detections_callback(const geometry_msgs::msg::PoseArray::SharedPtr detections)
    {
        if (depth_image_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Depth image not received yet");
            return;
        }

        geometry_msgs::msg::PoseArray all_detections;
        all_detections.header.frame_id = "base_link";

        for (auto& pose : detections->poses) {
            // Convert image coordinates to camera coordinates
            double u = pose.position.x;
            double v = pose.position.y;

            // Get depth value at (u, v)
            float depth = depth_image_.at<float>(static_cast<int>(v), static_cast<int>(u));
            if (std::isnan(depth) || depth <= 0) {
                RCLCPP_WARN(this->get_logger(), "Invalid depth value at (%f, %f)", u, v);
                continue;
            }

            // Calculate camera coordinates
            double x_cam = (u - cx) * depth / fx;
            double y_cam = (v - cy) * depth / fy;
            double z_cam = depth;

            // Convert camera coordinates to car's coordinate frame (x forward, y right, z up)
            geometry_msgs::msg::PoseStamped camera_pose;
            camera_pose.header.frame_id = "camera_frame";
            camera_pose.pose.position.x = z_cam;
            camera_pose.pose.position.y = x_cam;
            camera_pose.pose.position.z = -y_cam;

            // Transform to base_link
            geometry_msgs::msg::PoseStamped base_link_pose;
            try {
                buffer_.transform(camera_pose, base_link_pose, "base_link");
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
                continue;
            }

            all_detections.poses.push_back(base_link_pose.pose);
        }

        all_detections.header.stamp = this->get_clock()->now();
        base_link_det_pub->publish(all_detections);
    }

    // Member variables
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr detections_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr base_link_det_pub;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;

    double fx, fy, cx, cy;
    cv::Mat depth_image_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConePoseEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
