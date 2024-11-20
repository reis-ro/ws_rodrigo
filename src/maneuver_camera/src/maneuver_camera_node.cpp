
#include "maneuver_camera/maneuver_camera_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>

#include <functional>

const std::string img_topics[] = {
  "/carla/ego_vehicle/rgb_front/left/image_rect",
  "/carla/ego_vehicle/rgb_left/left/image_rect",
  "/carla/ego_vehicle/rgb_back/left/image_rect",
  "/carla/ego_vehicle/rgb_right/left/image_rect"
};

const float G[] {
  1., 0., 0.,
  0., 1., 0.,
  0., 0., 1.
};

const float P[] = { 
  400., 0.0, 400., 0.0, 
  0.0, 400., 300., 0.0, 
  0.0, 0.0, 1.0, 0.0
};

using namespace std::chrono_literals;

namespace maneuver_camera{
  ManeuverCameraNode::ManeuverCameraNode(const rclcpp::NodeOptions & options): Node("ManeuverCameraNode", options){
    RCLCPP_INFO_STREAM(this->get_logger(), "ManeuverCameraNode init" );

    for(uint8_t i = 0; i < 4; i++){
      this->imgs_subs_[i] = this->create_subscription<sensor_msgs::msg::Image>(
        img_topics[i],
        3,
        [i, this](const sensor_msgs::msg::Image::SharedPtr img){return this->receive_callback(i, img);}
      );
    }

    this->publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/composite_img/debug", 10);
    this->timer_ = this->create_wall_timer(1s, std::bind(&ManeuverCameraNode::publish_callback, this)); 
  }

  void ManeuverCameraNode::receive_callback(uint8_t num, const sensor_msgs::msg::Image::SharedPtr img){
    RCLCPP_INFO_STREAM(this->get_logger(), "Received image from " << (uint16_t) num << " " << img->header.stamp.sec << "." << img->header.stamp.nanosec);
    this->imgs_[num] = img;
  }

  void ManeuverCameraNode::publish_callback(){

    uint16_t width = this->imgs_[0]->width; 
    uint16_t height = this->imgs_[0]->height; 

    cv::Mat cv_img = cv::Mat::zeros(
      height, 
      width*4, 
      CV_MAKETYPE(CV_8U, 4)
    );

    for(uint8_t i = 0; i < 4; i++){
      cv::Mat ros_img(
        this->imgs_[i]->height, 
        this->imgs_[i]->width, 
        CV_MAKETYPE(CV_8U, 4),
        &this->imgs_[i]->data[0],
        this->imgs_[i]->step
      );

      ros_img.copyTo(cv_img(cv::Rect(width*i, 0, width, height)));
    }

    cv_bridge::CvImage out_msg_builder;
    out_msg_builder.header = this->imgs_[0]->header;
    out_msg_builder.encoding = this->imgs_[0]->encoding;
    out_msg_builder.image = cv_img;

    out_msg_ = out_msg_builder.toImageMsg();
    this->publisher_->publish(*out_msg_);
  }
}



