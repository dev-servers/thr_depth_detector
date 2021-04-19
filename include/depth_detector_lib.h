#pragma once
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <math.h> 


using namespace std;

class DepthDetectorNode {
  public:
    DepthDetectorNode();
    ~DepthDetectorNode();

  private:
    cv_bridge::CvImagePtr depth_ptr;
    ros::NodeHandle node_handle;
    ros::Subscriber sub_depth_img;
    ros::Subscriber sub_bounding_boxes;
    ros::Publisher pub_speed_flag;
    ros::Publisher pub_normalized_depth_img;
    void depth_img_callback(const sensor_msgs::ImageConstPtr &depth_img);
    void bounding_boxes_callback(darknet_ros_msgs::BoundingBoxes bounding_boxes);
};
