#include <depth_detector_lib.h>

constexpr int CENTER_PLANT = 0;
constexpr int CENTER_DITCH = 1;
constexpr int CENTER_LUMP = 2;

using namespace std;

DepthDetectorNode::DepthDetectorNode()
{
  // topic names
  string depth_img_topic = "/camera/aligned_depth_to_color/image_raw";
  string depth_camera_info_topic = "/camera/aligned_depth_to_color/camera_info";
  string bounding_boxes_topic = "/darknet_ros/bounding_boxes";
  string norm_depth_img_topic = "/normalized_depth_image";
  // advertise
  pub_speed_flag = node_handle.advertise<std_msgs::UInt8>("speed_flag", 1);
  pub_normalized_depth_img = node_handle.advertise<sensor_msgs::Image>(norm_depth_img_topic,1);
  // subscribe
  sub_depth_img = node_handle.subscribe(
    depth_img_topic, 1, &DepthDetectorNode::depth_img_callback, this);
  sub_bounding_boxes = node_handle.subscribe(
    bounding_boxes_topic, 1, &DepthDetectorNode::bounding_boxes_callback, this);
}
DepthDetectorNode::~DepthDetectorNode() {}
void
DepthDetectorNode::depth_img_callback(const sensor_msgs::ImageConstPtr& depth_img)
{
  // ROS_INFO("new image");
  // convert message to CvImagePtr
  try {
    this->depth_ptr = cv_bridge::toCvCopy(depth_img);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  assert(this->depth_ptr->image.type() == CV_16UC1);

  cv_bridge::CvImage normalized(std_msgs::Header(),"mono8",this->depth_ptr->image/255);
  pub_normalized_depth_img.publish(normalized.toImageMsg());

}
void
DepthDetectorNode::bounding_boxes_callback(
  darknet_ros_msgs::BoundingBoxes bounding_boxes)
{
  ROS_INFO("new boxes");
  float min_distance = -1;
  for (auto box : bounding_boxes.bounding_boxes) {
    if(box.Class == "person"){
    uint32_t width = box.ymax - box.ymin;
    uint32_t height = box.xmax - box.xmin;
    uint32_t xcenter = (box.xmax + box.xmin) / 2;
    uint32_t ycenter = (box.ymax + box.ymin) / 2;
    uint32_t xmin = (xcenter - 3) >= 0 ? (xcenter - 3) : 0;
    uint32_t xmax = (xmin + 6) < box.xmin + width ? (xmin + 6) : box.xmin + width;
    uint32_t ymin = (ycenter - 3) >= 0 ? (ycenter - 3) : 0;
    uint32_t ymax = (ymin + 6) < box.ymin + height ? (ymin + 6) : box.ymin + height;
    float distance = 0;
    uint32_t n_pxls = 0;
    ROS_INFO_STREAM(xmin << " " << xmax << " " << ymin << " " << ymax);
    for (int x = xmin; x <= xmax; x++) {
      for (int y = ymin; y <= ymax; y++) {
        // ROS_INFO_STREAM("depth at : " << x <<" " << y << " is : " << this->depth_ptr->image.at<uint16_t> (x, y));
        uint16_t d = this->depth_ptr->image.at<uint16_t> (x, y);

        if(!isnan(d)){
          distance += d;
          n_pxls++;
        }

      }
    }
    if(n_pxls > 0){
      distance = 0.001 * distance / n_pxls;
      distance = 0.001 * distance / n_pxls;
      ROS_INFO_STREAM("distance is:" << distance);
      if (min_distance < 0)
        min_distance = distance;
      else if (distance < min_distance)
        min_distance = distance;
    }
    }
  }
  ROS_INFO_STREAM("min distance is:" << min_distance);
  std_msgs::UInt8 msg;
  if (min_distance > 0 && min_distance < 3.0) {
    msg.data = 2;
  } else if (min_distance < 6.0) {
    msg.data = 1;
  } else {
    msg.data = 0;
  }
  pub_speed_flag.publish(msg);
}