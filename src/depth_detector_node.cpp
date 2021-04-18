#include <depth_detector_lib.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "depth_detector_node");
    PathFinderNode node;
    ros::Rate loop_rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}