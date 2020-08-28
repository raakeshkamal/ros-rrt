/**
**  Map Server ROS Node
**/
#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[])
{
    // This must be called before anything else ROS-related
    ros::init(argc, argv, "map_server");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Don't exit the program.
    ros::spin();
}
