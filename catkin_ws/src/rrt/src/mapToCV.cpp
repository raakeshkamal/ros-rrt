/**
**  Map Server ROS Node
**/
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void mapConvert(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;

    ROS_INFO("Got map %d %d", info.width, info.height);
    cv::Mat display(500, 500, CV_8UC3);
    for (unsigned int x = 0; x < info.width; x++)
        for (unsigned int y = 0; y < info.height; y++)
        {
            cv::Vec3b &color = display.at<cv::Vec3b>(y, x);
            uchar pixel = ((100.0f - msg->data[x + info.width * (info.height - y - 1)]) / 100.0f) * 255;
            color[0] = pixel;
            color[1] = pixel;
            color[2] = pixel;
        }
    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.
    imshow("Display window", display); // Show our image inside it.
    cv::waitKey(0);
}

int main(int argc, char *argv[])
{
    // This must be called before anything else ROS-related
    ros::init(argc, argv, "mapToCV");

    // Create a ROS node handle
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/map", 1000, mapConvert);

    // Don't exit the program.
    ros::spin();
    return 0;
}
