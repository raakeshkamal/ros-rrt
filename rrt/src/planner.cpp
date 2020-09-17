#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/console.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>

#include <iostream>
#include <time.h>

#include "rrt.h"

using namespace std;
using namespace Eigen;

RRT *rrt;
bool animate;
/**
 * 
 * @param  {nav_msgs::OccupancyGrid::ConstPtr} msg : 
 * @param  {Vector2i} nearestPos                   : 
 * @param  {Vector2i} newPos                       : 
 * @return {bool}                                  : 
 */
bool isSegmentObstacle(const nav_msgs::OccupancyGrid::ConstPtr &msg, const Vector2i &nearestPos, const Vector2i &newPos)
{
    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;

    int startX, startY, endX, endY;

    //create a box with startPos and newPos
    if (nearestPos.x() < newPos.x())
    {
        startX = nearestPos.x();
        endX = newPos.x();
    }
    else
    {
        endX = nearestPos.x();
        startX = newPos.x();
    }

    if (nearestPos.y() < newPos.y())
    {
        startY = nearestPos.y();
        endY = newPos.y();
    }
    else
    {
        endY = nearestPos.y();
        startY = newPos.y();
    }

    //check if box is occupied
    for (int i = startX; i <= endX; i++)
    {
        for (int j = startY; j <= endY; j++)
        {
            if (msg->data[i + info.width * (info.height - j - 1)] == 0)
                return true;
        }
    }
    return false;
}
/**
 * 
 * @param  {nav_msgs::OccupancyGrid::ConstPtr} msg : 
 */
void findPath(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;

    cout << "Got map of dimensions " << info.width << " x " << info.height << endl;

    //Error handling
    if (rrt->startPos.x() < 0 || rrt->startPos.y() < 0 || rrt->startPos.x() > info.width || rrt->startPos.y() > info.height)
    {
        cout<<"Coordinates of the start point is out of bounds! Please check launch file"<<endl;
        ros::shutdown();
    }
    else if (rrt->endPos.x() < 0 || rrt->endPos.y() < 0 || rrt->endPos.x() > info.width || rrt->endPos.y() > info.height)
    {
        cout<<"Coordinates of the end point is out of bounds! Please check launch file"<<endl;
        ros::shutdown();
    }
    else if (msg->data[rrt->startPos.x() + info.width * (info.height - rrt->startPos.y() - 1)] == 0)
    {
        cout<<"Coordinates of the start point is occupied! Please check launch file"<<endl;
        ros::shutdown();
    }
    else if (msg->data[rrt->endPos.x() + info.width * (info.height - rrt->endPos.y() - 1)] == 0)
    {
        cout<<"Coordinates of the end point is occupied! Please check launch file"<<endl;
        ros::shutdown();
    }
    else if(rrt->step_size < 0)
    {
        cout<<"Step size invalid! Please check launch file"<<endl;
        ros::shutdown();
    }
    else if(rrt->max_iter < 0){
        cout<<"Max iterations invalid! Please check launch file"<<endl;
        ros::shutdown();
    }

    if (!ros::isShuttingDown())
    {
        cv::Mat display(info.width, info.height, CV_8UC3);

        //convert OccupancyGrid to cv::Mat for display
        for (unsigned int x = 0; x < info.width; x++)
            for (unsigned int y = 0; y < info.height; y++)
            {
                cv::Vec3b &color = display.at<cv::Vec3b>(y, x);
                uchar pixel = ((100.0f - msg->data[x + info.width * (info.height - y - 1)]) / 100.0f) * 255;
                color[0] = pixel;
                color[1] = pixel;
                color[2] = pixel;
            }

        //add start Point and end Point
        cv::circle(display, cv::Point(rrt->startPos.x(), rrt->startPos.y()), 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
        cv::circle(display, cv::Point(rrt->endPos.x(), rrt->endPos.y()), 4, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);

        int iterStep = (int)((float)rrt->max_iter) / 10;

        //RRT Algo
        for (int i = 0; i < rrt->max_iter; i++)
        {
            if (i % iterStep == 0)
                cout << (int)(i / ((float)rrt->max_iter) * 100) << "% of iterations done...." << endl;

            //generate Random node
            Node *q = rrt->getRandomNode(info.width, info.height);
            if (q)
            {
                Node *qNearest = rrt->nearest(q->position); //find the nearest node in the tree
                if (rrt->distance(q->position, qNearest->position) > rrt->step_size)
                {
                    Vector2i newPos = rrt->newConfig(q->position, qNearest->position); //adjust random node for step_size
                    if (!isSegmentObstacle(msg, qNearest->position, newPos))           //check for obstacles
                    {
                        Node *qNew = new Node;
                        qNew->position = newPos;

                        rrt->add(qNearest, qNew); //add qNew to tree
                        if (animate)              //draw branch and node in openCV
                        {
                            cv::circle(display, cv::Point(qNew->position.x(), qNew->position.y()), 1,
                                       cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
                            cv::line(display, cv::Point(qNew->position.x(), qNew->position.y()),
                                     cv::Point(qNearest->position.x(), qNearest->position.y()), cv::Scalar(255, 0, 0));
                        }
                    }
                    if (animate)
                    {
                        imshow("Display window", display); // Show our image inside it.
                        cv::waitKey(10);
                    }
                }
            }
            if (rrt->goalReached()) //check if goal Reached
            {
                cout << "Goal reached successfully. Close window & Ctrl-C to exit" << endl;
                Node *q = rrt->lastNode;
                while (q != rrt->root) //display path in openCV
                {
                    cv::line(display, cv::Point(q->position.x(), q->position.y()),
                             cv::Point(q->parent->position.x(), q->parent->position.y()), cv::Scalar(0, 0, 255));
                    q = q->parent;
                }
                break;
            }
        }
        if (!rrt->goalReached()) //failure condition
            cout << "Oops! RRT failed to reach goal. Please increase maximum iterations or try with different step size." << endl;
        cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.
        imshow("Display window", display);                      // Show our image inside it.
        cv::waitKey(0);
    }
}
/**
 * 
 * @param  {int} argc    : 
 * @param  {char**} argv : 
 * @return {int}         : 
 */
int main(int argc, char **argv)
{
    srand(time(0)); //randomize the randomNode function

    ros::init(argc, argv, "planner");
    ros::NodeHandle nh("~");
    Vector2i startPos, endPos;
    int stepSize, maxIter;

    //gather inputs from launch file
    nh.getParam("startX", startPos.x());
    nh.getParam("startY", startPos.y());
    nh.getParam("endX", endPos.x());
    nh.getParam("endY", endPos.y());
    nh.getParam("stepSize", stepSize);
    nh.getParam("maxIter", maxIter);
    nh.getParam("animate", animate);

    rrt = new RRT(startPos, endPos, stepSize, maxIter); //create a RRT class instance

    ros::Subscriber sub = nh.subscribe("/map", 1000, findPath); //subscribe to /map

    // Don't exit the program.
    ros::spin();
    return 0;
}