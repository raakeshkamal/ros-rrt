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

const int STEP_SIZE = 10;
const int MAX_ITER = 15000;
const bool animate = true;

bool isSegmentObstacle(const nav_msgs::OccupancyGrid::ConstPtr &msg, const Vector2i &nearestPos, const Vector2i &newPos)
{
    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;

    int startX, startY, endX, endY;

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

void findPath(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;

    cout << "Got map of dimensions " << info.width << " x " << info.height << endl;

    RRT *rrt = new RRT;
    rrt->step_size = STEP_SIZE;
    rrt->max_iter = MAX_ITER;

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

    cv::circle(display, cv::Point(rrt->startPos.x(), rrt->startPos.y()), 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
    cv::circle(display, cv::Point(rrt->endPos.x(), rrt->endPos.y()), 4, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);

    int iterStep = (int)((float)rrt->max_iter) / 10;
    for (int i = 0; i < rrt->max_iter; i++)
    {
        if (i % iterStep == 0)
            cout << (int)(i / ((float)rrt->max_iter) * 100) << "% of iterations done...." << endl;
        Node *q = rrt->getRandomNode();
        if (q)
        {
            Node *qNearest = rrt->nearest(q->position);
            if (rrt->distance(q->position, qNearest->position) > rrt->step_size)
            {
                Vector2i newPos = rrt->newConfig(q->position, qNearest->position);
                if (!isSegmentObstacle(msg, qNearest->position, newPos))
                {
                    Node *qNew = new Node;
                    qNew->position = newPos;

                    rrt->add(qNearest, qNew);
                    if (animate)
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
        if (rrt->goalReached())
        {
            cout << "Goal reached successfully" << endl;
            Node *q = rrt->lastNode;
            while (q != rrt->root)
            {
                cv::line(display, cv::Point(q->position.x(), q->position.y()),
                         cv::Point(q->parent->position.x(), q->parent->position.y()), cv::Scalar(0, 0, 255));
                q = q->parent;
            }
            break;
        }
    }
    if (!rrt->goalReached())
        cout << "Oops! RRT failed to reach goal. Please increase maximum iterations or try with different step size." << endl;
    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.
    imshow("Display window", display);                      // Show our image inside it.
    cv::waitKey(0);
}

int main(int argc, char **argv)
{
    srand(time(0));
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/map", 1000, findPath);

    // Don't exit the program.
    ros::spin();
    return 0;
}