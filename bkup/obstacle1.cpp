#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <unistd.h>
#include <time.h>
#include "rrt.h"

using namespace std;
using namespace Eigen;

const int STEP_SIZE = 20;
const int MAX_ITER = 5000;

void isSegmentObstacle(const cv::Mat &img, const Vector2i nearestPos, Vector2i *newPos)
{
    Vector2i start, end;
    start = nearestPos;
    end = *newPos;
    float m;
    int c, i, j;
    if (end.x() != start.x())
    {
        m = ((float)(end.y() - start.y())) / ((float)(end.x() - start.x()));
        c = start.y();
        if (start.x() < end.x())
        {
            for (i = start.x(); i <= end.x(); i++)
            {
                j = (int)(m * (i - start.x())) + c;
                if ((int)(img.at<uchar>(j, i)) == 255)
                    break;
            }
        }
        else
        {
            for (i = start.x(); i >= end.x(); i--)
            {
                j = (int)(m * (i - start.x())) + c;
                if ((int)(img.at<uchar>(j, i)) == 255)
                    break;
            }
        }
    }
    else
    {
        i = start.x();
        if (start.y() < end.y())
        {
            for (j = start.y(); j <= end.y(); j++)
            {
                if ((int)(img.at<uchar>(j, i)) == 255)
                    break;
            }
        }
        else
        {
            for (j = start.y(); j >= end.y(); j--)
            {
                if ((int)(img.at<uchar>(j, i)) == 255)
                    break;
            }
        }
    }

    newPos->x() = i;
    newPos->y() = j;
}

int main(int argc, char **argv)
{
    srand(time(0));
    cv::String path = "/home/raakesh/Documents/ros-rrt/src/map1.png";
    cv::Mat image, display;
    image = cv::imread(path, CV_8UC1);               // Read the file
    display = cv::imread(path, CV_LOAD_IMAGE_COLOR); // Read the file

    //cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255,255,255));

    if (!image.data || !display.data) // Check for invalid input
    {
        cout << "Could not open or find the image" << endl;
        return -1;
    }
    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.

    RRT *rrt = new RRT;
    rrt->step_size = STEP_SIZE;
    rrt->max_iter = MAX_ITER;
    cv::circle(display, cv::Point(rrt->startPos.x(), rrt->startPos.y()), 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
    cv::circle(display, cv::Point(rrt->endPos.x(), rrt->endPos.y()), 4, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
    for (int i = 0; i < rrt->max_iter; i++)
    {
        Node *q = rrt->getRandomNode();
        if (q)
        {
            Node *qNearest = rrt->nearest(q->position);
            if (rrt->distance(q->position, qNearest->position) > rrt->step_size)
            {
                Vector2i newPos = rrt->newConfig(q->position, qNearest->position);
                //obstacle checks
                avoidObstacles(image, qNearest->position, &newPos);
                Node *qNew = new Node;
                qNew->position = newPos;

                rrt->add(qNearest, qNew);
                cv::circle(display, cv::Point(qNew->position.x(), qNew->position.y()), 1,
                           cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
                cv::line(display, cv::Point(qNew->position.x(), qNew->position.y()),
                         cv::Point(qNearest->position.x(), qNearest->position.y()), cv::Scalar(255, 0, 0));
                imshow("Display window", display); // Show our image inside it.
                cv::waitKey(10);
            }
        }
        if (rrt->goalReached())
        {
            cout << "Goal Reached" << endl;
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
    imshow("Display window", display); // Show our image inside it.
    cv::waitKey(0);
    return 0;
}