#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <unistd.h>
#include "rrt.h"

using namespace std;
using namespace Eigen;

const int STEP_SIZE = 10;
const int MAX_ITER = 2000;

int main(int argc, char **argv)
{

    //cv::Mat image;
    //image = cv::imread("/home/raakesh/Documents/cppprojects/src/map2.pgm", CV_LOAD_IMAGE_GRAYSCALE); // Read the file
    cv::Mat image(500, 500, CV_8UC3, cv::Scalar(0,0,0));

    if (!image.data) // Check for invalid input
    {
        cout << "Could not open or find the image" << endl;
        return -1;
    }
    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.

    RRT *rrt = new RRT;
    rrt->step_size = STEP_SIZE;
    rrt->max_iter = MAX_ITER;
    cv::circle(image, cv::Point(rrt->startPos.x(), rrt->startPos.y()), 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
    cv::circle(image, cv::Point(rrt->endPos.x(), rrt->endPos.y()), 4, cv::Scalar(0, 255 ,0), cv::FILLED, cv::LINE_8);
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
                Node *qNew = new Node;
                qNew->position = newPos;

                rrt->add(qNearest, qNew);
                cv::circle(image, cv::Point(qNew->position.x(), qNew->position.y()), 1, 
                                                            cv::Scalar(0, 255 ,0), cv::FILLED, cv::LINE_8);
                cv::line(image, cv::Point(qNew->position.x(), qNew->position.y()),
                         cv::Point(qNearest->position.x(), qNearest->position.y()), cv::Scalar(255, 0, 0));
                imshow("Display window", image); // Show our image inside it.
                cv::waitKey(10);
            }
        }
        if (rrt->goalReached())
        {
            cout << "Goal Reached" << endl;
            Node* q = rrt->lastNode;
            while(q != rrt->root){
                cv::line(image, cv::Point(q->position.x(), q->position.y()),
                         cv::Point(q->parent->position.x(), q->parent->position.y()), cv::Scalar(0, 0, 255));
                q = q->parent;
            }
            break;
        }
    }
    imshow("Display window", image); // Show our image inside it.
    cv::waitKey(0);
    return 0;
}