#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <unistd.h>
#include <time.h>
#include "rrtStar.h"

using namespace std;
using namespace Eigen;

const int STEP_SIZE = 10;
const int MAX_ITER = 10000;
const int IMAGE_WIDTH = 500;
const int IMAGE_HEIGHT = 500;
const Vector2i startPos(10, 10);
const Vector2i endPos(480, 480);
const float NEIGHBOUR_FACTOR = 2.0;

bool isSegmentObstacle(const cv::Mat &image, const Vector2i &nearestPos, const Vector2i &newPos)
{
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
			if ((int)(image.at<uchar>(j, i)) == 255)
				return true;
		}
	}
	return false;
}

int main(int argc, char **argv)
{
	srand(time(0));
	cv::String path = "/home/raakesh/Documents/cppprojects/ros-rrtStar/src/map3.png";
	cv::Mat image, display;
	image = cv::imread(path, CV_8UC1);               // Read the file
	display = cv::imread(path, CV_LOAD_IMAGE_COLOR); // Read the file

	//cv::Mat image(500, 500, CV_8UC1);
	//cv::Mat display(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));

	if (!image.data || !display.data) // Check for invalid input
	{
		cout << "Could not open or find the image" << endl;
		return -1;
	}
	cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.

	RRTSTAR *rrtStar = new RRTSTAR(startPos, endPos, STEP_SIZE, MAX_ITER);
	cv::circle(display, cv::Point(rrtStar->startPos.x(), rrtStar->startPos.y()), 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
	cv::circle(display, cv::Point(rrtStar->endPos.x(), rrtStar->endPos.y()), 4, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
	for (int i = 0; i < rrtStar->max_iter; i++)
	{
		Node *q = rrtStar->getRandomNode(IMAGE_WIDTH, IMAGE_WIDTH);
		if (q)
		{
			Node *qNearest = rrtStar->nearest(q->position);
			if (rrtStar->distance(q->position, qNearest->position) > rrtStar->step_size)
			{
				Vector2i newPos = rrtStar->newConfig(q->position, qNearest->position);
				if (!isSegmentObstacle(image, newPos, qNearest->position))
				{ //obstacle checks
					Node *qNew = new Node;
					qNew->position = newPos;
					vector<Node *> nearby_nodes;
					rrtStar->nearby(qNew->position, rrtStar->step_size * NEIGHBOUR_FACTOR, nearby_nodes);
					Node *qMinCost = qNearest;
					float minCost = qNearest->cost + rrtStar->distance(qNearest->position, qNew->position);
					for (int j = 0; j < (int)nearby_nodes.size(); j++)
					{
						Node *qNearby = nearby_nodes[j];
						if (!isSegmentObstacle(image, qNearby->position, qNew->position) && (qNearby->cost + rrtStar->distance(qNearby->position, qNew->position)) < minCost)
						{
							qMinCost = qNearby;
							minCost = qNearby->cost + rrtStar->distance(qNearby->position, qNew->position);
						}
					}
					rrtStar->add(qMinCost, qNew);
					cv::circle(display, cv::Point(qNew->position.x(), qNew->position.y()), 1,
							   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
					cv::line(display, cv::Point(qNew->position.x(), qNew->position.y()),
							 cv::Point(qMinCost->position.x(), qMinCost->position.y()), cv::Scalar(255, 0, 0));

					for (int j = 0; j < (int)nearby_nodes.size(); j++)
					{
						Node *qNearby = nearby_nodes[j];
						if (!isSegmentObstacle(image, qNew->position, qNearby->position) && (qNew->cost + rrtStar->distance(qNew->position, qNearby->position)) < qNearby->cost)
						{
							Node *qParentToNearby = qNearby->parent;
							//break link between qNearby and qParentToNearby
							cv::line(display, cv::Point(qNearby->position.x(), qNearby->position.y()),
									 cv::Point(qParentToNearby->position.x(), qParentToNearby->position.y()), cv::Scalar(0, 0, 0));

							//add link between qNew and qNearby
							qNearby->cost = qNew->cost + rrtStar->distance(qNew->position, qNearby->position);
							qNearby->parent = qNew;
							qNew->childern.push_back(qNearby);
							cv::line(display, cv::Point(qNew->position.x(), qNew->position.y()),
									 cv::Point(qNearby->position.x(), qNearby->position.y()), cv::Scalar(255, 0, 0));
							cv::circle(display, cv::Point(qNearby->position.x(), qNearby->position.y()), 1,
									   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
							cv::circle(display, cv::Point(qParentToNearby->position.x(), qParentToNearby->position.y()), 1,
									   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
						}
					}
				}
				imshow("Display window", display); // Show our image inside it.
				cv::waitKey(10);
			}
		}
		if (rrtStar->goalReached())
		{
			cout << "Goal Reached" << endl;
			Node *q = rrtStar->lastNode;
			while (q != rrtStar->root)
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