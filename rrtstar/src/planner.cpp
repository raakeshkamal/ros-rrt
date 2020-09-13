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

#include "rrtStar.h"

using namespace std;
using namespace Eigen;

RRTSTAR *rrtStar;
bool animate;
float NEIGHBOUR_FACTOR;

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

	// * create a box with startPos and newPos
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

	// * check if box is occupied
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

	// * Error handling
	if (rrtStar->startPos.x() < 0 || rrtStar->startPos.y() < 0 || rrtStar->startPos.x() > info.width || rrtStar->startPos.y() > info.height)
	{
		cout << "Coordinates of the start point is out of bounds! Please check launch file" << endl;
		ros::shutdown();
	}
	else if (rrtStar->endPos.x() < 0 || rrtStar->endPos.y() < 0 || rrtStar->endPos.x() > info.width || rrtStar->endPos.y() > info.height)
	{
		cout << "Coordinates of the end point is out of bounds! Please check launch file" << endl;
		ros::shutdown();
	}
	else if (msg->data[rrtStar->startPos.x() + info.width * (info.height - rrtStar->startPos.y() - 1)] == 0)
	{
		cout << "Coordinates of the start point is occupied! Please check launch file" << endl;
		ros::shutdown();
	}
	else if (msg->data[rrtStar->endPos.x() + info.width * (info.height - rrtStar->endPos.y() - 1)] == 0)
	{
		cout << "Coordinates of the end point is occupied! Please check launch file" << endl;
		ros::shutdown();
	}
	else if (rrtStar->step_size < 0)
	{
		cout << "Step size invalid! Please check launch file" << endl;
		ros::shutdown();
	}
	else if (rrtStar->max_iter < 0)
	{
		cout << "Max iterations invalid! Please check launch file" << endl;
		ros::shutdown();
	}

	if (!ros::isShuttingDown())
	{
		cv::Mat display(500, 500, CV_8UC3);

		// * convert OccupancyGrid to cv::Mat for display
		for (unsigned int x = 0; x < info.width; x++)
			for (unsigned int y = 0; y < info.height; y++)
			{
				cv::Vec3b &color = display.at<cv::Vec3b>(y, x);
				uchar pixel = ((100.0f - msg->data[x + info.width * (info.height - y - 1)]) / 100.0f) * 255;
				color[0] = pixel;
				color[1] = pixel;
				color[2] = pixel;
			}

		// * add start Point and end Point
		cv::circle(display, cv::Point(rrtStar->startPos.x(), rrtStar->startPos.y()), 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
		cv::circle(display, cv::Point(rrtStar->endPos.x(), rrtStar->endPos.y()), 4, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);

		int iterStep = (int)((float)rrtStar->max_iter) / 10;

		for (int i = 0; i < rrtStar->max_iter; i++)
		{
			if (i % iterStep == 0)
				cout << (int)(i / ((float)rrtStar->max_iter) * 100) << "% of iterations done...." << endl;

			Node *q = rrtStar->getRandomNode(info.width, info.height);
			if (q)
			{
				Node *qNearest = rrtStar->nearest(q->position);
				if (rrtStar->distance(q->position, qNearest->position) > rrtStar->step_size)
				{
					Vector2i newPos = rrtStar->newConfig(q->position, qNearest->position);
					if (!isSegmentObstacle(msg, newPos, qNearest->position))
					{ // * obstacle checks
						Node *qNew = new Node;
						qNew->position = newPos;
						vector<Node *> nearby_nodes;
						rrtStar->nearby(qNew->position, rrtStar->step_size * NEIGHBOUR_FACTOR, nearby_nodes);
						Node *qMinCost = qNearest;
						float minCost = qNearest->cost + rrtStar->distance(qNearest->position, qNew->position);
						// * search in space around qNew for least cost solution
						for (int j = 0; j < (int)nearby_nodes.size(); j++)
						{
							Node *qNearby = nearby_nodes[j];
							if (!isSegmentObstacle(msg, qNearby->position, qNew->position) && (qNearby->cost + rrtStar->distance(qNearby->position, qNew->position)) < minCost)
							{
								qMinCost = qNearby;
								minCost = qNearby->cost + rrtStar->distance(qNearby->position, qNew->position);
							}
						}
						rrtStar->add(qMinCost, qNew);// * link to the least cost solution
						if (animate)
						{
							cv::circle(display, cv::Point(qNew->position.x(), qNew->position.y()), 1,
									   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
							cv::line(display, cv::Point(qNew->position.x(), qNew->position.y()),
									 cv::Point(qMinCost->position.x(), qMinCost->position.y()), cv::Scalar(255, 0, 0));
						}

						for (int j = 0; j < (int)nearby_nodes.size(); j++) // * rewire the nearby nodes to optimize the cost
						{
							Node *qNearby = nearby_nodes[j];
							if (!isSegmentObstacle(msg, qNew->position, qNearby->position) && (qNew->cost + rrtStar->distance(qNew->position, qNearby->position)) < qNearby->cost)
							{
								Node *qParentToNearby = qNearby->parent;
								// * break link between qNearby and qParentToNearby
								if (animate)
									cv::line(display, cv::Point(qNearby->position.x(), qNearby->position.y()),
											 cv::Point(qParentToNearby->position.x(), qParentToNearby->position.y()), cv::Scalar(0, 0, 0));

								// * add link between qNew and qNearby
								qNearby->cost = qNew->cost + rrtStar->distance(qNew->position, qNearby->position);
								qNearby->parent = qNew;
								qNew->childern.push_back(qNearby);
								if (animate)
								{
									cv::line(display, cv::Point(qNew->position.x(), qNew->position.y()),
											 cv::Point(qNearby->position.x(), qNearby->position.y()), cv::Scalar(255, 0, 0));
									cv::circle(display, cv::Point(qNearby->position.x(), qNearby->position.y()), 1,
											   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
									cv::circle(display, cv::Point(qParentToNearby->position.x(), qParentToNearby->position.y()), 1,
											   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
								}
							}
						}
					}
					if (animate)
					{
						imshow("Display window", display); // * Show our image inside it.
						cv::waitKey(10);
					}
				}
			}
			if (rrtStar->goalReached())
			{
				cout << "Goal reached successfully" << endl;
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
		if (!rrtStar->goalReached()) //failure condition
			cout << "Oops! RRTSTAR failed to reach goal. Please increase maximum iterations or try with different step size." << endl;
		cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.
		imshow("Display window", display);						// Show our image inside it.
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
	srand(time(0)); // * randomize the randomNode function

	ros::init(argc, argv, "planner");
	ros::NodeHandle nh("~");
	Vector2i startPos, endPos;
	int stepSize, maxIter;
	float neighbourFactor;

	// * gather inputs from launch file
	nh.getParam("startX", startPos.x());
	nh.getParam("startY", startPos.y());
	nh.getParam("endX", endPos.x());
	nh.getParam("endY", endPos.y());
	nh.getParam("stepSize", stepSize);
	nh.getParam("neighbourFactor", NEIGHBOUR_FACTOR);
	nh.getParam("maxIter", maxIter);
	nh.getParam("animate", animate);

	rrtStar = new RRTSTAR(startPos, endPos, stepSize, maxIter); // * create a RRT class instance

	ros::Subscriber sub = nh.subscribe("/map", 1000, findPath); // * subscribe to /map

	// * Don't exit the program.
	ros::spin();
	return 0;
}