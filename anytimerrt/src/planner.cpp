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

#include "anytimeRRT.h"

using namespace std;
using namespace Eigen;

anytimeRRT *atRRT;
float NEIGHBOUR_FACTOR;

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

void calculateFreePath(Node *end, cv::Mat screen)
{
	Node *q = end;
	atRRT->freePath.clear();
	atRRT->freePath.resize(0);
	while (q != atRRT->root)
	{
		atRRT->freePath.insert(atRRT->freePath.begin(), q); // * add elements in reverse
		cv::line(screen, cv::Point(q->position.x(), q->position.y()),
				 cv::Point(q->parent->position.x(), q->parent->position.y()), cv::Scalar(0, 0, 255));
		q = q->parent;
	}
	atRRT->freePath.insert(atRRT->freePath.begin(), q); // * add elements in reverse
}

void buildTree(Node *root, cv::Mat screen)
{
	if (root == atRRT->root)
	{
		cv::circle(screen, cv::Point(root->position.x(), root->position.y()), 4,
				   cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_8);
		if (root->parent)
			cv::line(screen, cv::Point(root->position.x(), root->position.y()),
					 cv::Point(root->parent->position.x(), root->parent->position.y()), cv::Scalar(0, 255, 0));
	}
	else if (root == atRRT->qMin)
		calculateFreePath(root, screen);
	cv::circle(screen, cv::Point(root->position.x(), root->position.y()), 1,
			   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
	for (int i = 0; i < (int)root->childern.size(); i++)
	{
		cv::line(screen, cv::Point(root->position.x(), root->position.y()),
				 cv::Point(root->childern[i]->position.x(), root->childern[i]->position.y()), cv::Scalar(255, 0, 0));
		buildTree(root->childern[i], screen);
	}
}
void highlightNodes(cv::Mat screen)
{
	for (int i = 0; i < (int)atRRT->nodes.size(); i++)
	{
		cv::circle(screen, cv::Point(atRRT->nodes[i]->position.x(), atRRT->nodes[i]->position.y()), 1,
				   cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
		if (atRRT->nodes[i]->parent)
			cv::line(screen, cv::Point(atRRT->nodes[i]->position.x(), atRRT->nodes[i]->position.y()),
					 cv::Point(atRRT->nodes[i]->parent->position.x(), atRRT->nodes[i]->parent->position.y()), cv::Scalar(0, 255, 0));
	}
}
void highlightCommitedPath(cv::Mat display)
{
	if (atRRT->commitedPath[0]->parent)
		cv::line(display, cv::Point(atRRT->commitedPath[0]->position.x(), atRRT->commitedPath[0]->position.y()),
				 cv::Point(atRRT->commitedPath[0]->parent->position.x(), atRRT->commitedPath[0]->parent->position.y()), cv::Scalar(0, 255, 0));
	for (int i = 0; i < ((int)atRRT->commitedPath.size() - 1); i++)
	{
		cv::line(display, cv::Point(atRRT->commitedPath[i]->position.x(), atRRT->commitedPath[i]->position.y()),
				 cv::Point(atRRT->commitedPath[i + 1]->position.x(), atRRT->commitedPath[i + 1]->position.y()), cv::Scalar(0, 255, 0));
	}
}

void runRRTStar(const nav_msgs::OccupancyGrid::ConstPtr &msg, cv::Mat display)
{
	std_msgs::Header header = msg->header;
	nav_msgs::MapMetaData info = msg->info;
	cv::Mat screen;
	int forLoopIter = atRRT->max_iter;
	if (atRRT->numOfRuns != atRRT->maxRuns)
	{
		if ((int)atRRT->commitedPath.size() * atRRT->algoSpeed > forLoopIter)
			forLoopIter = (int)atRRT->commitedPath.size() * atRRT->algoSpeed;
	}
	for (int i = 0; i < forLoopIter; i++)
	{
		if (atRRT->numOfRuns != atRRT->maxRuns)
		{
			int currNodeIdx = (int)(i / (float)(atRRT->algoSpeed));
			if (currNodeIdx >= (int)atRRT->commitedPath.size())
				break;
			else
			{

				atRRT->currentNode = atRRT->commitedPath[currNodeIdx];
				cv::circle(display, cv::Point(atRRT->currentNode->position.x(), atRRT->currentNode->position.y()), 4,
						   cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
			}
		}
		Node *q = atRRT->getRandomNode(info.width, info.height);
		if (q)
		{
			Node *qNearest = atRRT->nearest(q->position);
			if (atRRT->distance(q->position, qNearest->position) > atRRT->step_size)
			{
				Vector2i newPos = atRRT->newConfig(q->position, qNearest->position);
				if (!isSegmentObstacle(msg, newPos, qNearest->position))
				{ // * obstacle checks
					Node *qNew = new Node;
					qNew->position = newPos;
					vector<Node *> nearby_nodes;
					atRRT->nearby(qNew->position, atRRT->step_size * NEIGHBOUR_FACTOR, nearby_nodes);
					Node *qMinCost = qNearest;
					float minCost = qNearest->cost + atRRT->distance(qNearest->position, qNew->position);
					for (int j = 0; j < (int)nearby_nodes.size(); j++)
					{
						Node *qNearby = nearby_nodes[j];
						if (!isSegmentObstacle(msg, qNearby->position, qNew->position) &&
							(qNearby->cost + atRRT->distance(qNearby->position, qNew->position)) < minCost)
						{
							qMinCost = qNearby;
							minCost = qNearby->cost + atRRT->distance(qNearby->position, qNew->position);
						}
					}
					atRRT->findQmin();
					bool addQnew = false;
					if (!atRRT->qMin)
						addQnew = true;
					else if ((qNew->cost + atRRT->costToGo(qNew)) < atRRT->qMin->cost) // * branch and bound
						addQnew = true;
					if (addQnew)
					{
						atRRT->add(qMinCost, qNew);

						for (int j = 0; j < (int)nearby_nodes.size(); j++)
						{
							Node *qNearby = nearby_nodes[j];
							if (!isSegmentObstacle(msg, qNew->position, qNearby->position) &&
								(qNew->cost + atRRT->distance(qNew->position, qNearby->position)) < qNearby->cost)
							{
								Node *qParentToNearby = qNearby->parent;
								// * break link between qNearby and qParentToNearby
								auto it = find(qParentToNearby->childern.begin(), qParentToNearby->childern.end(), qNearby);
								if (it != qParentToNearby->childern.end())
									qParentToNearby->childern.erase(it);

								// * add link between qNew and qNearby
								qNearby->cost = qNew->cost + atRRT->distance(qNew->position, qNearby->position);
								qNearby->parent = qNew;
								qNew->childern.push_back(qNearby);
							}
						}
					}
					screen = display.clone();
					buildTree(atRRT->root, screen);
					imshow("Display window", screen); // * Show our image inside it.
					cv::waitKey(10);
				}
			}
		}
		if (atRRT->goalReached() && atRRT->numOfRuns == atRRT->maxRuns) // * only for the first RRT run
		{
			return;
		}
	}
	return;
}

void findPath(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
	std_msgs::Header header = msg->header;
	nav_msgs::MapMetaData info = msg->info;

	cout << "Got map of dimensions " << info.width << " x " << info.height << endl;

	//Error handling
	if (atRRT->startPos.x() < 0 || atRRT->startPos.y() < 0 || atRRT->startPos.x() > info.width || atRRT->startPos.y() > info.height)
	{
		cout << "Coordinates of the start point is out of bounds! Please check launch file" << endl;
		ros::shutdown();
	}
	else if (atRRT->endPos.x() < 0 || atRRT->endPos.y() < 0 || atRRT->endPos.x() > info.width || atRRT->endPos.y() > info.height)
	{
		cout << "Coordinates of the end point is out of bounds! Please check launch file" << endl;
		ros::shutdown();
	}
	else if (msg->data[atRRT->startPos.x() + info.width * (info.height - atRRT->startPos.y() - 1)] == 0)
	{
		cout << "Coordinates of the start point is occupied! Please check launch file" << endl;
		ros::shutdown();
	}
	else if (msg->data[atRRT->endPos.x() + info.width * (info.height - atRRT->endPos.y() - 1)] == 0)
	{
		cout << "Coordinates of the end point is occupied! Please check launch file" << endl;
		ros::shutdown();
	}
	else if (atRRT->step_size < 0)
	{
		cout << "Step size invalid! Please check launch file" << endl;
		ros::shutdown();
	}
	else if (atRRT->max_iter < 0)
	{
		cout << "Max iterations invalid! Please check launch file" << endl;
		ros::shutdown();
	}
	if (!ros::isShuttingDown())
	{
		cv::Mat display(500, 500, CV_8UC3);
		cv::Mat screen;

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
		cv::circle(display, cv::Point(atRRT->startPos.x(), atRRT->startPos.y()), 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
		cv::circle(display, cv::Point(atRRT->endPos.x(), atRRT->endPos.y()), 4, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
		runRRTStar(msg, display);
		if (!atRRT->goalReached())
			cout << "RRT failed at first attempt. Either path doesn't exist or try with higher MAX_ITER" << endl;
		else
		{
			cout << "First RRT run complete. Path found." << endl;
			screen = display.clone();
			buildTree(atRRT->root, screen);
			calculateFreePath(atRRT->lastNode, screen);
			imshow("Display window", screen); // Show our image inside it.
			cv::waitKey(1000);
			atRRT->splitTree();
			highlightCommitedPath(display);
			atRRT->numOfRuns--;
			screen = display.clone();
			if (atRRT->numOfRuns > 0)
				buildTree(atRRT->root, screen);
			imshow("Display window", screen); // Show our image inside it.
			cv::waitKey(2000);
			for (int i = 1; i < atRRT->maxRuns; i++)
			{
				runRRTStar(msg, display);
				atRRT->findQmin();
				screen = display.clone();
				buildTree(atRRT->root, screen);
				imshow("Display window", screen); // Show our image inside it.
				cv::waitKey(1000);
				atRRT->splitTree();
				highlightCommitedPath(display);
				atRRT->numOfRuns--;
				screen = display.clone();
				if (atRRT->numOfRuns > 0)
					buildTree(atRRT->root, screen);
				imshow("Display window", screen); // Show our image inside it.
				cv::waitKey(2000);
			}
			for (int i = 0; i < (int)atRRT->commitedPath.size(); i++)
			{
				atRRT->currentNode = atRRT->commitedPath[i];
				cv::circle(display, cv::Point(atRRT->currentNode->position.x(), atRRT->currentNode->position.y()), 4,
						   cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
				imshow("Display window", display); // Show our image inside it.
				cv::waitKey(500);
			}
			imshow("Display window", display); // Show our image inside it.
			cv::waitKey(0);
		}
	}
}

int main(int argc, char **argv)
{
	srand(time(0));

	ros::init(argc, argv, "planner");
	ros::NodeHandle nh("~");

	Vector2i startPos, endPos;
	int stepSize, algoSpeed, maxIter, maxRuns;
	float costToGoFactor, neighbourFactor;

	//gather inputs from launch file
	nh.getParam("startX", startPos.x());
	nh.getParam("startY", startPos.y());
	nh.getParam("endX", endPos.x());
	nh.getParam("endY", endPos.y());
	nh.getParam("stepSize", stepSize);
	nh.getParam("algoSpeed", algoSpeed);
	nh.getParam("maxRuns", maxRuns);
	nh.getParam("costToGoFactor", costToGoFactor);
	nh.getParam("neighbourFactor", NEIGHBOUR_FACTOR);
	nh.getParam("maxIter", maxIter);

	atRRT = new anytimeRRT(startPos, endPos, stepSize, algoSpeed, maxIter, maxRuns, costToGoFactor); //create a RRT class instance

	ros::Subscriber sub = nh.subscribe("/map", 1000, findPath); //subscribe to /map

	// Don't exit the program.
	ros::spin();
	return 0;
}