#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <iostream>
#include <time.h>
#include "anytimeRRT.h"

using namespace std;
using namespace Eigen;

const int STEP_SIZE = 10;
const int MAX_ITER = 10000;
const int IMAGE_WIDTH = 500;
const int IMAGE_HEIGHT = 500;
const Vector2i startPos(10, 10);
const Vector2i endPos(480, 480);
const float NEIGHBOUR_FACTOR = 2.0;
const int MAX_RUNS = 2;
const float costToGoFactor = 0.5;

bool isSegmentObstacle(const cv::Mat &image, const Vector2i &nearestPos, const Vector2i &newPos)
{
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
			if ((int)(image.at<uchar>(j, i)) == 255)
				return true;
		}
	}
	return false;
}

void runRRTStar(anytimeRRT *atRRT, cv::Mat image, cv::Mat display)
{
	//todo : run rrt on free path with new root
	//todo : add robot motion to the function
	//todo : move robot one step for every fixed number of iteration
	//todo : for loop should run for max_iter only for first RUN
	for (int i = 0; i < atRRT->max_iter; i++)
	{
		Node *q = atRRT->getRandomNode(IMAGE_WIDTH, IMAGE_WIDTH);
		if (q)
		{
			Node *qNearest = atRRT->nearest(q->position);
			if (atRRT->distance(q->position, qNearest->position) > atRRT->step_size)
			{
				Vector2i newPos = atRRT->newConfig(q->position, qNearest->position);
				if (!isSegmentObstacle(image, newPos, qNearest->position))
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
						if (!isSegmentObstacle(image, qNearby->position, qNew->position) && (qNearby->cost + atRRT->distance(qNearby->position, qNew->position)) < minCost)
						{
							qMinCost = qNearby;
							minCost = qNearby->cost + atRRT->distance(qNearby->position, qNew->position);
						}
					}
					atRRT->add(qMinCost, qNew);
					/*cv::circle(display, cv::Point(qNew->position.x(), qNew->position.y()), 1,
							   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
					cv::line(display, cv::Point(qNew->position.x(), qNew->position.y()),
							 cv::Point(qMinCost->position.x(), qMinCost->position.y()), cv::Scalar(255, 0, 0));*/

					for (int j = 0; j < (int)nearby_nodes.size(); j++)
					{
						Node *qNearby = nearby_nodes[j];
						if (!isSegmentObstacle(image, qNew->position, qNearby->position) && (qNew->cost + atRRT->distance(qNew->position, qNearby->position)) < qNearby->cost)
						{
							Node *qParentToNearby = qNearby->parent;
							// * break link between qNearby and qParentToNearby
							remove(qParentToNearby->childern.begin(), qParentToNearby->childern.end(), qNearby);
							/*cv::line(display, cv::Point(qNearby->position.x(), qNearby->position.y()),
									 cv::Point(qParentToNearby->position.x(), qParentToNearby->position.y()), cv::Scalar(0, 0, 0));*/

							// * add link between qNew and qNearby
							qNearby->cost = qNew->cost + atRRT->distance(qNew->position, qNearby->position);
							qNearby->parent = qNew;
							qNew->childern.push_back(qNearby);
							/*cv::line(display, cv::Point(qNew->position.x(), qNew->position.y()),
									 cv::Point(qNearby->position.x(), qNearby->position.y()), cv::Scalar(255, 0, 0));
							cv::circle(display, cv::Point(qNearby->position.x(), qNearby->position.y()), 1,
									   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
							cv::circle(display, cv::Point(qParentToNearby->position.x(), qParentToNearby->position.y()), 1,
									   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);*/
						}
					}
				}
				// * : branch and bound after every change to RRT
				atRRT->findQmin(); // * find qMin
				if (!atRRT->qMin)  // * if qMin is available
				{  //todo : 3rd debug test
					for (int i = 0; i < atRRT->nodes.size(); i++)
					{
						if (atRRT->nodes[i]->cost + atRRT->costToGo(atRRT->nodes[i]) > atRRT->qMin->cost)
						{
							Node *purgedNode = atRRT->nodes[i];
							// * : delete node from parent if not null
							if (!(purgedNode->parent))
								remove(purgedNode->parent->childern.begin(), purgedNode->parent->childern.end(), purgedNode);
							// * : delete node from all vectors
							remove(atRRT->nodes.begin(), atRRT->nodes.end(), purgedNode);
							delete purgedNode;
						}
					}
				}
				/*imshow("Display window", display); // * Show our image inside it.
				cv::waitKey(10);*/
			}
		}

		if (atRRT->goalReached() && atRRT->numOfRuns == atRRT->maxRuns) // * only for the first RRT run
		{
			//todo : 1st debug test
			// * generate freepath here after every run and update freepath variable
			cout << "First RRT run complete. Path found" << endl;
			Node *q = atRRT->lastNode;
			atRRT->freePath.clear();
			atRRT->freePath.resize(0);
			while (q != atRRT->root)
			{
				atRRT->freePath.insert(atRRT->freePath.begin(), q); // * add elements in reverse
				/*cv::line(display, cv::Point(q->position.x(), q->position.y()),
						 cv::Point(q->parent->position.x(), q->parent->position.y()), cv::Scalar(0, 0, 255));*/
				q = q->parent;
			}
			atRRT->splitTree();//todo : 2nd debug test
			break;
		}
		//// : review erase and remove function
		//// : review all node removals including old code update all vectors and their size also
		//// : review for update of all vectors for add and delete
		//todo : create a fresh image everytime
		//// : split path into commited and free path using splitTree
		//todo : highlight commited nodes ,remove nodes and branches from nodes vector
		//todo : delete branches from image
	}
	if (atRRT->numOfRuns != atRRT->maxRuns && !atRRT->qMin)
	{	// * from second run onwards
		// * generate freepath here after every run and update freepath variable
		Node *q = atRRT->qMin;
		atRRT->freePath.clear();
		atRRT->freePath.resize(0);
		while (q != atRRT->root)
		{
			atRRT->freePath.insert(atRRT->freePath.begin(), q); // * add elements in reverse
			cv::line(display, cv::Point(q->position.x(), q->position.y()),
					 cv::Point(q->parent->position.x(), q->parent->position.y()), cv::Scalar(0, 0, 255));
			q = q->parent;
		}
		atRRT->splitTree();
	}
	if (!atRRT->goalReached())
	{ // * first RRT run failure
	}
}

int main(int argc, char **argv)
{
	srand(time(0));
	cv::String path = "/home/raakesh/Documents/cppprojects/ros-rrtStar/src/map3.png";
	cv::Mat image, display;
	image = cv::imread(path, CV_8UC1);				 // Read the file
	display = cv::imread(path, CV_LOAD_IMAGE_COLOR); // Read the file

	//cv::Mat image(500, 500, CV_8UC1);
	//cv::Mat display(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));

	if (!image.data || !display.data) // Check for invalid input
	{
		cout << "Could not open or find the image" << endl;
		return -1;
	}
	cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.

	////: push to a function pass new root
	anytimeRRT *atRRT = new anytimeRRT(startPos, endPos, STEP_SIZE, MAX_ITER, MAX_RUNS, costToGoFactor);
	cv::circle(display, cv::Point(atRRT->startPos.x(), atRRT->startPos.y()), 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
	cv::circle(display, cv::Point(atRRT->endPos.x(), atRRT->endPos.y()), 4, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
	runRRTStar(atRRT, image, display);
	imshow("Display window", display); // Show our image inside it.
	cv::waitKey(0);
	return 0;
}