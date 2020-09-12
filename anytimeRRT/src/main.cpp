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
const int ALGO_SPEED = 200;
const int MAX_ITER = 10000;
const int IMAGE_WIDTH = 500;
const int IMAGE_HEIGHT = 500;
const Vector2i startPos(10, 10);
const Vector2i endPos(480, 480);
const float NEIGHBOUR_FACTOR = 3.0;
const int MAX_RUNS = 4;
const float costToGoFactor = 3;

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

void buildTree(Node *root, cv::Mat screen)
{
	cv::circle(screen, cv::Point(root->position.x(), root->position.y()), 1,
			   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
	for (int i = 0; i < (int)root->childern.size(); i++)
	{
		cv::line(screen, cv::Point(root->position.x(), root->position.y()),
				 cv::Point(root->childern[i]->position.x(), root->childern[i]->position.y()), cv::Scalar(255, 0, 0));
		buildTree(root->childern[i], screen);
	}
}
void highlightNodes(anytimeRRT *atRRT, cv::Mat screen)
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
void calculateFreePath(anytimeRRT *atRRT, Node *end, cv::Mat screen)
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
void highlightCommitedPath(anytimeRRT *atRRT, cv::Mat display)
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

void runRRTStar(anytimeRRT *atRRT, cv::Mat image, cv::Mat display)
{
	cv::Mat screen;
	int forLoopIter = atRRT->max_iter;
	if(atRRT->numOfRuns != atRRT->maxRuns){
		if((int)atRRT->commitedPath.size()*atRRT->algoSpeed > forLoopIter)
			forLoopIter = (int)atRRT->commitedPath.size()*atRRT->algoSpeed;
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
						if (!isSegmentObstacle(image, qNearby->position, qNew->position) &&
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
							if (!isSegmentObstacle(image, qNew->position, qNearby->position) &&
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

int main(int argc, char **argv)
{
	srand(time(0));
	cv::String path = "/home/raakesh/Documents/cppprojects/anytimeRRT/src/map1.png";
	cv::Mat image, display, screen;
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

	anytimeRRT *atRRT = new anytimeRRT(startPos, endPos, STEP_SIZE, ALGO_SPEED, MAX_ITER, MAX_RUNS, costToGoFactor);
	cv::circle(display, cv::Point(atRRT->startPos.x(), atRRT->startPos.y()), 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
	cv::circle(display, cv::Point(atRRT->endPos.x(), atRRT->endPos.y()), 4, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
	runRRTStar(atRRT, image, display);
	if (!atRRT->goalReached())
		cout << "RRT failed at first attempt. Either path doesn't exist or try with higher MAX_ITER" << endl;
	else
	{
		cout << "First RRT run complete. Path found." << endl;
		screen = display.clone();
		buildTree(atRRT->root, screen);
		calculateFreePath(atRRT, atRRT->lastNode, screen);
		imshow("Display window", screen); // Show our image inside it.
		cv::waitKey(1000);
		atRRT->splitTree();
		highlightCommitedPath(atRRT, display);
		atRRT->numOfRuns--;
		screen = display.clone();
		if (atRRT->numOfRuns > 0)
			buildTree(atRRT->root, screen);
		imshow("Display window", screen); // Show our image inside it.
		cv::waitKey(2000);
		for (int i = 1; i < atRRT->maxRuns; i++)
		{
			runRRTStar(atRRT, image, display);
			atRRT->findQmin();
			screen = display.clone();
			buildTree(atRRT->root, screen);
			calculateFreePath(atRRT, atRRT->qMin, screen);
			imshow("Display window", screen); // Show our image inside it.
			cv::waitKey(1000);
			atRRT->splitTree();
			highlightCommitedPath(atRRT, display);
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
	return 0;
}