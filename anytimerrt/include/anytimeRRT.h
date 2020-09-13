#include <stdlib.h>
#include <vector>
#include <math.h>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

struct Node
{
    vector<Node *> childern;
    Node *parent;
    Vector2i position;
    double cost;
};

class anytimeRRT
{
public:
    anytimeRRT(const Vector2i _startPos, const Vector2i _endPos, int _stepSize, int _algoSpeed,
                                            int _maxIter, int _maxRuns, float _costToGoFactor);
    void reset();
    Node *getRandomNode(const int imgWidth, const int imgHeight);
    Node *nearest(const Vector2i point);
    void nearby(const Vector2i point, const float radius, vector<Node *> &nearby_nodes);
    Vector2i newConfig(const Vector2i q, const Vector2i qNearest);
    float distance(const Vector2i p, const Vector2i q);
    void add(Node *qNearest, Node *qNew);
    float costToGo(const Node *q);
    void splitTree();// * : generate commited, call deletecommited and numOfRuns--
    bool goalReached();
    void findQmin();
    void deleteBranch(Node *root);
    void rebuildNodesVec(Node *newRoot);// * : reintializes nodes from freePath
    vector<Node *> nodes;// * : remove commited nodes and branches in deleteCommited()
    vector<Node *> freePath;// * : initialize after RRT run and divide in splitTree()
    vector<Node *> completedPath;//todo : tracks the robot movement
    vector<Node *> commitedPath;// * : generate in splitTree()
    Node *root, *currentNode, *qMin, *lastNode;//todo : currentNode is postion of node in commitedPath, qMin = node of least cost in goal region
    Vector2i startPos, endPos;
    int max_iter;
    int algoSpeed;
    int step_size;
    int maxRuns;
    int numOfRuns;
    float costToGoFactor;// * has to have value between 0 and 1
};