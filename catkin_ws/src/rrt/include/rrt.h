#include <stdlib.h>
#include <vector>
#include <math.h>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

struct Node
{
    vector<Node *> children;
    Node *parent;
    Vector2i position;
};

class RRT
{
public:
    RRT(const Vector2i _startPos, const Vector2i _endPos, int _stepSize, int _maxIter);
    void reset();
    Node *getRandomNode(const int imgWidth, const int imgHeight);
    Node *nearest(const Vector2i point);
    float distance(const Vector2i &p, const Vector2i &q);
    Vector2i newConfig(const Vector2i &q, const Vector2i &qNearest);
    void add(Node *qNearest,Node *qNew);
    bool goalReached();
    void deleteNodes(Node *root);
    vector<Node *> nodes;
    vector<Node *> path;
    Node *root, *lastNode;
    Vector2i startPos, endPos;
    int max_iter;
    int step_size;
};
