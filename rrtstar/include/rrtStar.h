#include <stdlib.h>
#include <vector>
#include <math.h>
#include <Eigen/Core>
#include <constants.h>

using namespace std;
using namespace Eigen;

struct Node
{
    vector<Node *> childern;
    Node *parent;
    Vector2i position;
    double cost;
};

class RRTSTAR
{
public:
    RRTSTAR(const Vector2i _startPos, const Vector2i _endPos, int _stepSize, int _maxIter);
    void reset();
    Node *getRandomNode(const int imgWidth, const int imgHeight);
    Node *nearest(const Vector2i point);
    void nearby(const Vector2i point, const float radius, vector<Node *> &nearby_nodes);
    Vector2i newConfig(const Vector2i q, const Vector2i qNearest);
    float distance(const Vector2i p, const Vector2i q);
    void add(Node *qNearest, Node *qNew);
    bool goalReached();
    void deleteNodes(Node *root);
    vector<Node *> nodes;
    vector<Node *> path;
    Node *root, *lastNode;
    Vector2i startPos, endPos;
    int max_iter;
    int step_size;
};
