#include <rrtStar.h>

RRTSTAR::RRTSTAR(const Vector2i _startPos, const Vector2i _endPos, int _stepSize, int _maxIter)
{
    startPos = _startPos;
    endPos = _endPos;
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    root->cost = 0.0;
    lastNode = root;
    nodes.push_back(root);
    step_size = _stepSize;
    max_iter = _maxIter;
}

void RRTSTAR::reset()
{
    deleteNodes(root);
    nodes.clear();
    nodes.resize(0);
    path.clear();
    path.resize(0);

    root->parent = NULL;
    root->position = startPos;
    root->cost = 0.0;
    lastNode = root;
    nodes.push_back(root);
}

Node *RRTSTAR::getRandomNode(const int imgWidth, const int imgHeight)
{
    Node *ret;
    Vector2i point(rand() % imgWidth, rand() % imgHeight);
    ret = new Node;
    ret->position = point;
    return ret;
}

float RRTSTAR::distance(const Vector2i p, const Vector2i q)
{
    Vector2i r = p - q;
    return sqrt(powf(r.x(), 2) + powf(r.y(), 2));
}

Node *RRTSTAR::nearest(const Vector2i point)
{
    float minDist = numeric_limits<float>::max();
    Node *closest = NULL;
    for (int i = 0; i < (int)nodes.size(); i++)
    {
        float dist = distance(point, nodes[i]->position);
        if (dist < minDist)
        {
            closest = nodes[i];
            minDist = dist;
        }
    }
    return closest;
}

void RRTSTAR::nearby(Vector2i point, float radius, vector<Node *> &nearby_nodes)
{
    for (int i = 0; i < (int)nodes.size(); i++)
    {
        double dist = distance(point, nodes[i]->position);
        if (dist < radius)
        {
            nearby_nodes.push_back(nodes[i]);
        }
    }
}

Vector2i RRTSTAR::newConfig(const Vector2i q, const Vector2i qNearest)
{
    Vector2f direction = (q - qNearest).cast<float>();
    direction = direction / direction.norm();
    Vector2i ret = qNearest + (step_size * direction).cast<int>();
    return ret;
}

void RRTSTAR::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNew->cost = qNearest->cost + distance(qNearest->position, qNew->position);
    qNearest->childern.push_back(qNew);
    nodes.push_back(qNew);
    lastNode = qNew;
}

bool RRTSTAR::goalReached()
{
    if (distance(lastNode->position, endPos) < step_size)
        return true;
    else
        return false;
}

void RRTSTAR::deleteNodes(Node *root)
{
    for (int i = 0; i < (int)root->childern.size(); i++)
    {
        deleteNodes(root->childern[i]);
    }
    delete root;
}