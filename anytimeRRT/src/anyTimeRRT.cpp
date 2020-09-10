#include <anytimeRRT.h>

anytimeRRT::anytimeRRT(const Vector2i _startPos, const Vector2i _endPos, int _stepSize,
                       int _maxIter, int _maxRuns, float _costToGoFactor)
{
    startPos = _startPos;
    endPos = _endPos;
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    root->cost = 0.0;
    lastNode = root;
    currentNode = root;
    qMin = NULL;
    nodes.push_back(root);
    step_size = _stepSize;
    max_iter = _maxIter;
    maxRuns = _maxRuns;
    numOfRuns = maxRuns;
    costToGoFactor = _costToGoFactor;
}

void anytimeRRT::reset()
{
    deleteFullTree(root);
    nodes.clear();
    nodes.resize(0);
    freePath.clear();
    freePath.resize(0);
    completedPath.clear();
    completedPath.resize(0);
    commitedPath.clear();
    commitedPath.resize(0);

    root->parent = NULL;
    root->position = startPos;
    root->cost = 0.0;
    lastNode = root;
    currentNode = root;
    nodes.push_back(root);
}

Node *anytimeRRT::getRandomNode(const int imgWidth, const int imgHeight)
{
    Node *ret;
    Vector2i point(rand() % imgWidth, rand() % imgHeight);
    ret = new Node;
    ret->position = point;
    return ret;
}

float anytimeRRT::distance(const Vector2i p, const Vector2i q)
{
    Vector2i r = p - q;
    return sqrt(powf(r.x(), 2) + powf(r.y(), 2));
}

float anytimeRRT::costToGo(const Node *q)
{
    return distance(q->position, endPos) * costToGoFactor;
}

Node *anytimeRRT::nearest(const Vector2i point)
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

void anytimeRRT::nearby(Vector2i point, float radius, vector<Node *> &nearby_nodes)
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

Vector2i anytimeRRT::newConfig(const Vector2i q, const Vector2i qNearest)
{
    Vector2f direction = (q - qNearest).cast<float>();
    direction = direction / direction.norm();
    Vector2i ret = qNearest + (step_size * direction).cast<int>();
    return ret;
}

void anytimeRRT::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNew->cost = qNearest->cost + distance(qNearest->position, qNew->position);
    qNearest->childern.push_back(qNew);
    nodes.push_back(qNew);
    lastNode = qNew;
}

bool anytimeRRT::goalReached()
{
    if (distance(lastNode->position, endPos) < step_size)
        return true;
    else
        return false;
}

void anytimeRRT::findQmin()
{
    vector<Node *> endNodes;
    for (int i = 0; i < (int)nodes.size(); i++)
    {
        if (distance(nodes[i]->position, endPos) < step_size)
            endNodes.push_back(nodes[i]);
    }
    float cmin = numeric_limits<float>::max();
    for (int i = 0; i < (int)endNodes.size(); i++)
    {
        if (endNodes[i]->cost < cmin)
        {
            qMin = endNodes[i];
            cmin = qMin->cost;
        }
    }
    if(endNodes.empty()) // * if goal not reached
        qMin = NULL;
}

void anytimeRRT::splitTree()
{
    if (numOfRuns != 0)
    {
        int commitedPathSize = (int)((float)freePath.size() / numOfRuns);
        auto first = freePath.begin();
        auto last = freePath.begin() + commitedPathSize;
        commitedPath.resize(commitedPathSize);
        copy(first, last, commitedPath.begin());
        freePath.erase(first, last);
        root = freePath[0];
        deleteCommited();
        numOfRuns--;
    }
}

void anytimeRRT::deleteFullTree(Node *root)
{
    for (int i = 0; i < (int)root->childern.size(); i++)
    {
        deleteFullTree(root->childern[i]);
    }
    delete root;
}

void anytimeRRT::deleteBranch(Node *branch)
{
    for (int i = 0; i < (int)branch->childern.size(); i++)
    {
        deleteBranch(branch->childern[i]);
    }
    remove(nodes.begin(), nodes.end(), branch);
    delete branch;
}

void anytimeRRT::deleteCommited()
{
    for (int i = 0; i < (int)commitedPath.size(); i++)
    {
        remove(nodes.begin(), nodes.end(), commitedPath[i]);
        for (int j = 0; j < (int)commitedPath[i]->childern.size(); j++)
            deleteBranch(commitedPath[i]->childern[j]);
    }
}