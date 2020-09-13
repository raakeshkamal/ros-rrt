#include <anytimeRRT.h>
#include <iostream>
/**
 * anytimeRRT::anytimeRRT 
 * 
 * @param  {Vector2i} _startPos    : 
 * @param  {Vector2i} _endPos      : 
 * @param  {int} _stepSize         : 
 * @param  {int} _algoSpeed        : 
 * @param  {int} _maxIter          : 
 * @param  {int} _maxRuns          : 
 * @param  {float} _costToGoFactor : 
 */
anytimeRRT::anytimeRRT(const Vector2i _startPos, const Vector2i _endPos, int _stepSize, int _algoSpeed,
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
    algoSpeed = _algoSpeed;
    max_iter = _maxIter;
    maxRuns = _maxRuns;
    numOfRuns = maxRuns;
    costToGoFactor = _costToGoFactor;
}
/**
 * anytimeRRT 
 * 
 */
void anytimeRRT::reset()
{
    deleteBranch(root);
    nodes.clear();
    nodes.resize(0);
    freePath.clear();
    freePath.resize(0);
    commitedPath.clear();
    commitedPath.resize(0);

    root->parent = NULL;
    root->position = startPos;
    root->cost = 0.0;
    lastNode = root;
    currentNode = root;
    nodes.push_back(root);
}
/**
 * Node*anytimeRRT::getRandomNode 
 * 
 * @param  {int} imgWidth  : 
 * @param  {int} imgHeight : 
 */
Node *anytimeRRT::getRandomNode(const int imgWidth, const int imgHeight)
{
    Node *ret;
    Vector2i point(rand() % imgWidth, rand() % imgHeight);
    ret = new Node;
    ret->position = point;
    return ret;
}
/**
 * anytimeRRT 
 * 
 * @param  {Vector2i} p : 
 * @param  {Vector2i} q : 
 * @return {float}      : 
 */
float anytimeRRT::distance(const Vector2i p, const Vector2i q)
{
    Vector2i r = p - q;
    return sqrt(powf(r.x(), 2) + powf(r.y(), 2));
}
/**
 * anytimeRRT 
 * 
 * @param  {Node*} q : 
 * @return {float}   : 
 */
float anytimeRRT::costToGo(const Node *q)
{
    return distance(q->position, endPos) * costToGoFactor;
}
/**
 * Node*anytimeRRT::nearest 
 * 
 * @param  {Vector2i} point : 
 */
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
/**
 * anytimeRRT 
 * 
 * @param  {Vector2i} point : 
 * @param  {float} radius   : 
 * @param  {vector<Node*} > : 
 */
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
/**
 * anytimeRRT 
 * 
 * @param  {Vector2i} q        : 
 * @param  {Vector2i} qNearest : 
 * @return {Vector2i}          : 
 */
Vector2i anytimeRRT::newConfig(const Vector2i q, const Vector2i qNearest)
{
    Vector2f direction = (q - qNearest).cast<float>();
    direction = direction / direction.norm();
    Vector2i ret = qNearest + (step_size * direction).cast<int>();
    return ret;
}
/**
 * anytimeRRT 
 * 
 * @param  {Node*} qNearest : 
 * @param  {Node*} qNew     : 
 */
void anytimeRRT::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNew->cost = qNearest->cost + distance(qNearest->position, qNew->position);
    qNearest->childern.push_back(qNew);
    nodes.push_back(qNew);
    lastNode = qNew;
}
/**
 * anytimeRRT 
 * 
 * @return {bool}  : 
 */
bool anytimeRRT::goalReached()
{
    if (distance(lastNode->position, endPos) < step_size)
        return true;
    else
        return false;
}
/**
 * anytimeRRT 
 * 
 */
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
    if (endNodes.empty()) // * if goal not reached
        qMin = NULL;
}
/**
 * anytimeRRT 
 * 
 */
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
        nodes.clear();
        nodes.resize(0);
        rebuildNodesVec(root);
        //numOfRuns--;
    }
}
/**
 * anytimeRRT 
 * 
 * @param  {Node*} root : 
 */
void anytimeRRT::deleteBranch(Node *root)
{
    auto it = find(nodes.begin(), nodes.end(), root);
    for (int i = 0; i < (int)root->childern.size(); i++)
    {
        deleteBranch(root->childern[i]);
    }
    nodes.erase(it);
    delete root;
}
/**
 * anytimeRRT 
 * 
 * @param  {Node*} newRoot : 
 */
void anytimeRRT::rebuildNodesVec(Node *newRoot)
{
    nodes.push_back(newRoot);
    for (int i = 0; i < (int)newRoot->childern.size(); i++)
    {
        rebuildNodesVec(newRoot->childern[i]);
    }
}