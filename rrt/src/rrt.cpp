#include "rrt.h"
/**
 * RRT::RRT 
 * 
 * @param  {Vector2i} _startPos : 
 * @param  {Vector2i} _endPos   : 
 * @param  {int} _stepSize      : 
 * @param  {int} _maxIter       : 
 */
RRT::RRT(const Vector2i _startPos, const Vector2i _endPos, int _stepSize, int _maxIter)
{
    startPos = _startPos;
    endPos = _endPos;
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    lastNode = root;
    nodes.push_back(root);
    step_size = _stepSize;
    max_iter = _maxIter;
}
/**
 * RRT 
 * 
 */
void RRT::reset()
{
    deleteNodes(root);
    nodes.clear();
    nodes.resize(0);
    path.clear();
    path.resize(0);

    root->parent = NULL;
    root->position = startPos;
    lastNode = root;
    nodes.push_back(root);
}
/**
 * Node*RRT::getRandomNode 
 * 
 * @param  {int} imgWidth  : 
 * @param  {int} imgHeight : 
 */
Node *RRT::getRandomNode(const int imgWidth, const int imgHeight)
{
    Node *ret;
    Vector2i point(rand() % imgWidth, rand() % imgHeight);
    ret = new Node;
    ret->position = point;
    return ret;
}
/**
 * RRT 
 * 
 * @param  {Vector2i} p : 
 * @param  {Vector2i} q : 
 * @return {float}      : 
 */
float RRT::distance(const Vector2i &p, const Vector2i &q)
{
    Vector2i v = p - q;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}
/**
 * Node*RRT::nearest 
 * 
 * @param  {Vector2i} point : 
 */
Node *RRT::nearest(const Vector2i point)
{
    float minDist = numeric_limits<float>::max();
    Node *closest = NULL;
    for (int i = 0; i < (int)nodes.size(); i++)
    {
        float dist = distance(point, nodes[i]->position);
        if (dist < minDist)
        {
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}
/**
 * RRT 
 * 
 * @param  {Vector2i} q        : 
 * @param  {Vector2i} qNearest : 
 * @return {Vector2i}          : 
 */
Vector2i RRT::newConfig(const Vector2i &q, const Vector2i &qNearest)
{
    Vector2f direction = (q - qNearest).cast<float>();
    direction = direction / direction.norm();
    Vector2i ret = qNearest + (step_size * direction).cast<int>();
    return ret;
}
/**
 * RRT 
 * 
 * @param  {Node*} qNearest : 
 * @param  {Node*} qNew     : 
 */
void RRT::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNearest->children.push_back(qNew);
    nodes.push_back(qNew);
    lastNode = qNew;
}
/**
 * RRT 
 * 
 * @return {bool}  : 
 */
bool RRT::goalReached()
{
    if (distance(lastNode->position, endPos) < step_size)
        return true;
    else
        return false;
}
/**
 * RRT 
 * 
 * @param  {Node*} root : 
 */
void RRT::deleteNodes(Node *root)
{
    for (int i = 0; i < (int)root->children.size(); i++)
    {
        deleteNodes(root->children[i]);
    }
    delete root;
}