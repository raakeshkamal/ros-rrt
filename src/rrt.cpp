#include "rrt.h"
/**
 * RRT::RRT 
 * 
 */
RRT::RRT()
{
    startPos.x() = START_POS_X;
    startPos.y() = START_POS_Y;
    endPos.x() = END_POS_X;
    endPos.y() = END_POS_Y;
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    lastNode = root;
    nodes.push_back(root);
    step_size = 3;
    max_iter = 3000;
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
 * RRT 
 * 
 * @return {Node*}  : 
 */
Node *RRT::getRandomNode()
{
    Node *ret;
    Vector2i point(rand() % IMG_WIDTH, rand() % IMG_HEIGHT);
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
Node *RRT::nearest(Vector2i point)
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