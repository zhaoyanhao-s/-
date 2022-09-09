#pragma once
#include "cuboid.h"

//八叉树的节点也是一个长方体
class OctreeNode : public Cuboid{
public:
    //OctreeNode(const float& halflength, const float& halfwidth, const float& halfheight, const Point& center) :
    //	Cuboid(halflength, halfwidth, halfheight, center) {};
    ~OctreeNode();
public:
    //指向子节点的指针数组
    OctreeNode* m_children[8];
    //包含在该节点中的所有点，链表的形式存储
    Point* m_pointslist{nullptr};
public:
    //构建一颗特定深度的八叉树
    static OctreeNode* BuildOctree(const Point& center, const float& halflength, const float& halfwidth, const float& halfheight, int stopDepth);
    //向八叉树中插入点
    static void InsertObject(OctreeNode* pTree, Point* pPoint);

};

