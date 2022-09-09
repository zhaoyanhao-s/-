#include "octree.h"
#include "function.h"

OctreeNode* OctreeNode::BuildOctree(const Point& center, const float& halflength, const float& halfwidth, const float& halfheight, int stopDepth) {
    if (stopDepth < 0) return NULL;
    else {
        // Construct and fill in 'root' of this subtree
        OctreeNode* pNode = new OctreeNode();
        pNode->m_center = center;
        pNode->m_halfwidth = halfwidth;
        pNode->m_halfheight = halfheight;
        pNode->m_halflength = halflength;
        pNode->m_R2 = halfwidth * halfwidth + halflength * halflength;
        pNode->m_pointslist = nullptr;

        //递归的构造出他的八个子节点
        Point offset;
        float widthstep = halfwidth * 0.5f;
        float heightstep = halfheight * 0.5f;
        float lengthstep = halflength * 0.5f;
        for (int i = 0; i < 8; ++i) {
            // 子节点对应中心位置和父节点中心位置的偏移量
            offset.m_x = ((i & 1) ? lengthstep : -lengthstep);
            offset.m_y = ((i & 2) ? widthstep : -widthstep);
            offset.m_z = ((i & 4) ? heightstep : -heightstep);
            Point realcenter = center + offset;
            pNode->m_children[i] = BuildOctree(realcenter, lengthstep, widthstep, heightstep, stopDepth - 1);
        }
        return pNode;
    }
}

void OctreeNode::InsertObject(OctreeNode* pTree, Point* pPoint) {
    //点在不在矩形里面，只需要判断点的坐标和矩形中心及其长宽高的关系
    //以长为例，只要点的x 和 矩形中心的 x 的差的绝对值小于等于 矩形的半长，则在 x 方向上，此点落在矩形中。
    int straddle = 0, index = 0;

    for (int i = 0; i < 3; i++)
    {
        float delta = (*pPoint)[i] - pTree->m_center[i];
        //说明点没有落在该矩形内
        if (abs(delta) > (*pTree)[i])
        {
            straddle = 1;
            break;
        }
        //判断落点应该在那个子节点上
        if (delta > 0.0f)
            index |= (1 << i);
    }
    if (!straddle && pTree->m_children[index]) {
        // 点落在区域内且子节点不为空则递归的调用，一直找到该点所在的最小子块
        InsertObject(pTree->m_children[index], pPoint);
    }
    else {
        //没有子节点，直接插入当前节点的队列中去，头插法
        pPoint->m_next = pTree->m_pointslist;
        pTree->m_pointslist = pPoint;
    }
}

OctreeNode::~OctreeNode() {
    while (m_pointslist != nullptr) {
        Point* next = m_pointslist->m_next;
        delete m_pointslist;
        m_pointslist = next;
    }
    if (m_children[0] != nullptr) {
        for (int i = 0; i < 8; ++i) {
            delete m_children[i];
        }
    }
}
