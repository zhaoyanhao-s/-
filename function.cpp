#include "function.h"
// 两个包围盒的碰撞检测，OOB的表达形式，一个中心点，三个旋转轴(就是垂直于对应面的方向角，例如Axis就是垂直x面的方向角)，三个边长/半边长
// 检查所选轴之间是否有分离平面
// 只能在靠近的区域使用这种方法，全部使用的话计算量太大
// 而且我们应该先从父节点开始判断，如果其不在父节点中，那么他的所有子节点就不用在判断了

//抛弃不可能的碰撞 即 矩形下边缘 > -1.1
bool abortTesting(const float& zdown,const float& zup) {
    return (zdown > zup);
}


//检测是否存在分离平面/分离轴
bool getSeparatingPlane(const Point& RPos, const Point& Plane, const Cuboid& box1, const Cuboid& box2)
{
    return (fabs(RPos * Plane) >
        (fabs((box1.m_axeX * box1.m_halflength) * Plane) +
            fabs((box1.m_axeY * box1.m_halfwidth) * Plane) +
            fabs((box1.m_axeZ * box1.m_halfheight) * Plane) +
            fabs((box2.m_axeX * box2.m_halflength) * Plane) +
            fabs((box2.m_axeY * box2.m_halfwidth) * Plane) +
            fabs((box2.m_axeZ * box2.m_halfheight) * Plane)));
}

// 检测所有15个可能的组合   可以优化成平面问题
bool getCollision(const Cuboid& box1, const Cuboid& box2)
{
    static Point RPos;
    RPos = box2.m_center - box1.m_center;
    //
    return !(
#ifdef TWO_D
        getSeparatingPlane(RPos, box1.m_axeX, box1, box2) ||
        getSeparatingPlane(RPos, box1.m_axeY, box1, box2) ||
        getSeparatingPlane(RPos, box1.m_axeZ, box1, box2) ||
        getSeparatingPlane(RPos, box2.m_axeX, box1, box2) ||
        getSeparatingPlane(RPos, box2.m_axeY, box1, box2) ||
        getSeparatingPlane(RPos, box2.m_axeZ, box1, box2) ||
#endif// TWO_D
        getSeparatingPlane(RPos, box1.m_axeX ^ box2.m_axeX, box1, box2) ||
        getSeparatingPlane(RPos, box1.m_axeX ^ box2.m_axeY, box1, box2) ||
#ifdef TWO_D
        getSeparatingPlane(RPos, box1.m_axeX ^ box2.m_axeZ, box1, box2) ||
#endif// TWO_D
        getSeparatingPlane(RPos, box1.m_axeY ^ box2.m_axeX, box1, box2) ||
        getSeparatingPlane(RPos, box1.m_axeY ^ box2.m_axeY, box1, box2)
#ifdef TWO_D
        || getSeparatingPlane(RPos, box1.m_axeY ^ box2.m_axeZ, box1, box2) ||
        getSeparatingPlane(RPos, box1.m_axeZ ^ box2.m_axeX, box1, box2) ||
        getSeparatingPlane(RPos, box1.m_axeZ ^ box2.m_axeY, box1, box2) ||
        getSeparatingPlane(RPos, box1.m_axeZ ^ box2.m_axeZ, box1, box2)
#endif// TWO_D
        );
}
bool getCollisionAABB(const Cuboid& octree, const Cuboid& robot) {
    //std::cout << "octree.m_R2 = " << octree.m_R2 << std::endl;
    //std::cout << "robot.m_R2 = " << robot.m_R2 << std::endl;
    return (((octree.m_center.m_x-robot.m_center.m_x)*(octree.m_center.m_x - robot.m_center.m_x)
        + (octree.m_center.m_y - robot.m_center.m_y) * (octree.m_center.m_y - robot.m_center.m_y))
        < (octree.m_R2 + robot.m_R2));
}


//点和长方体的碰撞检测，有一个特点就是该长方体的高是固定的，可以简化成矩形和点的碰撞，利用凸多边形的射线方法检测
//但是对于长方形这种简单的凸多边形，利用投影法则就可以完成了
bool getCollisionProject(Point* p,const Cuboid& robot) {
    //抛弃z方向上不可能的点
    if (p->m_z > robot.m_center.m_z+robot.m_halfheight) return false;
    //矩形中心和点的连线矢量
    Point vec1 = (*p) - robot.m_center;
    //在机器人正方向上的投影
    float proj_y = abs(vec1 * robot.m_axeY);
    float proj_x = abs(vec1 * robot.m_axeX);

    //std::cout << "(abs(vec1 * robot.m_axeY)----all = " << abs(vec1 * robot.m_axeY) << std::endl;
    if (proj_x <= robot.m_halflength && proj_y <= robot.m_halfwidth) {
        //std::cout << "vec = " << vec1.m_x << " " << vec1.m_y << std::endl;
        //std::cout << "robot.m_axeY = " << robot.m_axeY.m_x << " " << robot.m_axeY.m_y << std::endl;
        //std::cout << "proj_x = " << proj_x << std::endl;
        //std::cout << "proj_y = " << proj_y << std::endl;
        return true;
    }
    else
        return false;
}
//递归遍历八叉树所有节点，得到碰撞的结果集
void getCollisionOctreeNode(OctreeNode* node, const Cuboid& robot) {
    if (node == nullptr)
        return;

    if (abortTesting(node->m_center[2] - node->m_halfheight,robot.m_center.m_z+robot.m_halfheight) )  //z轴不合适，直接扔
        return;

    if (getCollisionAABB(*node,robot)) {
        if (node->m_children[0] == nullptr)  //自己就是叶子节点，直接写入结果集
            collisionnode.push_back(node);
        else {                               //不是叶子节点则递归调用
            for (int i = 0; i < 8; ++i)
                getCollisionOctreeNode(node->m_children[i], robot);
        }
    }
    else
        return;
}

//遍历结果集，得到碰撞发生的点
bool getCollisionPoint(const std::vector<OctreeNode*>& collisionnode, const Cuboid& robot) {
    bool target = false;
    //所有碰撞的节点
    for (const auto& num : collisionnode) {
        //该节点上的所有点
        Point* realhead = num->m_pointslist;
        while (num->m_pointslist != nullptr) {
            if (getCollisionProject(num->m_pointslist, robot)) {
                target = true;
                results.insert(num->m_pointslist);
            }
            num->m_pointslist = num->m_pointslist->m_next;
        }
        num->m_pointslist = realhead;
    }
    return target;
}



Point operator+(const Point& lhs, const Point& rhs) {
    return Point{ lhs.m_x + rhs.m_x, lhs.m_y + rhs.m_y, lhs.m_z + rhs.m_z };
}

Point operator-(const Point& lhs, const Point& rhs) {
    return Point{ lhs.m_x - rhs.m_x, lhs.m_y - rhs.m_y, lhs.m_z - rhs.m_z };
}
//点乘-1
float operator* (const Point& lhs, const Point& rhs) {
    return lhs.m_x * rhs.m_x + lhs.m_y * rhs.m_y + lhs.m_z * rhs.m_z ;
}
//点乘-2
Point operator* (const Point& lhs, const float& rhs) {
    return Point{ lhs.m_x * rhs, lhs.m_y * rhs, lhs.m_z * rhs };
}
//叉乘
Point operator^ (const Point& lhs, const Point& rhs) {
    return{ lhs.m_y * rhs.m_z - lhs.m_z * rhs.m_y, lhs.m_z * rhs.m_x - lhs.m_x * rhs.m_z, lhs.m_x * rhs.m_y - lhs.m_y * rhs.m_x };
}
