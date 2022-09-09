#pragma once
#include "octree.h"
#include <vector>
#include <unordered_set>

extern std::vector<OctreeNode*> collisionnode;
extern std::unordered_set<Point*> results;

bool abortTesting(const float& z);
bool getSeparatingPlane(const Point& RPos, const Point& Plane, const Cuboid& box1, const Cuboid& box2);
bool getCollision(const Cuboid& box1, const Cuboid& box2);
bool getCollisionAABB(const Cuboid& octree, const Cuboid& robot);
bool getCollisionProject(Point* p, const Cuboid& robot);
void getCollisionOctreeNode(OctreeNode* node, const Cuboid& robot);
bool getCollisionPoint(const std::vector<OctreeNode*>& collisionnode, const Cuboid& robot);
void test();

Point operator+(const Point& lhs, const Point& rhs);
Point operator-(const Point& lhs, const Point& rhs);
//点乘-1
float operator* (const Point& lhs, const Point& rhs);
//点乘-2
Point operator* (const Point& lhs, const float& rhs);
//叉乘
Point operator^ (const Point& lhs, const Point& rhs);
