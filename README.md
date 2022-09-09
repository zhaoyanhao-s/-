点云碰撞检测

一个简单的点云和机器人的碰撞检测，使用八叉树结构离散空间，使用包围球来简化模型，比起包围盒而言，单个碰撞单元的碰撞域增大，但是计算的复杂度大大减小了，使用包围盒的SAT算法大量的向量叉乘，大概有数百次的乘法运算，而两个球之间的碰撞只需判断R1+R2和两个圆心之间的距离即可。缺点是如果点个密度继续增大，那么碰撞域的增加引入大量的点，将得不偿失。

下面是100\*100\*3的空间中的点云分布

![](file://C:\Users\17321\AppData\Roaming\marktext\images\2022-09-09-15-40-46-image.png?msec=1662710676833)

靠近：

![](file://C:\Users\17321\AppData\Roaming\marktext\images\2022-09-09-15-38-52-image.png?msec=1662710676827)

显示碰撞单元（黄色部分）：

![](file://C:\Users\17321\AppData\Roaming\marktext\images\2022-09-09-15-39-28-image.png?msec=1662710676827)

## 编译环境和依赖

Windows vs2019 + qt 6.3.0 + qmake

QT应该包含这两个核心组件

QT += core gui

QT += openglwidgets

## 使用

### robot

由三个长方体cuboid组成，属性如图

![](file://C:\Users\17321\AppData\Roaming\marktext\images\2022-09-09-15-46-43-image.png?msec=1662710676826)

### Point

由xyz三维坐标组成，在可视化界面中是一个个立方体的形式。

### 交互

```cpp
#include "axbopenglwidget.h""
//WSAD键移动视角
virtual void keyPressEvent(QKeyEvent *event);
//鼠标拖动移动方向角
virtual void mouseMoveEvent(QMouseEvent *event);
```

### 应用程序输出

#### 打印

collisionnode[] 和每一个机器人组件发生碰撞的八叉树节点数组

point_x : 碰撞点的x坐标

point_y : 碰撞点的y坐标

count ：发生碰撞的点的个数

NodeCollision consume time ： 完成一次完整的八叉树节点碰撞花费时间（所有组件）。
PointCollision consume time ：完成一次完整的点碰撞检测花费时间（所有组件）。
Total consume time ： 完成100次检测花费时间。

#### 内存

std::unordered_set<Point*> results; 存储发生碰撞的节点。

