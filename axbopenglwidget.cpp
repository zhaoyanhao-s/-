#include "axbopenglwidget.h"
#include "octree.h"
#include "function.h"
#include "mytimer.h"

#define DEBUG
#define TIMEOUTMSEC 100
QPoint deltaPos;
float PI=3.1415926;
float fov=45.0;

unsigned int VBO, VAO,EBO;
std::vector<OctreeNode*> collisionnode;
std::unordered_set<Point*> results;
//单位立方体
float vertices[36][3] = {
{-0.5f, -0.5f, -0.5f},
 {0.5f, -0.5f, -0.5f},
 {0.5f, 0.5f, -0.5f},
 {0.5f, 0.5f, -0.5f},
{-0.5f, 0.5f, -0.5f},
{-0.5f, -0.5f, -0.5f},

{-0.5f, -0.5f, 0.5f},
{0.5f, -0.5f, 0.5f},
{0.5f, 0.5f, 0.5f},
{0.5f, 0.5f, 0.5f},
{-0.5f, 0.5f, 0.5f},
{-0.5f, -0.5f, 0.5f},

{-0.5f, 0.5f, 0.5f},
{-0.5f, 0.5f, -0.5f},
{-0.5f, -0.5f, -0.5f},
{-0.5f, -0.5f, -0.5f},
{-0.5f, -0.5f, 0.5f},
{-0.5f, 0.5f, 0.5f},

 {0.5f, 0.5f, 0.5f},
 {0.5f, 0.5f, -0.5f},
 {0.5f, -0.5f, -0.5f},
 {0.5f, -0.5f, -0.5f},
 {0.5f, -0.5f, 0.5f},
 {0.5f, 0.5f, 0.5f},

{-0.5f, -0.5f, -0.5f},
 {0.5f, -0.5f, -0.5f},
 {0.5f, -0.5f, 0.5f},
 {0.5f, -0.5f, 0.5f},
{-0.5f, -0.5f, 0.5f},
{-0.5f, -0.5f, -0.5f},

{-0.5f, 0.5f, -0.5f},
 {0.5f, 0.5f, -0.5f},
 {0.5f, 0.5f, 0.5f},
 {0.5f, 0.5f, 0.5f},
{-0.5f, 0.5f, 0.5f},
{-0.5f, 0.5f, -0.5f}
};
float robot_maxt[36][3] = {};

AXBOpenGLWidget::AXBOpenGLWidget(QWidget *parent) : QOpenGLWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus);
    //定时重画
    connect(&timer,SIGNAL(timeout()),this,SLOT(on_timeout()));
    timer.start(100);

    //Camera position
    cameraPos = QVector3D(0.0f, 0.0f, 100.0f);
    //Camera direction
    cameraTarget = QVector3D(0.0f, 0.0f, 0.0f);
    cameraDirection = QVector3D(cameraPos - cameraTarget);
    cameraDirection.normalize();
    //Right axis
    up = QVector3D(0.0f, 1.0f, 0.0f);
    cameraRight = QVector3D::crossProduct(up, cameraDirection);
    cameraRight.normalize();
    //Up axis
    cameraUp = QVector3D::crossProduct(cameraDirection, cameraRight);
    cameraFront=QVector3D(0.0, 0.0, -1.0);


    Cuboid space;

    // 方便八叉树划分，空间坐标原点定于几何中心
    space.m_center = { 0.f, 0.f, 0.f };

    // 设置半长
    space.m_halflength = 50.f;
    space.m_halfwidth = 50.f;
    space.m_halfheight = 1.5f;

    // 设置旋转轴，就是法向量
    space.m_axeX = { 1.f, 0.f, 0.f };
    space.m_axeY = { 0.f, 1.f, 0.f };
    space.m_axeZ = { 0.f, 0.f, 1.f };
    //建立一颗八叉树
    OctreeNode* octree = OctreeNode::BuildOctree(space.m_center, space.m_halflength, space.m_halfwidth, space.m_halfheight, 4);
    CELLTimestamp timer;
    CELLTimestamp timer3;
    CELLTimestamp timer2;
    float timer_answer = 0.0f;
    float timer_part = 0.0f;
    float timer_point = 0.0f;
    srand((unsigned)time(NULL));//这里以当前时间为种子
#ifdef DEBUG
    int count = 0;
#endif //DEBUG
   for (int i = 0; i < 100000; ++i)
   {
        Point* tmp = new Point();
        int length = space.m_halflength * 200;
        int width = space.m_halfwidth * 200;
        int height = space.m_halfheight * 200;
        tmp->m_x = static_cast<float>(rand() % length - space.m_halflength*100)/100;
        tmp->m_y = static_cast<float>(rand() % width - space.m_halfwidth*100)/100;
        tmp->m_z = static_cast<float>(rand() % height - space.m_halfheight*100)/100;
        QVector3D point{tmp->m_x,tmp->m_y, tmp->m_z};
        //插入opengl中
        cubePositions.push_back(point);
        timer.update();
        OctreeNode::InsertObject(octree, tmp);
        timer_answer += timer.getElapsedTimeInMilliSec();
   }

   std::cout << "Insert consume time = " << timer_answer << "ms" << std::endl;

    //产生机器人
    Cuboid robot;

    robot.m_center = { 0.f, 0.f, 0.f };
    // 设置机器人大小
    robot.m_halflength = 0.5f;
    robot.m_halfwidth = 0.25f;
    robot.m_halfheight = 0.2f;

    // 设置机器人方向
    robot.m_axeX = { 1.f, 0.f, 0.f };
    robot.m_axeY = { 0.f, 1.f, 0.f };
    robot.m_axeZ = { 0.f, 0.f, 1.f };
    robot.m_R2 = robot.m_halflength * robot.m_halflength + robot.m_halfwidth * robot.m_halfwidth;


    for (int i = 0; i < 1; ++i) {
        float maxdelta = pow(robot.m_halflength * robot.m_halflength + robot.m_halfwidth * robot.m_halfwidth, 0.5);
        //随机在空间中产生位置
        robot.m_center.m_x = static_cast<float>(((rand() % (int)((space.m_halflength - maxdelta) * 200)) + maxdelta * 100 - space.m_halflength * 100)) / 100;//根据需要设置除数
        robot.m_center.m_y = static_cast<float>(((rand() % (int)((space.m_halfwidth - maxdelta) * 200)) + maxdelta * 100 - space.m_halfwidth * 100)) / 100;
#ifdef DEBUG
        std::cout << "robot [x] ; " << robot.m_center.m_x << std::endl;
        std::cout << "robot [y] ; " << robot.m_center.m_y << std::endl;
#endif // DEBUG
        robot.m_center.m_z = -1.3f;
        QVector3D point{robot.m_center.m_x,robot.m_center.m_y, robot.m_center.m_z};
        cameraTarget = point;
        //插入opengl中
        cubePositions.push_back(point);
        //随机产生角度
        float rotation = static_cast<float>(rand() % 3000) / 1000;
        m_rotation = rotation/3.14*180;
        robot.setAxe(rotation);
#ifdef DEBUG
        std::cout << "robot[rotation] = " << rotation << std::endl;
#endif // DEBUG
        //测试
        timer2.update();
        getCollisionOctreeNode(octree, robot);
        timer_part += timer2.getElapsedTimeInMilliSec();
        for (int i = 0; i < collisionnode.size(); ++i) {
            int c = 0;
            Point* real = collisionnode[i]->m_pointslist;
            while (collisionnode[i]->m_pointslist != nullptr)
            {
                collisionnode[i]->m_pointslist = collisionnode[i]->m_pointslist->m_next;
                ++c;
            }
            collisionnode[i]->m_pointslist = real;
            std::cout << "collisionnode[" << i << "] = " << c << std::endl;
        }

        timer3.update();
        getCollisionPoint(collisionnode, robot);
        timer_point += timer3.getElapsedTimeInMilliSec();

        //if (getCollisionPoint(collisionnode, robot))
        //    std::cout << "Collision!!!" << std::endl;
        //else std::cout << "No collision." << std::endl;

        //把结果集传递给opengl,让发生碰撞的改变颜色
        for (const auto& point : results)
        {
           float x = point->m_x;
           float y = point->m_y;
           float z = point->m_z;
           QVector3D item{x,y,z};
           ansset.push_back(item);
           std::cout<<"has insert ansset"<<std::endl;
        }
#ifdef DEBUG
        for (auto point : results)
        {
            std::cout << "point_x = " << point->m_x << std::endl;
            std::cout << "point_y = " << point->m_y << std::endl;
            ++count;
        }
        std::cout << std::endl;
#endif // DEBUG
        results.clear();
    }
#ifdef DEBUG
    std::cout << "count = " << count << std::endl;
#endif // DEBUG
    std::cout << "NodeCollision consume time = " << timer_part << "ms" << std::endl;
    std::cout << "PointCollision consume time = " << timer_point << "ms" << std::endl;
    std::cout << "Total consume time = " << (timer_part + timer_point)*100 << "ms" << std::endl;
    delete octree;
}

AXBOpenGLWidget::~AXBOpenGLWidget()
{
    //析构时释放对象
    makeCurrent();
    glDeleteBuffers(1,&VBO);
    glDeleteBuffers(1,&EBO);
    glDeleteVertexArrays(1,&VAO);
    doneCurrent();
}

void AXBOpenGLWidget::drawShape(AXBOpenGLWidget::Shape shape)
{
    m_shape=shape;
    update();
}

//线框渲染还是点渲染
void AXBOpenGLWidget::setWirefame(bool wireframe)
{
    makeCurrent();
    if(wireframe)
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    else
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    update();
    doneCurrent();
}

void AXBOpenGLWidget::initializeGL()
{
    initializeOpenGLFunctions();

    //创建VBO和VAO对象，并赋予ID
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);


    //绑定VBO和VAO对象
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    //为当前绑定到target的缓冲区对象创建一个新的数据存储。
    //如果data不是NULL，则使用来自此指针的数据初始化数据存储
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    //告知显卡如何解析缓冲里的属性值
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    //开启VAO管理的第一个属性值
    glEnableVertexAttribArray(0);
    //解除VBO绑定
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    bool success;
    //添加着色器
    shaderProgram.addShaderFromSourceFile(QOpenGLShader::Vertex,":/shaders/shapes.vert");
    shaderProgram.addShaderFromSourceFile(QOpenGLShader::Fragment,":/shaders/shapes.frag");
    success=shaderProgram.link();
    if(!success)
        qDebug()<<"ERR:"<<shaderProgram.log();


    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    //绑定着色器
    shaderProgram.bind();
    //解除VAO绑定
    glBindVertexArray(0);

    //投影转换，实现透视投影
    QMatrix4x4 projection;
    projection.perspective(45,(float)width()/height(),0.1,300);
    shaderProgram.setUniformValue("projection", projection);
}

void AXBOpenGLWidget::resizeGL(int w, int h)
{
    Q_UNUSED(w);Q_UNUSED(h);
    //glViewport(0, 0, w, h);
}

void AXBOpenGLWidget::paintGL()
{
    //坐标变换，从局部坐标变换到世界坐标系中，主要就是 旋转 缩放 移动
    QMatrix4x4 model;
    //视角变化，实现视角的变换，决定相机放置的位置，这个也是内部算法帮我们处理了
    QMatrix4x4 view;


    //视角设置   相机位置   相机目标点               辅助叉乘向量
    view.lookAt(cameraPos, cameraPos+cameraFront,QVector3D(0.0, 1.0, 0.0));






    //背景绘制
    glClearColor(0.f, 0.f, 0.f, 1.0f);
    //深度缓冲，是否遮挡
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glClear(GL_COLOR_BUFFER_BIT);

    shaderProgram.bind();

    //绑定VAO
    glBindVertexArray(VAO);

    QVector4D robot;
    switch (m_shape) {
    case ALL:
        shaderProgram.setUniformValue("view", view);
        //这些是点，最后一个是机器人
        for (int i = 0; i < cubePositions.size()-1; ++i) {
            //标准化
            QVector4D color;
            model.setToIdentity();
            //位移
            model.translate(cubePositions[i]);
            //旋转 角度制, 角度，和旋转轴，对本例来说，旋转轴设计成 z 轴即可
            model.rotate(0.0f, 0.0f, 0.0f,1.0f);
            //缩放
            model.scale(0.01f,0.01f,0.01f);
            shaderProgram.setUniformValue("model", model);
            color = {255,255,255,1};
            shaderProgram.setUniformValue("color",color);
            glDrawArrays(GL_TRIANGLES,0,36);
        }
        //绘制机器人
        model.setToIdentity();


        //位移
        model.translate(cubePositions[cubePositions.size()-1]);
        //旋转
        model.rotate(m_rotation,0,0,1);
        //缩放
        model.scale(1.f,0.5f,0.4f);

        shaderProgram.setUniformValue("model",model);
        robot = {255.f,255.f,255.f,1.0f};
        shaderProgram.setUniformValue("color",robot);
        glDrawArrays(GL_TRIANGLES,0,36);

        break;
    case COLLISION:
        for(int i = 0; i < ansset.size(); ++i){
            QVector4D color;
            model.setToIdentity();
            //位移
            model.translate(ansset[i]);

            //旋转 角度制, 角度，和旋转轴，对本例来说，旋转轴设计成 z 轴即可
            //model.rotate(0.0f, 0.0f, 0.0f,1.0f);
            //缩放
            model.scale(0.01f,0.01f,0.01f);
            shaderProgram.setUniformValue("model", model);
            color = {255,255,0,1};
            shaderProgram.setUniformValue("color",color);
            glDrawArrays(GL_TRIANGLES,0,36);
            std::cout<<"has insert ansset"<<std::endl;
        }
        //绘制机器人
        model.setToIdentity();


        //位移
        model.translate(cubePositions[cubePositions.size()-1]);
        //旋转
        model.rotate(m_rotation,0,0,1);
        //缩放
        model.scale(1.f,0.5f,0.4f);

        shaderProgram.setUniformValue("model",model);
        robot = {255.f,255.f,255.f,1.0f};
        shaderProgram.setUniformValue("color",robot);
        glDrawArrays(GL_TRIANGLES,0,36);

    default:
        break;
    }
}
#include <QKeyEvent>
void AXBOpenGLWidget::keyPressEvent(QKeyEvent *event)
{
    float cameraSpeed=10*TIMEOUTMSEC/1000.0;
    switch (event->key()) {

    case Qt::Key_W: cameraPos += cameraSpeed * cameraFront;break;
    case Qt::Key_S: cameraPos -= cameraSpeed * cameraFront;break;
    case Qt::Key_D: cameraPos += cameraSpeed * cameraRight;break;
    case Qt::Key_A: cameraPos -= cameraSpeed * cameraRight;break;
    default:
        break;
    }

    shaderProgram.bind();
    update();
}
void AXBOpenGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    static float yaw=-90;
    static float pitch=0;
    static QPoint lastPos(width()/2,height()/2);
    auto currentPos=event->pos();
    deltaPos=currentPos-lastPos;
    lastPos=currentPos;
    float sensitivity = 0.1f; // change this value to your liking
    deltaPos *= sensitivity;
    yaw += deltaPos.x();
    pitch -= deltaPos.y();// reversed since y-coordinates go from bottom to top
    //qDebug()<<deltaPos.x()<<","<<deltaPos.y();
    if(pitch > 89.0f) pitch = 89.0f;
    if(pitch < -89.0f) pitch = -89.0f;
    cameraFront.setX(cos(yaw*PI/180) * cos(pitch*PI/180));
    cameraFront.setY(sin(pitch*PI/180));
    cameraFront.setZ(sin(yaw*PI/180) * cos(pitch*PI/180));
    cameraFront.normalize();
    update();
}

void AXBOpenGLWidget::wheelEvent(QWheelEvent *event)
{
    if(fov >= 1.0f && fov <= 75.0f)
        fov -= event->angleDelta().y()/120;//一步是120
    if(fov <= 1.0f) fov = 1.0f;
    if(fov >= 75.0f) fov = 75.0f;
    update();
}


void AXBOpenGLWidget::on_timeout()
{
    update();
}
