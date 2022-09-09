#ifndef AXBOPENGLWIDGET_H
#define AXBOPENGLWIDGET_H


#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QTimer>
#include <QTime>
#include <vector>
class AXBOpenGLWidget : public QOpenGLWidget,QOpenGLFunctions_3_3_Core
{
    Q_OBJECT
public:
    enum Shape{None,ALL,COLLISION};
    explicit AXBOpenGLWidget(QWidget *parent = nullptr);
    ~AXBOpenGLWidget();

    void drawShape(Shape shape);
    void setWirefame(bool wireframe);
protected:
    virtual void initializeGL();
    virtual void resizeGL(int w, int h);
    virtual void keyPressEvent(QKeyEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
    virtual void wheelEvent(QWheelEvent *event);
    virtual void paintGL();
    //void keyPressEvent(QKeyEvent *event);
signals:

public slots:
    void on_timeout();
private:
    Shape m_shape;
    QOpenGLShaderProgram shaderProgram;
    QOpenGLShaderProgram shaderProgramRobot;
    QOpenGLTexture * textureWall;
    QOpenGLTexture * textureSmile;
    QOpenGLTexture * textureSmall;
    std::vector<QVector3D> cubePositions; //没有碰撞的点的集合
    std::vector<QVector3D> ansset;        //发生碰撞的点的集合

    //相机设置
    QVector3D cameraPos;
    QVector3D cameraTarget;
    QVector3D cameraDirection;
    QVector3D up;
    QVector3D cameraRight;
    QVector3D cameraUp;
    QVector3D cameraFront;

    float m_rotation;
    QTimer timer;
    QTime m_time;
};



#endif // AXBOPENGLWIDGET_H
