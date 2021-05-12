#ifndef __ROBOT_GL_HPP
#define __ROBOT_GL_HPP

#include <QtWidgets>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <GL/glut.h>
#include <list>
#include <map>
#include <robot/robot.hpp>

class RobotGL: public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    RobotGL(robot::BonePtr b, const robot::JointMap &jmap);
    void turn_joint(const std::map<int, float> &degs);

protected:
    void initializeGL();
    void paintGL();

    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
    virtual void wheelEvent(QWheelEvent *event);
    virtual void keyPressEvent(QKeyEvent *event);
    virtual void keyReleaseEvent(QKeyEvent *event);

private:
    void init_3d_model();
    void draw_3d_sphere(float raduis);
    void draw_3d_cylinder(float length, float r_top, float r_button);
    void draw_3d_axe(float length);
    void draw_3d_ground(float width = 3.0f, float div = 0.25f);
    void draw_3d_bone(robot::BonePtr b);
    void setUserView();

    std::map<int, float> joints_deg_;
    float m_Rotate_X, m_Rotate_Y;
    float m_transX, m_transY, m_transZ;
    int m_KeyState;
    QPoint m_mousePoint;
    bool m_IsPressed;
    robot::BonePtr main_bone_;
    GLUquadricObj   *quad_obj;
};

#endif
