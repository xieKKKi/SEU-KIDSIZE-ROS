#include <algorithm>
#include "RobotGL.hpp"

#define NORMAL_SIZE_AXE 1.0f
#define DRAW_LIST_AXE 10
#define DRAW_LIST_U_AXE 11

static GLfloat m_mat_diff_env[] = { 0.0f, 0.0f, 1.0f, 1.0f };
static GLfloat m_mat_diff_normal[] = { 1.0f, 0.6f, 0.2f, 1.0f };

using namespace std;
using namespace robot;

RobotGL::RobotGL(BonePtr b, const robot::JointMap &jmap) : main_bone_(b)
{
  setMinimumSize(600, 600);
  m_Rotate_X = -45.0;
  m_Rotate_Y = -36.0;

  m_transX = 0.5f;
  m_transY = 0.5f;
  m_transZ = 1.0f;

  m_KeyState = 0;
  m_mousePoint = QPoint(300, 300);
  setMouseTracking(false);
  joints_deg_.clear();

  for (auto &j : jmap)
  {
    joints_deg_[j.second->jid] = j.second->current_deg;
  }
}

void RobotGL::turn_joint(const std::map<int, float> &degs)
{
  for (auto &d : degs)
  {
    joints_deg_[d.first] = d.second;
  }

  this->update();
}

void RobotGL::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  setUserView();
  // draw_3d_ground();
  // draw_3d_axe(NORMAL_SIZE_AXE);
  draw_3d_bone(main_bone_);
  glFlush();
}

void RobotGL::draw_3d_bone(BonePtr b)
{
  if (!b)
  {
    return;
  }

  float r_b = 0.01;
  float r_j = 0.02;
  float j_l = 0.04;
  glRotatef(b->cr[0], 1.0f, 0.0f, 0.0f);
  glRotatef(b->cr[1], 0.0f, 1.0f, 0.0f);
  glRotatef(b->cr[2], 0.0f, 0.0f, 1.0f);
  glTranslatef(-b->cp[0], -b->cp[1], -b->cp[2]);

  glPushMatrix();
  draw_3d_cylinder(b->length, r_b, r_b);

  for (auto &j : b->joints)
  {
    glPushMatrix();

    if (j->can_turn)
    {
      glTranslatef(-j->cp[0], -j->cp[1], -j->cp[2]);
      glRotatef(j->cr[0], 1.0f, 0.0f, 0.0f);
      glRotatef(j->cr[1], 0.0f, 1.0f, 0.0f);
      glRotatef(j->cr[2], 0.0f, 0.0f, 1.0f);

      if (j->next_bone)
      {
        glTranslatef(0.0f, 0.0f, r_j);
        draw_3d_cylinder(j_l, r_j, r_j);
      }
    }
    else
    {
      glTranslatef(-j->cp[0], -j->cp[1], -j->cp[2]);
      draw_3d_sphere(r_j);
    }

    glPopMatrix();
  }

  glPopMatrix();

  for (auto &j : b->joints)
  {
    glPushMatrix();
    glTranslatef(-j->cp[0], -j->cp[1], -j->cp[2]);
    glRotatef(j->cr[0], 1.0f, 0.0f, 0.0f);
    glRotatef(j->cr[1], 0.0f, 1.0f, 0.0f);
    glRotatef(j->cr[2], 0.0f, 0.0f, 1.0f);

    if (j->can_turn)
    {
      glRotatef(joints_deg_[j->jid] + j->init_deg, 0.0f, 0.0f, 1.0f);
      glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    }

    if (j->next_bone)
    {
      draw_3d_bone(j->next_bone);
    }

    glPopMatrix();
  }
}

void RobotGL::setUserView()
{
  int w, h;
  w = this->size().width();
  h = this->size().height();
  glViewport(0, 0, (GLfloat)w, (GLfloat)h);
  gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 0.1, 20.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  float g_look[3];
  float rad_xy = float(3.1415927f * m_Rotate_X / 180.0f);
  float rad_z = float(3.1415927f * m_Rotate_Y / 180.0f);

  g_look[0] = float(m_transX - 100 * cos(rad_xy));
  g_look[1] = float(m_transY + 100 * sin(rad_xy));
  g_look[2] = 100 * tan(rad_z);

  gluLookAt(m_transX, m_transY, m_transZ, g_look[0], g_look[1], g_look[2], 0.0, 0.0, 1.0);
}

void RobotGL::initializeGL()
{
  initializeOpenGLFunctions();
  GLfloat mat_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
  GLfloat mat_shininess[] = { 50.0f };
  GLfloat mat_emission[] = { 0.2f, 0.2f, 0.2f, 1.0f };

  GLfloat light_ambient[] = { 0.1f, 0.1f, 0.1f, 1.0f };
  GLfloat light_position[] = { -1.0f, 1.0f, -1.0f, 0.0f };
  GLfloat white_light[] = { 1.0f, 1.0f, 1.0f, 1.0f };

  GLfloat lmodel_ambient[] = { 0.1f, 0.1f, 0.1f, 1.0f };

  GLfloat light1_ambient[] = { 0.1f, 0.1f, 0.1f, 1.0f };
  GLfloat light1_position[] = { 1.0f, -1.0f, 1.0f, 0.0f };
  GLfloat light1_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
  GLfloat light1_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };

  glClearColor(0.8f, 1.0f, 1.0f, 0.0f);
  glShadeModel(GL_SMOOTH);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
  glMaterialfv(GL_FRONT, GL_EMISSION, mat_emission);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, m_mat_diff_normal);

  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);
  glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);

  glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
  glLightfv(GL_LIGHT1, GL_SPECULAR, light1_specular);
  glLightfv(GL_LIGHT1, GL_AMBIENT, light1_ambient);

  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_FRONT_FACE);
  glFrontFace(GL_CCW);

  quad_obj = NULL;

  if (quad_obj == NULL)
  {
    quad_obj = gluNewQuadric();
  }

  gluQuadricDrawStyle(quad_obj, GLU_FILL);
  gluQuadricNormals(quad_obj, GLU_SMOOTH);
  gluQuadricOrientation(quad_obj, GLU_INSIDE);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  init_3d_model();
}

void RobotGL::init_3d_model()
{
  // init normal axe
  glNewList(DRAW_LIST_AXE, GL_COMPILE);
  draw_3d_cylinder(NORMAL_SIZE_AXE, NORMAL_SIZE_AXE / 200.0f, NORMAL_SIZE_AXE / 200.0f);
  glPushMatrix();
  glTranslatef(0.0f, 0.0f, -NORMAL_SIZE_AXE);
  draw_3d_cylinder(NORMAL_SIZE_AXE / 10.0f, 0.0f, NORMAL_SIZE_AXE / 100.0f);
  glPopMatrix();
  glEndList();
}

void RobotGL::draw_3d_cylinder(float length, float r_top, float r_button)
{
  glPushMatrix();
  glTranslatef(0.0f, 0.0f, -length);
  gluCylinder(quad_obj, r_top, r_button, length, 20, 10);
  gluQuadricOrientation(quad_obj, GLU_OUTSIDE);
  gluDisk(quad_obj, 0, r_top, 20, 10);
  glTranslatef(0.0f, 0.0f, length);
  gluQuadricOrientation(quad_obj, GLU_INSIDE);
  gluDisk(quad_obj, 0, r_button, 20, 10);
  glPopMatrix();
}

void RobotGL::draw_3d_sphere(float raduis)
{
  gluSphere(quad_obj, raduis, 20.0f, 20.0f);
}

void RobotGL::draw_3d_axe(float length)
{
  glPushMatrix();
  glRotatef(90, 0, 0, 1);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, m_mat_diff_env);

  glPushMatrix();
  glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
  glTranslatef(0.0f, 0.0f, length / 2.0f);
  glCallList(DRAW_LIST_AXE);  // z
  glPopMatrix();
  glPushMatrix();
  glTranslatef(0.0f, -length / 2.0f, 0.0f);
  glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
  glCallList(DRAW_LIST_AXE);  // y
  glPopMatrix();
  glPushMatrix();
  glTranslatef(-length / 2.0f, 0.0f, 0.0f);
  glRotatef(-90.0f, 0.0f, 1.0f, 0.0f);
  glCallList(DRAW_LIST_AXE);  // x
  glPopMatrix();

  void *font = GLUT_BITMAP_HELVETICA_18;
  glRasterPos3f(length / 2.0f - 0.08f, 0.0f, 0.0f);
  glutBitmapCharacter(font, 'X');
  glRasterPos3f(0.0f, length / 2.0f - 0.08f, 0.0f);
  glutBitmapCharacter(font, 'Y');
  glRasterPos3f(0.0f, 0.0f, length / 2.0f - 0.08f);
  glutBitmapCharacter(font, 'Z');
  glMaterialfv(GL_FRONT, GL_DIFFUSE, m_mat_diff_normal);
  glPopMatrix();
}

void RobotGL::draw_3d_ground(float width, float div)
{
  glMaterialfv(GL_FRONT, GL_DIFFUSE, m_mat_diff_env);
  glPushAttrib(GL_CURRENT_BIT);
  glEnable(GL_BLEND);
  glPushMatrix();
  glTranslatef(0, 0.0f, 0);
  float size0 = width * 2;
  glBegin(GL_LINES);

  for (float x = -size0; x <= size0; x += div)
  {
    glVertex3f(x, -size0, 0);
    glVertex3f(x, size0, 0);
  }

  for (float y = -size0; y <= size0; y += div)
  {
    glVertex3f(-size0, y, 0);
    glVertex3f(size0, y, 0);
  }

  glEnd();
  glPopMatrix();
  glDisable(GL_BLEND);
  glPopAttrib();
  glMaterialfv(GL_FRONT, GL_DIFFUSE, m_mat_diff_normal);
}

void RobotGL::mousePressEvent(QMouseEvent *event)
{
  this->setFocusPolicy(Qt::StrongFocus);

  if (event->button() == Qt::LeftButton)
  {
    m_mousePoint = event->pos();
    m_IsPressed = true;
  }
}

void RobotGL::mouseReleaseEvent(QMouseEvent *event)
{
  m_IsPressed = false;
}

void RobotGL::mouseMoveEvent(QMouseEvent *event)
{
  QPoint p;

  if (m_IsPressed)
  {
    p = event->pos();
    m_Rotate_X += (p.x() - m_mousePoint.x()) / 3.0f;
    m_Rotate_Y -= (p.y() - m_mousePoint.y()) / 3.0f;
    m_mousePoint = p;

    if (m_Rotate_Y > 90.0f)
    {
      m_Rotate_Y = 90.0f;
    }
    else if (m_Rotate_Y < -90.0f)
    {
      m_Rotate_Y = -90.0f;
    }

    this->update();
  }
}

void RobotGL::wheelEvent(QWheelEvent *event)
{
  float speed = 0.2f;
  float rad_xy = 3.1415927f * m_Rotate_X / 180.0f;
  float rad_z = 3.1415927f * m_Rotate_Y / 180.0f;

  if (event->delta() > 0)
  {
    m_transX -= cos(rad_xy) * speed / 3.0f;
    m_transY += sin(rad_xy) * speed / 3.0f;
    m_transZ += tan(rad_z) * speed / 3.0f;
  }
  else
  {
    m_transX += cos(rad_xy) * speed / 3.0f;
    m_transY -= sin(rad_xy) * speed / 3.0f;
    m_transZ -= tan(rad_z) * speed / 3.0f;
  }

  this->update();
}

void RobotGL::keyPressEvent(QKeyEvent *event)
{
  float speed = 0.1f;
  float rad_xy = 3.1415927f * m_Rotate_X / 180.0f;

  switch (event->key())
  {
    case Qt::Key_W:
    case Qt::Key_Up:
      m_KeyState = 1;
      break;

    case Qt::Key_S:
    case Qt::Key_Down:
      m_KeyState = 2;
      break;

    case Qt::Key_A:
    case Qt::Key_Left:
      m_KeyState = 3;
      break;

    case Qt::Key_D:
    case Qt::Key_Right:
      m_KeyState = 4;
      break;

    case Qt::Key_PageUp:
      m_KeyState = 5;
      break;

    case Qt::Key_PageDown:
      m_KeyState = 6;
      break;

    case Qt::Key_Q:
      m_KeyState = 7;
      break;

    case Qt::Key_E:
      m_KeyState = 8;
      break;

    case Qt::Key_R:
      m_KeyState = 9;
      break;

    case Qt::Key_F:
      m_KeyState = 10;
      break;

    case Qt::Key_T:
      m_KeyState = 11;
      break;
  }

  if (m_KeyState == 1)
  {
    m_transX -= cos(rad_xy) * speed / 3.0f;
    m_transY += sin(rad_xy) * speed / 3.0f;
  }

  if (m_KeyState == 2)
  {
    m_transX += cos(rad_xy) * speed / 3.0f;
    m_transY -= sin(rad_xy) * speed / 3.0f;
  }

  if (m_KeyState == 3)
  {
    m_transX -= sin(rad_xy) * speed / 3.0f;
    m_transY -= cos(rad_xy) * speed / 3.0f;
  }

  if (m_KeyState == 4)
  {
    m_transX += sin(rad_xy) * speed / 3.0f;
    m_transY += cos(rad_xy) * speed / 3.0f;
  }

  if (m_KeyState == 5)
  {
    m_transZ += speed;
  }

  if (m_KeyState == 6)
  {
    m_transZ -= speed;
  }

  if (m_KeyState == 9)
  {
    m_transX = 1;
    m_transY = 0;
    m_transZ = 0.4;
    m_Rotate_X = 0.0;
    m_Rotate_Y = 0.0;
  }

  if (m_KeyState == 10)
  {
    m_transX = 0;
    m_transY = 1;
    m_transZ = 0.4;
    m_Rotate_X = -90.0;
    m_Rotate_Y = 0.0;
  }

  if (m_KeyState == 11)
  {
    m_transX = 0;
    m_transY = 0;
    m_transZ = 1.2;
    m_Rotate_X = 0.0;
    m_Rotate_Y = 90.0;
  }

  this->update();
}

void RobotGL::keyReleaseEvent(QKeyEvent *event)
{
  m_KeyState = 0;
  QWidget::keyReleaseEvent(event);
}
