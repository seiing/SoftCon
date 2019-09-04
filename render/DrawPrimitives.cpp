#include "DrawPrimitives.h"
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif
static GLUquadricObj *quadObj;
static void initQuadObj(void)
{
    quadObj = gluNewQuadric();
}
#define QUAD_OBJ_INIT { if(!quadObj) initQuadObj(); }
void 
GUI::
DrawTetrahedron(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& p3,const Eigen::Vector4d& color,const bool& draw_line)
{
	glEnable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(color[0],color[1],color[2],color[3]);

	if(draw_line) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	} else {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	Eigen::Vector3d center = 0.25*(p0+p1+p2+p3);

	Eigen::Vector3d normal = ((p1-p0).cross(p2-p0)).normalized();
	if((center-Eigen::Vector3d((p0+p1+p2)/3.0)).dot(normal)>0.0)
	{
		normal = ((p2-p0).cross(p1-p0)).normalized();
		glBegin(GL_TRIANGLES);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3f(p0[0],p0[1],p0[2]);
		glVertex3f(p2[0],p2[1],p2[2]);
		glVertex3f(p1[0],p1[1],p1[2]);
		glEnd();
	} else {
		glBegin(GL_TRIANGLES);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3f(p0[0],p0[1],p0[2]);
		glVertex3f(p1[0],p1[1],p1[2]);
		glVertex3f(p2[0],p2[1],p2[2]);
		glEnd();    
	}

	normal = ((p1-p0).cross(p3-p0)).normalized();
	if((center-Eigen::Vector3d((p0+p1+p3)/3.0)).dot(normal)>0.0)
	{
		normal = ((p3-p0).cross(p1-p0)).normalized();
		glBegin(GL_TRIANGLES);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3f(p0[0],p0[1],p0[2]);
		glVertex3f(p3[0],p3[1],p3[2]);
		glVertex3f(p1[0],p1[1],p1[2]);
		glEnd();
	} else {
		glBegin(GL_TRIANGLES);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3f(p0[0],p0[1],p0[2]);
		glVertex3f(p1[0],p1[1],p1[2]);
		glVertex3f(p3[0],p3[1],p3[2]);
		glEnd();
	}

	normal = ((p2-p0).cross(p3-p0)).normalized();
	if((center-Eigen::Vector3d((p0+p2+p3)/3.0)).dot(normal)>0.0)
	{
		normal = ((p3-p0).cross(p2-p0)).normalized();
		glBegin(GL_TRIANGLES);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3f(p0[0],p0[1],p0[2]);
		glVertex3f(p3[0],p3[1],p3[2]);
		glVertex3f(p2[0],p2[1],p2[2]);
		glEnd();
	} else {
		glBegin(GL_TRIANGLES);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3f(p0[0],p0[1],p0[2]);
		glVertex3f(p2[0],p2[1],p2[2]);
		glVertex3f(p3[0],p3[1],p3[2]);
		glEnd();
	}

	normal = ((p2-p1).cross(p3-p1)).normalized();
	if((center-Eigen::Vector3d((p1+p2+p3)/3.0)).dot(normal)>0.0)
	{
		normal = ((p3-p1).cross(p2-p1)).normalized();
		glBegin(GL_TRIANGLES);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3f(p1[0],p1[1],p1[2]);
		glVertex3f(p3[0],p3[1],p3[2]);
		glVertex3f(p2[0],p2[1],p2[2]);
		glEnd();
	} else {
		glBegin(GL_TRIANGLES);
		glNormal3d(normal[0],normal[1],normal[2]);
		glVertex3f(p1[0],p1[1],p1[2]);
		glVertex3f(p2[0],p2[1],p2[2]);
		glVertex3f(p3[0],p3[1],p3[2]);
		glEnd();
	}

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);
}
void 
GUI::
DrawTriangle(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,
	const Eigen::Vector3d& n0,const Eigen::Vector3d& n1,const Eigen::Vector3d& n2,
	const Eigen::Vector4d& color,const bool& draw_line)
{
	glEnable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(color[0],color[1],color[2],color[3]);

	if(draw_line) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	} else {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	glBegin(GL_TRIANGLES);
    glNormal3d(n0[0],n0[1],n0[2]);
    glVertex3f(p0[0],p0[1],p0[2]);
    glNormal3d(n1[0],n1[1],n1[2]);
    glVertex3f(p1[0],p1[1],p1[2]);
    glNormal3d(n2[0],n2[1],n2[2]);
    glVertex3f(p2[0],p2[1],p2[2]);
    glEnd();

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);	
}
void 
GUI::
DrawLine(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,
		const Eigen::Vector4d& color,const double& line_width)
{
	glEnable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(color[0],color[1],color[2],color[3]);
	glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(p0[0],p0[1],p0[2]);
    glVertex3f(p1[0],p1[1],p1[2]);
    glEnd();
	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);	
}
void 
GUI::
DrawPoint(const Eigen::Vector3d& p0,const double& size,const Eigen::Vector4d& color)
{
	glEnable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(color[0],color[1],color[2],color[3]);
	glPointSize(size);
	glBegin(GL_POINTS);
	glVertex3f(p0[0],p0[1],p0[2]);
	glEnd();
	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);
}
void 
GUI::
DrawCylinder(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const double& line_width,const Eigen::Vector4d& color)
{
	Eigen::Vector3d u(0,0,1);
    Eigen::Vector3d v = p0-p1;
    Eigen::Vector3d mid = 0.5*(p0+p1);
    double len = v.norm();
    v /= len;
    Eigen::Isometry3d T;
    T.setIdentity();
    Eigen::Vector3d axis = u.cross(v);
    axis.normalize();
    double angle = acos(u.dot(v));
    Eigen::Matrix3d w_bracket = Eigen::Matrix3d::Zero();
    w_bracket(0, 1) = -axis(2);
    w_bracket(1, 0) =  axis(2);
    w_bracket(0, 2) =  axis(1);
    w_bracket(2, 0) = -axis(1);
    w_bracket(1, 2) = -axis(0);
    w_bracket(2, 1) =  axis(0);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity()+(sin(angle))*w_bracket+(1.0-cos(angle))*w_bracket*w_bracket;
    T.linear() = R;
    T.translation() = mid;
    glPushMatrix();
    glMultMatrixd(T.data());
    glEnable(GL_BLEND);
    glColor4f(color[0],color[1],color[2],color[3]);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    GUI::DrawCylinder(line_width,len,6,6);
    glPopMatrix();
    glDisable(GL_BLEND);
}
void 
GUI::
DrawCylinder(double radius, double height, int slices, int stacks)
{
  glPushMatrix();

  // Graphics assumes Cylinder is centered at CoM
  // gluCylinder places base at z = 0 and top at z = height
  glTranslated(0.0, 0.0, -0.5*height);

  // Code taken from glut/lib/glut_shapes.c
  QUAD_OBJ_INIT;
  gluQuadricDrawStyle(quadObj, GLU_FILL);
  gluQuadricNormals(quadObj, GLU_SMOOTH);
  //gluQuadricTexture(quadObj, GL_TRUE);

  // glut/lib/glut_shapes.c
  
  gluSphere(quadObj,radius,stacks,stacks);
  gluCylinder(quadObj, radius, radius, height, slices, stacks);
  glTranslatef(0,0,height);
  gluSphere(quadObj,radius,stacks,stacks);

  glPopMatrix();
}
void 
GUI::
DrawTorus(const Eigen::Vector3d& center,const double& r,const Eigen::Vector3d& dir,const Eigen::Vector4d& color)
{
	glEnable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(color[0],color[1],color[2],color[3]);

	Eigen::Vector3d normDir = dir.normalized();
	glPushMatrix();
	glTranslatef(center[0],center[1],center[2]);
    glRotated(acos(normDir[2])*180/M_PI, -normDir[1], normDir[0], 0);
    glScalef(1.2f,1.0f,1.2f);
    glutSolidTorus(0.5*r,r,10,10);
	glPopMatrix();
	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);
}
void 
GUI::
DrawArrow3D(const Eigen::Vector3d& _pt, const Eigen::Vector3d& _vec,
            const double _thickness,const Eigen::Vector3d& color,
            const double _arrowThickness)
{
	glEnable(GL_LIGHTING);
    glColor3f(color[0],color[1],color[2]);
    Eigen::Vector3d normDir = _vec.normalized();
    double _length = _vec.norm();

    double arrowLength;
    if (_arrowThickness == -1)
    arrowLength = 4*_thickness;
    else
    arrowLength = 2*_arrowThickness;

    // draw the arrow body as a cylinder
    GLUquadricObj *c;
    c = gluNewQuadric();
    gluQuadricDrawStyle(c, GLU_FILL);
    gluQuadricNormals(c, GLU_SMOOTH);

    glPushMatrix();
    glTranslatef(_pt[0], _pt[1], _pt[2]);
    glRotated(acos(normDir[2])*180/M_PI, -normDir[1], normDir[0], 0);
    gluCylinder(c, _thickness, _thickness, _length-arrowLength, 16, 16);

    // draw the arrowhed as a cone
    glPushMatrix();
    glTranslatef(0, 0, _length-arrowLength);
    gluCylinder(c, arrowLength*0.5, 0.0, arrowLength, 10, 10);
    glPopMatrix();

    glPopMatrix();

    gluDeleteQuadric(c);
    glDisable(GL_LIGHTING);
}