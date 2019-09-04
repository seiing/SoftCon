#include "DrawFunctions.h"
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif
void
GUI::
DrawWorld()
{
	glEnable(GL_LIGHTING);  
	glBegin(GL_QUADS);

	double ground_y = -2.0;
	double width_x = 100.0;
	double width_z = 100.0;
	double height_y = 200.0;

	for(int x=-width_x;x<width_x;x+=1)
	{
		for(int z=-width_z;z<width_z;z+=1)
		{
			glColor3f(100.0/255.0,100.0/255.0,100.0/255.0);
			glNormal3d(0,1,0);
			glVertex3f(x,ground_y,z+1.0);
			glVertex3f(x+1.0,ground_y,z+1.0);
			glVertex3f(x+1.0,ground_y,z);
			glVertex3f(x,ground_y,z);
		}
	}
	glEnd();

	glBegin(GL_QUADS);
	glNormal3d(0,0,1);
	glVertex3f(-width_x,ground_y,-width_z);
	glVertex3f(width_x,ground_y,-width_z);
	glVertex3f(width_x,height_y,-width_z);
	glVertex3f(-width_x,height_y,-width_z);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3d(0,0,-1);
	glVertex3f(-width_x,ground_y,width_z);
	glVertex3f(-width_x,height_y,width_z);
	glVertex3f(width_x,height_y,width_z);
	glVertex3f(width_x,ground_y,width_z);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3d(1,0,0);
	glVertex3f(-width_x,ground_y,-width_z);
	glVertex3f(-width_x,height_y,-width_z);
	glVertex3f(-width_x,height_y,width_z);
	glVertex3f(-width_x,ground_y,width_z);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3d(-1,0,0);
	glVertex3f(width_x,ground_y,-width_z);
	glVertex3f(width_x,ground_y,width_z);
	glVertex3f(width_x,height_y,width_z);
	glVertex3f(width_x,height_y,-width_z);
	glEnd();

    glDisable(GL_LIGHTING);
}
void 
GUI::
DrawCharacter(Octopus* octopus,const Eigen::VectorXd& x,const Eigen::Vector3d& eye)
{
	glEnable(GL_LIGHTING); 

	const std::vector<Eigen::Vector3i>& contours = octopus->GetContours();
	std::vector<std::pair<int,double>> sorted_contours;
	const Eigen::VectorXd& v_n = octopus->GetVertexNormal();

	for(int i=0; i<contours.size(); i++)
	{
		const Eigen::Vector3i& c = contours[i];
		int idx0 = (c)[0];
		int idx1 = (c)[1];
		int idx2 = (c)[2];

		Eigen::Vector3d p0 = x.block<3,1>(3*idx0,0);
		Eigen::Vector3d p1 = x.block<3,1>(3*idx1,0);
		Eigen::Vector3d p2 = x.block<3,1>(3*idx2,0);

		Eigen::Vector3d center = 0.3333*(p0+p1+p2);

		sorted_contours.push_back(std::make_pair(i,(center-eye).norm()));
	}
	std::sort(sorted_contours.begin(), sorted_contours.end(), [](const std::pair<int, double>& A, const std::pair<int, double>& B){
		return A.second > B.second;
	});
	for(int i=0; i<sorted_contours.size(); i++)
	{
		const auto& c = contours[sorted_contours[i].first];
		int idx0 = (c)[0];
		int idx1 = (c)[1];
		int idx2 = (c)[2];

		Eigen::Vector3d p0 = x.block<3,1>(3*idx0,0);
		Eigen::Vector3d p1 = x.block<3,1>(3*idx1,0);
		Eigen::Vector3d p2 = x.block<3,1>(3*idx2,0);

		Eigen::Vector3d n0 = v_n.block<3,1>(3*idx0,0);
		Eigen::Vector3d n1 = v_n.block<3,1>(3*idx1,0);
		Eigen::Vector3d n2 = v_n.block<3,1>(3*idx2,0);

		GUI::DrawTriangle(p0,p1,p2,n0,n1,n2,Eigen::Vector4d(200.0/256.0,200.0/256.0,200.0/256.0,0.4));
	}
	glDisable(GL_LIGHTING);
}
void 
GUI::
DrawMuscles(const std::vector<Muscle*>& muscles,const Eigen::VectorXd& x)
{
	glEnable(GL_LIGHTING);

	std::vector<Eigen::Vector3d> colors ={
		Eigen::Vector3d(30/256.0, 158/256.0, 185/256.0),
		Eigen::Vector3d(254/256.0, 114/256.0, 121/256.0),
	};
	int count; 

	for(const auto& m : muscles)
	{
		for(const auto& seg : m->GetSegments())
		{
			double act = seg->GetActivationLevel();

			Eigen::Vector4d start_bary = seg->GetStartBarycentric();
			Eigen::Vector4d end_bary = seg->GetEndBarycentric();
			Eigen::Vector4i start_idx = seg->GetStartIdx();
			Eigen::Vector4i end_idx = seg->GetEndIdx();

			double a = seg->GetActivationLevel();

			Eigen::Vector3d p_start = start_bary[0]*x.block<3,1>(3*start_idx[0],0) + start_bary[1]*x.block<3,1>(3*start_idx[1],0) 
					+ start_bary[2]*x.block<3,1>(3*start_idx[2],0) + start_bary[3]*x.block<3,1>(3*start_idx[3],0);
			Eigen::Vector3d p_end = end_bary[0]*x.block<3,1>(3*end_idx[0],0) + end_bary[1]*x.block<3,1>(3*end_idx[1],0) 
					+ end_bary[2]*x.block<3,1>(3*end_idx[2],0) + end_bary[3]*x.block<3,1>(3*end_idx[3],0);

			Eigen::Vector4d color;
			color[0] = colors[count%2][0];
			color[1] = colors[count%2][1];
			color[2] = colors[count%2][2];
			color[3] = 2*a;

			GUI::DrawCylinder(p_start,p_end,0.003,color);	
		}
		count+=1;
	}

	glDisable(GL_LIGHTING);
}
void 
GUI::
DrawActivations(const double& x, const double& y,const double& length,const double& height,Muscle* muscle1,Muscle* muscle2)
{
	glDisable(GL_LIGHTING);

	GLint oldMode;
    glGetIntegerv(GL_MATRIX_MODE, &oldMode);
    glMatrixMode(GL_PROJECTION);

    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0, 0.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glRasterPos2f(x, y);

    auto act1 = muscle1->GetActivationLevels();
    auto act2 = muscle2->GetActivationLevels();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBegin(GL_POLYGON);
    glColor4f(0.8,0.8,0.8,0.75);
	glVertex2d(x,y);
	glVertex2d(x+length/act1.size()*(act1.size()-1),y);
	glVertex2d(x+length/act1.size()*(act1.size()-1),y+height);
	glVertex2d(x,y+height);
	glVertex2d(x,y);
	glDisable(GL_BLEND);
    glEnd();

    double base = y+0.5*height;

    // down
	for(int i=0; i<act1.size()-1; i++) {
		float x1,y1;
		float x2,y2;

		x1=x+length/act1.size()*i;
		x2=x+length/act1.size()*(i+1);

		y1=y+0.5*height-2.0*act1[i]*0.5*height; 
		y2=y+0.5*height-2.0*act1[i+1]*0.5*height;

		if(y1<y)
			y1 = y;
		if(y2<y)
			y2 = y;

		glColor3f(30/256.0, 158/256.0, 185/256.0);
		glBegin(GL_POLYGON);
	    glVertex2d(x1,base);
	    glVertex2d(x1,y1);
	    glVertex2d(x2,y2);
	    glVertex2d(x2,base);
	    glVertex2d(x1,base);
	    glEnd();
	}	
	// up
	for(int i=0; i<act2.size()-1; i++) {
		float x1,y1;
		float x2,y2;

		x1=x+length/act2.size()*i;
		x2=x+length/act2.size()*(i+1);

		y1=y+2.0*act2[i]*0.5*height + 0.5*height;
		y2=y+2.0*act2[i+1]*0.5*height + 0.5*height;

		if(y1<y)
			y1 = y;
		if(y2<y)
			y2 = y;

		glColor3f(254/256.0, 114/256.0, 121/256.0);
		glBegin(GL_POLYGON);
	    glVertex2d(x1,base);
	    glVertex2d(x2,base);
	    glVertex2d(x2,y2);
	    glVertex2d(x1,y1);
	    glVertex2d(x1,base);
	    glEnd();
	}	

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(oldMode);

    glEnable(GL_LIGHTING);    
}