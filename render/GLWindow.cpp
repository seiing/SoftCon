#include "GLWindow.h"
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif
std::vector<GLWindow*> GLWindow::mWindows;
std::vector<int> GLWindow::mWinIDs;
GLWindow::
GLWindow()
	:mCamera(new Camera()),mIsDrag(false),mPrevX(0),mPrevY(0),mDisplayTimeout(1.0/30.0)
{

}
GLWindow::
~GLWindow()
{
}
void
GLWindow::
InitWindow(int _w,int _h,const char* _name)
{
	mWindows.push_back(this);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA | GLUT_MULTISAMPLE | GLUT_ACCUM);
	glutInitWindowPosition(150, 100);
	glutInitWindowSize(_w, _h);
	mWinIDs.push_back(glutCreateWindow(_name));
	glutDisplayFunc(DisplayEvent);
	glutReshapeFunc(ReshapeEvent);
	glutKeyboardFunc(KeyboardEvent);
	glutMouseFunc(MouseEvent);
	glutMotionFunc(MotionEvent);
	glutTimerFunc(mDisplayTimeout, TimerEvent, 0);
	mScreenshotTemp.resize(4*_w*_h);
  	mScreenshotTemp2.resize(4*_w*_h);
}
inline GLWindow* GLWindow::current() {
  int id = glutGetWindow();
  for (unsigned int i = 0; i < mWinIDs.size(); i++) {
    if (mWinIDs.at(i) == id) {
      return mWindows.at(i);
    }
  }
  std::cout << "An unknown error occurred!" << std::endl;
  exit(0);
}
void
GLWindow::
DisplayEvent()
{
	current()->Display();
}
void
GLWindow::
KeyboardEvent(unsigned char key,int x,int y)
{
	current()->Keyboard(key,x,y);
}
void
GLWindow::
MouseEvent(int button, int state, int x, int y)
{
	current()->Mouse(button,state,x,y);
}
void
GLWindow::
MotionEvent(int x, int y)
{
	current()->Motion(x,y);
}
void
GLWindow::
ReshapeEvent(int w, int h)
{
	current()->Reshape(w,h);
}
void
GLWindow::
TimerEvent(int value)
{
	current()->Timer(value);
}