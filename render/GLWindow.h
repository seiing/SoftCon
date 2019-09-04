#ifndef __GL_WINDOW_H_
#define __GL_WINDOW_H_
#include "Camera.h"
#include <vector>	
class Camera;
class GLWindow
{
public:
	GLWindow();
	~GLWindow();

	virtual void InitWindow(int _w,int _h,const char* _name);
	static void DisplayEvent();
	static void KeyboardEvent(unsigned char key,int x,int y);
	static void MouseEvent(int button, int state, int x, int y);
	static void MotionEvent(int x, int y);
	static void ReshapeEvent(int w, int h);
	static void TimerEvent(int value);
	
	static GLWindow* current();
	static std::vector<GLWindow*> mWindows;
	static std::vector<int> mWinIDs;

protected:
	virtual void Display() = 0;
	virtual void Keyboard(unsigned char key,int x,int y) = 0;
	virtual void Mouse(int button, int state, int x, int y) = 0;
	virtual void Motion(int x, int y) = 0;
	virtual void Reshape(int w, int h) = 0;
	virtual void Timer(int value) = 0;

protected:
	Camera*		mCamera;
	bool		mIsDrag;
	int 		mMouseType;
	int 		mPrevX,mPrevY;
	int 		mDisplayTimeout;

	// Screenshot
	std::vector<unsigned char> mScreenshotTemp;
	std::vector<unsigned char> mScreenshotTemp2;
};

#endif