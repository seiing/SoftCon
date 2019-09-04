#include "SimulationWindow.h"
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif
namespace p = boost::python;
namespace np = boost::python::numpy;
int main(int argc,char** argv)
{
	Py_Initialize();
	np::initialize();
	if( argc < 2 ) {
		SimulationWindow* simwindow = new SimulationWindow();
		glutInit(&argc, argv);
		simwindow->InitWindow(1920,1080,"OctoCon");
		glutMainLoop();
	} else {
		SimulationWindow* simwindow = new SimulationWindow(std::string(argv[1]));
		glutInit(&argc, argv);

		simwindow->InitWindow(1920,1080,"OctoCon");
		glutMainLoop();
	}
}
