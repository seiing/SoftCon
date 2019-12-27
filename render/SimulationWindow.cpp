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
SimulationWindow::
SimulationWindow()
	:GLWindow(),mCurFrame(0),mTotalFrame(0),mElapsedTime(0.0),mPlay(false),mIsNNLoaded(false)
{
	mDisplayTimeout = 33;

	initLights();

	mEnvironment = new Environment();

	mActions.resize(mEnvironment->GetActions().size());
	mActions.setZero();
	
	try {
		mm = p::import("__main__");
		mns = mm.attr("__dict__");
		sys_module = p::import("sys");

		p::str module_dir = (std::string(SOFTCON_DIR)+"/learn").c_str();
		sys_module.attr("path").attr("insert")(1, module_dir);

		p::exec("from run import Environment",mns);
		p::exec("from IPython import embed",mns);

		env_module = p::eval("Environment()",mns);
	}
	catch (const p::error_already_set&)
	{
		PyErr_Print();
	}
}
SimulationWindow::
SimulationWindow(std::string network_name)
	:SimulationWindow()
{
	try {
		mIsNNLoaded = true;

		p::object load = env_module.attr("loadNN");
		load(network_name);
		mEnvironment->Reset();
		mEnvironment->UpdateRandomTargetVelocity();
	} 
	catch (const p::error_already_set&)
	{
		PyErr_Print();
	}
}
Eigen::VectorXd
SimulationWindow::
GetActionFromNN(const Eigen::VectorXd& state)
{
	Eigen::VectorXd action = Eigen::VectorXd::Zero(mEnvironment->GetActions().size());

	if(!mIsNNLoaded)
		return action;
	p::object get_action;
	
	get_action = env_module.attr("get_action");
	p::tuple shape = p::make_tuple(state.rows());
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray state_np = np::empty(shape,dtype);
	float* dest = reinterpret_cast<float*>(state_np.get_data());
	for(int i =0;i<state.rows();i++)
		dest[i] = state[i];
	p::object temp = get_action(state_np);
	np::ndarray action_np = np::from_object(temp);
	
	float* srcs = reinterpret_cast<float*>(action_np.get_data());
	for(int i=0;i<action.rows();i++)
		action[i] = srcs[i];

	return action;
}
void
SimulationWindow::
initLights()
{
  static float ambient[]           	 = {0.4, 0.4, 0.4, 1.0};
  static float diffuse[]             = {0.4, 0.4, 0.4, 1.0};
  static float front_mat_shininess[] = {60.0};
  static float front_mat_specular[]  = {0.2, 0.2,  0.2,  1.0};
  static float front_mat_diffuse[]   = {0.2, 0.2, 0.2, 1.0};
  static float lmodel_ambient[]      = {0.2, 0.2,  0.2,  1.0};
  static float lmodel_twoside[]      = {GL_TRUE};

  GLfloat position[] = {0.0, 1.0, 1.0, 0.0};
  GLfloat position1[] = {0.0, 1.0, -1.0, 0.0};

  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, position);

  glLightModelfv(GL_LIGHT_MODEL_AMBIENT,  lmodel_ambient);
  glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

  glEnable(GL_LIGHT1);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT1, GL_POSITION, position1);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);

  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, front_mat_shininess);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  front_mat_specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   front_mat_diffuse);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glDisable(GL_CULL_FACE);
  glEnable(GL_NORMALIZE);

  glEnable(GL_FOG);

  GLfloat fogColor[] = {53.0/255.0,214.0/255.0,237.0/255.0,1.0};
  glFogfv(GL_FOG_COLOR,fogColor);
  glFogi(GL_FOG_MODE,GL_EXP);
  glFogf(GL_FOG_DENSITY,0.05);
  glFogf(GL_FOG_START,4.0);
  glFogf(GL_FOG_END,8.0);
}
void
SimulationWindow::
Display()
{
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	initLights();
	glEnable(GL_LIGHTING);

	mCamera->Apply();

	const Eigen::VectorXd& x = mEnvironment->GetSoftWorld()->GetPositions();
	const Eigen::Vector3d& eye = mCamera->GetEye();
	mEnvironment->GetOctopus()->SetVertexNormal();
	const std::vector<Muscle*>& muscles = mEnvironment->GetOctopus()->GetMuscles();

	GUI::DrawWorld();
	GUI::DrawMuscles(muscles,x);
	GUI::DrawCharacter(mEnvironment->GetOctopus(),x,eye);

	Eigen::Vector3d center_position = 
		x.block<3,1>(3*mEnvironment->GetOctopus()->GetCenterIndex(),0);
	Eigen::Vector3d forward_vector = mEnvironment->GetOctopus()->GetForwardVector(x);
	Eigen::Vector3d target_velocity = mEnvironment->GetTargetVelocity();
	Eigen::Vector3d average_velocity = mEnvironment->GetAverageVelocity();

	GUI::DrawArrow3D(center_position,average_velocity,0.01,Eigen::Vector3d(255.0/256,56.0/256,109.0/256));
	GUI::DrawArrow3D(center_position,target_velocity,0.01,Eigen::Vector3d(12.0/256.0,239.0/256.0,103.0/256.0));

	double init_x = 0.78;
	double init_y = 1.0;

	double length = 0.2;
	double height = 0.08;

	for(int i=0; i<8; i++) 
	{
		const auto& m1 = muscles[2*i];
		const auto& m2 = muscles[2*i+1];
		GUI::DrawActivations(init_x,init_y-(i+1)*1.2*height,length,height,m1,m2);
	}

	glutSwapBuffers();
}
void
SimulationWindow::
Keyboard(unsigned char key,int x,int y)
{
	switch(key)
	{
		case 27: exit(0); break;
		case ' ': {
			mPlay = !mPlay;
			if(mPlay)
				std::cout << "Play." << std::endl;
			else 
				std::cout << "Pause." << std::endl;
			break;
		}
		case 'r': {
			mEnvironment->Reset();
			break;
		}
		case 'k': {
			mPlay = true;
			const auto& muscles = mEnvironment->GetOctopus()->GetMuscles();
			mActions.resize(mEnvironment->GetActions().size());
			mActions.setZero();

			int cnt = 0;
			for(const auto& m : muscles)
			{
				mActions[cnt+0]=0.0;
				mActions[cnt+1]=0.7;
				mActions[cnt+2]=0.3;
				mActions[cnt+3]=5.0;
				cnt+=4;
			}

			mActions = mEnvironment->GetNormalizer()->RealToNorm(mActions);
			break;
		}
	}
	glutPostRedisplay();
}
void
SimulationWindow::
Mouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
	{
		mIsDrag = true;
		mMouseType = button;
		mPrevX = x;
		mPrevY = y;
	}
	else
	{
		mIsDrag = false;
		mMouseType = 0;
	}

	glutPostRedisplay();
}
void
SimulationWindow::
Motion(int x, int y)
{
	if (!mIsDrag)
		return;

	int mod = glutGetModifiers();
	if (mMouseType == GLUT_LEFT_BUTTON)
	{
		mCamera->Translate(x,y,mPrevX,mPrevY);
	}
	else if (mMouseType == GLUT_MIDDLE_BUTTON)
	{
		mCamera->Pan(x,y,mPrevX,mPrevY);
	}
	else if (mMouseType == GLUT_RIGHT_BUTTON)
	{
		mCamera->Rotate(x,y,mPrevX,mPrevY);
	}
	mPrevX = x;
	mPrevY = y;
	glutPostRedisplay();
}
void
SimulationWindow::
Reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	mCamera->Apply();
	glutPostRedisplay();
}
void
SimulationWindow::
Timer(int value)
{
	int ratio = mEnvironment->GetSimulationHz()/mEnvironment->GetControlHz();

	if(mPlay)
	{
		if(mIsNNLoaded) {
			int cur_phase = mEnvironment->GetPhase();
			Eigen::VectorXd action = GetActionFromNN(mEnvironment->GetStates());
			mEnvironment->SetActions(action);
			for(int i=0; i<ratio; i++) {
				// if(mEnvironment->isCollision()) {
				// 	mEnvironment->Step(mEnvironment->GetObstacles());	
				// } else {
					mEnvironment->Step();	
				// }
			}
			mEnvironment->GetRewards();
			mEnvironment->SetPhase(cur_phase+1);
			mEnvironment->isEndOfEpisode();
		} else {
			int cur_phase = mEnvironment->GetPhase();
			mEnvironment->SetActions(mActions);
			for(int i=0; i<ratio; i++)
				mEnvironment->Step();

			mEnvironment->SetPhase(cur_phase+1);
		}
	}
	glutTimerFunc(mDisplayTimeout, TimerEvent,1);
	glutPostRedisplay();
}