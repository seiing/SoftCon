#ifndef __DRAW_FUNCTIONS_H__
#define __DRAW_FUNCTIONS_H__
#include "../sim/Octopus.h"
#include "../sim/Muscle.h"
#include "DrawPrimitives.h"
namespace GUI
{
	void DrawWorld();
	void DrawCharacter(Octopus* octopus,const Eigen::VectorXd& x,const Eigen::Vector3d& eye);
	void DrawMuscles(const std::vector<Muscle*>& muscles,const Eigen::VectorXd& x);
	void DrawActivations(const double& x, const double& y,const double& length,const double& height,Muscle* muscle1,Muscle* muscle2);
	void DrawOBJ();
};
#endif