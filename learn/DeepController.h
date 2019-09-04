#ifndef __DEEP_CONTROLLER_H__
#define __DEEP_CONTROLLER_H__
#include <vector>
#include <tuple>
#include <string>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include "Environment.h"
namespace p = boost::python;
namespace np = boost::python::numpy;
class DeepController
{
public:
	DeepController(std::string name);

	np::ndarray GetStates();
	void SetActions(np::ndarray a, int n);
	void Step();
	p::dict GetRewards();
	bool isEndOfEpisode();
	void Reset();
	void UpdateRandomTargetVelocity();

	double GetTimeStep() {return mEnvironment->GetSoftWorld()->GetTimeStep();};
	np::ndarray GetTargetVelocity();

	int GetSimulationHz() {return mEnvironment->GetSimulationHz();};
	int GetControlHz() {return mEnvironment->GetControlHz();};

	np::ndarray GetNormUpperBound();
	np::ndarray GetNormLowerBound();

private:
	Environment*	mEnvironment;

};
#endif