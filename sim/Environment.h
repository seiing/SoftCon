#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__
#include <deque>
#include "fem/World.h"
#include "Octopus.h"
#include "Normalizer.h"
class Octopus;
class Normalizer;
class Environment
{
public:
	Environment();
	
	FEM::World* GetSoftWorld(){return mSoftWorld;};
	Octopus* GetOctopus(){return mOctopus;};

	double GetSimulationHz(){return mSimulationHz;};
	double GetControlHz(){return mControlHz;};

	void Step();
	void Reset();

	const Eigen::VectorXd& GetStates();

	void InitializeActions();
	const Eigen::VectorXd& GetActions() {return mActions;};
	void SetActions(const Eigen::VectorXd& actions);

	std::map<std::string,double> GetRewards();

	bool isEndOfEpisode();

	Eigen::VectorXd GetNormLowerBound() {return mNormLowerBound;};
	Eigen::VectorXd GetNormUpperBound() {return mNormUpperBound;};
	Normalizer*	GetNormalizer() {return mNormalizer;};

	void SetPhase(const int& phase); 
	const int& GetPhase() {return mPhase;};

	Eigen::Vector3d GetAverageVelocity() {return mAverageVelocity;};
	Eigen::Vector3d GetTargetVelocity() {return mTargetVelocity;};
	void UpdateRandomTargetVelocity();

private:
	FEM::World*						mSoftWorld;
	Octopus*						mOctopus;

	int 							mSimulationHz;
	int 							mControlHz;

	Normalizer*						mNormalizer;
	Eigen::VectorXd 				mNormLowerBound;
	Eigen::VectorXd 				mNormUpperBound;

	Eigen::VectorXd					mStates;
	Eigen::VectorXd					mActions;

	int 							mPhase;

	Eigen::Vector3d					mTargetVelocity;
	Eigen::Vector3d					mAverageVelocity;
	std::deque<Eigen::Vector3d>		mAverageVelocityDeque;
};
#endif