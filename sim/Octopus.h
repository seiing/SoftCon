#ifndef __OCTOPUS_H__
#define __OCTOPUS_H__
#include "fem/World.h"
#include "fem/Mesh/MeshHeaders.h"
#include "Muscle.h"
class Octopus
{
public:
	Octopus(const double& muscle_stiffness,const double& youngs_modulus,const double& poisson_ratio);

	void Initialize(FEM::World* world);
	void SetOctopus(const std::string& path);
	void SetMesh(const std::string& path,const Eigen::Affine3d& T);
	void MakeMuscles(const std::string& path,const double& gamma);

	void SetActivationLevels(const Eigen::VectorXd& actions,const int& phase); 
	void SetKey(std::string filename);

	Eigen::Matrix3d GetReferenceRotation(bool type,const Eigen::VectorXd& x);
	void SetInitReferenceRotation(const Eigen::VectorXd& x);

	const std::vector<Eigen::Vector3i>& GetContours(){return mContours;};

	void SetVertexNormal();
	const Eigen::VectorXd& GetVertexNormal(){return mVertexNormal;};

	const std::vector<Muscle*>& GetMuscles() {return mMuscles;};

	Eigen::VectorXd ComputeDragForces(FEM::World* world);

	Eigen::Vector3d GetUpVector(const Eigen::VectorXd& x);
	Eigen::Vector3d GetForwardVector(const Eigen::VectorXd& x);	

	const int& GetCenterIndex() {return mCenterIndex;};
	const int& GetEndEffectorIndex() {return mEndEffectorIndex;};
	const std::vector<int>& GetSamplingIndex() {return mSamplingIndex;};

private:
	double 									mMuscleStiffness;
	double 									mYoungsModulus;
	double									mPoissonRatio;

	FEM::Mesh*								mMesh;
	Eigen::Affine3d							mScalingMatrix;

	std::vector<FEM::Constraint*>			mConstraints;

	std::vector<Muscle*>					mMuscles;

	std::vector<Eigen::Vector3i> 			mContours;

	int 									mCenterIndex;
	int 									mEndEffectorIndex;
	std::vector<int> 						mSamplingIndex;
	Eigen::Vector3d							mLocalContourIndex;

	Eigen::VectorXd							mVertexNormal;

	Eigen::Matrix3d 						mInitReferenceMatrix;
};
#endif