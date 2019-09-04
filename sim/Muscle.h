#ifndef __MUSCLE_H__
#define __MUSCLE_H__
#include "MuscleSegment.h"
#include "fem/Mesh/MeshHeaders.h"
class Muscle
{
public:
	Muscle(int num_sampling,std::vector<Eigen::Vector3d> point_list);

	void Initialize(FEM::Mesh* mesh,double muscle_stiffness);
	void Reset();

	const std::vector<MuscleSegment*>& GetSegments() {return mSegments;};

	const std::vector<Eigen::Vector3d>& GetStarts() {return mStarts;};	
	const std::vector<Eigen::Vector3d>& GetEnds() {return mEnds;};	

	const Eigen::VectorXd& GetActions() {return mActions;};
	Eigen::VectorXd GetActionUpperBound() {return mActionUpperBound;};
	Eigen::VectorXd GetActionLowerBound() {return mActionLowerBound;};

	void SetKey(const Eigen::VectorXd& key);
	void SetActivationLevels(const Eigen::VectorXd& action,const int& phase);
	Eigen::VectorXd GetActivationLevels(){return mActivationLevels;};

private:	
	double 							mNumSampling;
	std::vector<MuscleSegment*> 	mSegments;

	Eigen::VectorXd 				mActivationLevels;

	std::vector<Eigen::Vector3d> 	mStarts;
	std::vector<Eigen::Vector3d> 	mEnds;

	Eigen::VectorXd					mActions;
	Eigen::VectorXd 				mActionUpperBound;	
	Eigen::VectorXd 				mActionLowerBound;

	Eigen::VectorXd					mKey;
};
bool isInTetra(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& p3,const Eigen::Vector3d& start,const Eigen::Vector3d& end);
Eigen::Matrix3d GetDm(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& p3);
bool isLineTriangleIntersect(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& start,const Eigen::Vector3d& end);
#endif