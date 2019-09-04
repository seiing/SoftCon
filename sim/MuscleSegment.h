#ifndef __MUSCLE_SEGMENT_H__
#define __MUSCLE_SEGMENT_H__
#include "fem/Constraint/ConstraintHeader.h"
class MuscleSegment
{
public:
	MuscleSegment();
	void AddMuscleConstraint(FEM::LinearMuscleConstraint* lmc);
	const std::vector<FEM::LinearMuscleConstraint*>& GetMuscleConstraints(){return mMuscleConstraints;};

	void SetActivationLevel(double act);
	double GetActivationLevel(){return mActivationLevel;};

	void SetStart(const Eigen::Vector4i& start,const Eigen::Vector4d& barycentric);
	void SetEnd(const Eigen::Vector4i& end,const Eigen::Vector4d& barycentric);
	
	const Eigen::Vector4i& GetStartIdx() {return mStart;};
	const Eigen::Vector4i& GetEndIdx() {return mEnd;};
	const Eigen::Vector4d& GetStartBarycentric() {return mStartBarycentric;};
	const Eigen::Vector4d& GetEndBarycentric() {return mEndBarycentric;};

private:
	double 										mActivationLevel;
	std::vector<FEM::LinearMuscleConstraint*> 	mMuscleConstraints;

	Eigen::Vector4i mStart;
	Eigen::Vector4i mEnd;
	Eigen::Vector4d mStartBarycentric;
	Eigen::Vector4d mEndBarycentric;
};
#endif