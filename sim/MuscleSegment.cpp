#include "MuscleSegment.h"
MuscleSegment::
MuscleSegment()
	:mActivationLevel(0.0)
{
	mStart.setZero();
	mStartBarycentric.setZero();
	mEnd.setZero();
	mEndBarycentric.setZero();
}
void
MuscleSegment::
AddMuscleConstraint(FEM::LinearMuscleConstraint* lmc)
{
	mMuscleConstraints.push_back(lmc);	
}
void
MuscleSegment::
SetStart(const Eigen::Vector4i& start,const Eigen::Vector4d& barycentric)
{
	mStart = start;
	mStartBarycentric = barycentric;
}
void
MuscleSegment::
SetEnd(const Eigen::Vector4i& end,const Eigen::Vector4d& barycentric)
{
	mEnd = end;
	mEndBarycentric = barycentric;
}
void
MuscleSegment::
SetActivationLevel(double activation)
{
	mActivationLevel = activation;
	for(const auto& c : mMuscleConstraints)
	{
		c->SetActivationLevel(activation);
	}
}
