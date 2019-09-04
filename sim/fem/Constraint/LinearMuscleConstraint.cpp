#include "LinearMuscleConstraint.h"
#include <Eigen/SVD>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <iostream>
using namespace FEM;

LinearMuscleConstraint::
LinearMuscleConstraint(const double& stiffness,
		const Eigen::Vector3d& fiber_direction,
		const double& activation_level, 
		int i0,int i1,int i2,int i3,
		double vol,const Eigen::Matrix3d& invDm,
		// const Eigen::Vector4d& barycentric1,const Eigen::Vector4d& barycentric2,
		double weight)
	:Constraint(stiffness),
	mi0(i0),mi1(i1),mi2(i2),mi3(i3),
	mStiffness(stiffness),mFiberDirection(fiber_direction),
	mVol(vol),mInvDm(invDm),mDs(Eigen::Matrix3d::Zero()),
	mActivationLevel(activation_level),mWeight(weight)
{
	mF.setZero();
	// mBarycentric.clear();
	// mBarycentric.push_back(barycentric1);
	// mBarycentric.push_back(barycentric2);

	mStiffness *= mWeight;
}
void
LinearMuscleConstraint:: 
ComputeF
(const Eigen::VectorXd& x)
{
	Eigen::Vector3d x0(x.block<3,1>(mi0*3,0));

	Eigen::Matrix3d Ds;

	Ds.block<3,1>(0,0) = x.block<3,1>(mi1*3,0)-x0;
	Ds.block<3,1>(0,1) = x.block<3,1>(mi2*3,0)-x0;
	Ds.block<3,1>(0,2) = x.block<3,1>(mi3*3,0)-x0;

	mDs = Ds;
	mF = mDs * mInvDm;
}
void
LinearMuscleConstraint::
ComputeP
(Eigen::Matrix3d& P)
{
	P = mStiffness*(mF*mFiberDirection*mFiberDirection.transpose() - mp0*mFiberDirection.transpose());
}	
void	
LinearMuscleConstraint::
ComputedPdF
(Tensor3333& dPdF)
{
	Tensor3333 dFdF;
	dFdF.SetIdentity();

	for(int i=0; i<3;i++)
		for(int j=0;j<3;j++)
			dPdF(i,j) = mStiffness*dFdF(i,j)*mFiberDirection*mFiberDirection.transpose();
}
void
LinearMuscleConstraint::	
EvaluateJMatrix(int index, std::vector<Eigen::Triplet<double>>& J_triplets)
{
	Eigen::MatrixXd Ai(3,12);

	Eigen::Vector3d v = mInvDm*mFiberDirection;

	double a,b,c;
	a = v[0];
	b = v[1];
	c = v[2];

	Ai<<
		-(a+b+c),0,0,a,0,0,b,0,0,c,0,0,
		0,-(a+b+c),0,0,a,0,0,b,0,0,c,0,
		0,0,-(a+b+c),0,0,a,0,0,b,0,0,c;

	Eigen::MatrixXd MuAiT = mVol*mStiffness*Ai.transpose();

	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+0,3*index+0,MuAiT(3*0+0,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+0,3*index+1,MuAiT(3*0+0,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+0,3*index+2,MuAiT(3*0+0,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+1,3*index+0,MuAiT(3*0+1,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+1,3*index+1,MuAiT(3*0+1,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+1,3*index+2,MuAiT(3*0+1,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+2,3*index+0,MuAiT(3*0+2,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+2,3*index+1,MuAiT(3*0+2,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+2,3*index+2,MuAiT(3*0+2,3*0+2)));

	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+0,3*index+0,MuAiT(3*1+0,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+0,3*index+1,MuAiT(3*1+0,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+0,3*index+2,MuAiT(3*1+0,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+1,3*index+0,MuAiT(3*1+1,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+1,3*index+1,MuAiT(3*1+1,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+1,3*index+2,MuAiT(3*1+1,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+2,3*index+0,MuAiT(3*1+2,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+2,3*index+1,MuAiT(3*1+2,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+2,3*index+2,MuAiT(3*1+2,3*0+2)));

	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+0,3*index+0,MuAiT(3*2+0,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+0,3*index+1,MuAiT(3*2+0,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+0,3*index+2,MuAiT(3*2+0,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+1,3*index+0,MuAiT(3*2+1,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+1,3*index+1,MuAiT(3*2+1,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+1,3*index+2,MuAiT(3*2+1,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+2,3*index+0,MuAiT(3*2+2,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+2,3*index+1,MuAiT(3*2+2,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+2,3*index+2,MuAiT(3*2+2,3*0+2)));

	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+0,3*index+0,MuAiT(3*3+0,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+0,3*index+1,MuAiT(3*3+0,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+0,3*index+2,MuAiT(3*3+0,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+1,3*index+0,MuAiT(3*3+1,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+1,3*index+1,MuAiT(3*3+1,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+1,3*index+2,MuAiT(3*3+1,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+2,3*index+0,MuAiT(3*3+2,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+2,3*index+1,MuAiT(3*3+2,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+2,3*index+2,MuAiT(3*3+2,3*0+2)));
}
void
LinearMuscleConstraint::	
EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets)
{
	Eigen::MatrixXd Ai(3,12);
	Eigen::Vector3d v = mInvDm*mFiberDirection;

	double a,b,c;
	a = v[0];
	b = v[1];
	c = v[2];

	Ai<<
		-(a+b+c),0,0,a,0,0,b,0,0,c,0,0,
		0,-(a+b+c),0,0,a,0,0,b,0,0,c,0,
		0,0,-(a+b+c),0,0,a,0,0,b,0,0,c;

	auto MuAiTAi = mVol*mStiffness*((Ai.transpose())*Ai);

	int idx[4] = {mi0,mi1,mi2,mi3};

	for(int i =0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0,3*idx[j]+0,MuAiTAi(3*i+0,3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0,3*idx[j]+1,MuAiTAi(3*i+0,3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0,3*idx[j]+2,MuAiTAi(3*i+0,3*j+2)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1,3*idx[j]+0,MuAiTAi(3*i+1,3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1,3*idx[j]+1,MuAiTAi(3*i+1,3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1,3*idx[j]+2,MuAiTAi(3*i+1,3*j+2)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2,3*idx[j]+0,MuAiTAi(3*i+2,3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2,3*idx[j]+1,MuAiTAi(3*i+2,3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2,3*idx[j]+2,MuAiTAi(3*i+2,3*j+2)));
		}
	}
}
void
LinearMuscleConstraint::	
EvaluateDVector(const Eigen::VectorXd& x)
{
	ComputeF(x);
	Computep0();
}
void
LinearMuscleConstraint::
GetDVector(int& index,Eigen::VectorXd& d)
{
	d.block<3,1>(3*index,0) = mp0;
	index++;
}
void
LinearMuscleConstraint::
Computep0()
{
	mp0 = (1.0-mActivationLevel)*mF*mFiberDirection;
}	
int
LinearMuscleConstraint::
GetDof()
{
	return 1;
}
ConstraintType
LinearMuscleConstraint::
GetType()
{
	return ConstraintType::LINEAR_MUSCLE;
}
void 
LinearMuscleConstraint::
SetActivationLevel(const double& a) 
{
	mActivationLevel = a;
}
const double& 
LinearMuscleConstraint::
GetActivationLevel() 
{
	return mActivationLevel;
}
const Eigen::Vector3d&
LinearMuscleConstraint::
GetFiberDirection()
{
	return mFiberDirection;
}
void 
LinearMuscleConstraint::
SetActivationIndex(const int& i)
{
	mActivationIndex = i;
}