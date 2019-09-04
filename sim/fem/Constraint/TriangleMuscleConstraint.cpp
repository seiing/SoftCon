#include "TriangleMuscleConstraint.h"
#include <Eigen/SVD>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <iostream>

using namespace FEM;
TriangleMuscleConstraint::
TriangleMuscleConstraint(double stiffness,const Eigen::Vector2d& fiber_direction,int i0,int i1,int i2,double area,const Eigen::Matrix2d& invDm)
	:Constraint(stiffness),mFiberDirection(fiber_direction),
	mi0(i0),mi1(i1),mi2(i2),mArea(area),mInvDm(invDm),mActivationLevel(0.0)
{

}
int
TriangleMuscleConstraint::
GetDof()
{
	return 1;
}

void
TriangleMuscleConstraint::
EvaluateJMatrix(int index, std::vector<Eigen::Triplet<double>>& J_triplets)
{
	Eigen::MatrixXd Ai(3,9);

	Eigen::Vector2d v = mInvDm*mFiberDirection;

	Ai<<
		-v[0]-v[1],0,0,v[0],0,0,v[1],0,0,
		0,-v[0]-v[1],0,0,v[0],0,0,v[1],0,
		0,0,-v[0]-v[1],0,0,v[0],0,0,v[1];

	Eigen::MatrixXd MuAiT = mStiffness*mArea*Ai.transpose();

	int idx[3] = {mi0,mi1,mi2};

	for(int i=0;i<3;i++)
	{
		J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*(index)+0, MuAiT(3*i+0,0)));
		J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*(index)+1, MuAiT(3*i+0,1)));
		J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*(index)+2, MuAiT(3*i+0,2)));
		J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*(index)+0, MuAiT(3*i+1,0)));
		J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*(index)+1, MuAiT(3*i+1,1)));
		J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*(index)+2, MuAiT(3*i+1,2)));
		J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*(index)+0, MuAiT(3*i+2,0)));
		J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*(index)+1, MuAiT(3*i+2,1)));
		J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*(index)+2, MuAiT(3*i+2,2)));
	}
}
void
TriangleMuscleConstraint::
EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets)
{
	Eigen::MatrixXd Ai(3,9);

	Eigen::Vector2d v = mInvDm*mFiberDirection;
	Ai<<
		-v[0]-v[1],0,0,v[0],0,0,v[1],0,0,
		0,-v[0]-v[1],0,0,v[0],0,0,v[1],0,
		0,0,-v[0]-v[1],0,0,v[0],0,0,v[1];

	Eigen::MatrixXd MuAiTAi = mStiffness*mArea*((Ai.transpose())*Ai);

	int idx[3] = {mi0,mi1,mi2};
	for(int i =0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*idx[j]+0, MuAiTAi(3*i+0, 3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*idx[j]+1, MuAiTAi(3*i+0, 3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*idx[j]+2, MuAiTAi(3*i+0, 3*j+2)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*idx[j]+0, MuAiTAi(3*i+1, 3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*idx[j]+1, MuAiTAi(3*i+1, 3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*idx[j]+2, MuAiTAi(3*i+1, 3*j+2)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*idx[j]+0, MuAiTAi(3*i+2, 3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*idx[j]+1, MuAiTAi(3*i+2, 3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*idx[j]+2, MuAiTAi(3*i+2, 3*j+2)));
		}
	}
}
void
TriangleMuscleConstraint::
EvaluateDVector(const Eigen::VectorXd& x)
{
	Eigen::Vector3d x0(x.segment<3>(mi0*3));

	Eigen::Matrix32d Ds, P;
	Ds.col(0) = x.segment<3>(mi1*3) - x0;
	Ds.col(1) = x.segment<3>(mi2*3) - x0;

	P.col(0) = Ds.col(0).normalized();
	P.col(1) = (Ds.col(1)-Ds.col(1).dot(P.col(0))*P.col(0)).normalized();

	Eigen::Matrix2d F = P.transpose()*Ds*mInvDm;

	md = (1.0-mActivationLevel)*P*F*mFiberDirection;	
}
void
TriangleMuscleConstraint::
GetDVector(int& index,Eigen::VectorXd& d)
{
	d.segment<3>(3*(index)) = md;
	index++;
}