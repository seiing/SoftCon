#include "Constraint.h"
#include "AttachmentConstraint.h"
#include <iostream>

using namespace FEM;
AttachmentConstraint::
AttachmentConstraint(const double& stiffness,int i0,const Eigen::Vector3d& p)
	:Constraint(stiffness),mi0(i0),mp(p)
{
	md.setZero();
}
int
AttachmentConstraint::
GetDof()
{
	return 1;
}
ConstraintType 
AttachmentConstraint::
GetType()	   
{
	return ConstraintType::ATTACHMENT; 
}
void	
AttachmentConstraint::
EvaluateJMatrix(int index, std::vector<Eigen::Triplet<double>>& J_triplets)
{
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+0,3*index+0,mStiffness));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+1,3*index+1,mStiffness));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+2,3*index+2,mStiffness));
}
void	
AttachmentConstraint::
EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets)
{
	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+0,3*mi0+0,mStiffness));
	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+1,3*mi0+1,mStiffness));
	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+2,3*mi0+2,mStiffness));
}
void
AttachmentConstraint::
EvaluateDVector(const Eigen::VectorXd& x)
{
	md = mp;
}
void
AttachmentConstraint::
GetDVector(int& index,Eigen::VectorXd& d)
{
	d.block<3,1>(index*3,0) = md;
	index++;
}