#ifndef __ATTACHMENT_CONSTRAINT_H__
#define __ATTACHMENT_CONSTRAINT_H__	
#include "Constraint.h"
#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace FEM
{
class Constraint;
enum ConstraintType;

class AttachmentConstraint : public Constraint
{
public:
	AttachmentConstraint(const double& stiffness,int i0,const Eigen::Vector3d& p);

	int GetDof() override;
	ConstraintType GetType() override;

	void	EvaluateJMatrix(int index, std::vector<Eigen::Triplet<double>>& J_triplets);
	void	EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets);
	void 	EvaluateDVector(const Eigen::VectorXd& x);
	void 	GetDVector(int& index,Eigen::VectorXd& d);

	int&			 GetI0() {return mi0;}
	Eigen::Vector3d& GetP()  {return mp;}

protected:
	int mi0;
	Eigen::Vector3d mp;
	Eigen::Vector3d md;
};

};
#endif