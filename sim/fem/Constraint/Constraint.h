#ifndef __CONSTRAINT_H__
#define __CONSTRAINT_H__	
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
namespace FEM
{
enum ConstraintType
{
	ATTACHMENT,
	COROTATE,
	LINEAR_MUSCLE,
	BENDING,
	STRAIN,
	TRIANGLE_MUSCLE
};
class Constraint
{
public:
	Constraint(const double& stiffness);

	virtual int GetDof() = 0;
	virtual ConstraintType GetType() = 0;

	virtual void	EvaluateJMatrix(int index, std::vector<Eigen::Triplet<double>>& J_triplets) =0;
	virtual void	EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets) =0;
	virtual void 	EvaluateDVector(const Eigen::VectorXd& x) = 0;
	virtual void 	GetDVector(int& index,Eigen::VectorXd& d) = 0;
	
protected:
	double mStiffness;

};
};
#endif