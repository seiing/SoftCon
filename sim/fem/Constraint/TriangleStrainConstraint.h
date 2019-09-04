#ifndef __TRIANGLE_STRAIN_CONSTRAINT_H__
#define __TRIANGLE_STRAIN_CONSTRAINT_H__
#include "Constraint.h"
#include <Eigen/Geometry>

namespace Eigen
{
using Matrix32d = Matrix<double, 3, 2>;
};
namespace FEM
{
class TriangleStrainConstraint : public Constraint
{
public:
	TriangleStrainConstraint(double stiffness,int i0,int i1,int i2,double area,const Eigen::Matrix2d& invDm);
	int GetDof() override;

	void EvaluateJMatrix(int index, std::vector<Eigen::Triplet<double>>& J_triplets);
	void EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets);
	void EvaluateDVector(const Eigen::VectorXd& x);
	void GetDVector(int& index,Eigen::VectorXd& d);

	int GetI0() {return mi0;}
	int GetI1() {return mi1;}
	int GetI2() {return mi2;}
protected:
	int mi0,mi1,mi2;
	double mArea;
	Eigen::Matrix2d mInvDm;

	Eigen::Matrix32d md;
};
};


#endif