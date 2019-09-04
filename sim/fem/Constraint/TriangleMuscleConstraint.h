#ifndef __TRIANGLE_MUSCLE_CONSTRAINT_H__
#define __TRIANGLE_MUSCLE_CONSTRAINT_H__
#include "Constraint.h"
#include <Eigen/Geometry>
namespace Eigen
{
using Matrix32d = Matrix<double, 3, 2>;
};
namespace FEM
{
class TriangleMuscleConstraint : public Constraint
{
public:
	TriangleMuscleConstraint(double stiffness,const Eigen::Vector2d& fiber_direction,int i0,int i1,int i2,double area,const Eigen::Matrix2d& invDm);
	int GetDof() override;

	void EvaluateJMatrix(int index, std::vector<Eigen::Triplet<double>>& J_triplets);
	void EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets);
	void EvaluateDVector(const Eigen::VectorXd& x);
	void GetDVector(int& index,Eigen::VectorXd& d);

	int GetI0() {return mi0;}
	int GetI1() {return mi1;}
	int GetI2() {return mi2;}

	void SetActivationLevel(double a){mActivationLevel = std::min(1.0,std::max(0.0,a));}
	double GetActivationLevel(){return mActivationLevel;}
	const Eigen::Vector2d& GetFiberDirection() {return mFiberDirection;}
protected:
	Eigen::Vector2d mFiberDirection;
	double			mActivationLevel;

	int mi0,mi1,mi2;
	double mArea;
	Eigen::Matrix2d mInvDm;

	Eigen::Vector3d md;
};
};


#endif