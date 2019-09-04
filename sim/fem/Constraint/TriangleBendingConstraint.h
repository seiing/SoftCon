#ifndef __TRIANGLE_BENDING_CONSTRAINT_H__
#define __TRIANGLE_BENDING_CONSTRAINT_H__
#include "Constraint.h"
#include <Eigen/Geometry>

namespace Eigen
{
using Matrix34d = Matrix<double, 3, 4>;
};
namespace FEM
{
class TriangleBendingConstraint : public Constraint
{
public:
	TriangleBendingConstraint(double stiffness,int i0,int i1,int i2,int i3,double voronoi_area,double n,const Eigen::Vector4d& w);
	int GetDof() override;

	void EvaluateJMatrix(int index, std::vector<Eigen::Triplet<double>>& J_triplets);
	void EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets);
	void EvaluateDVector(const Eigen::VectorXd& x);
	void GetDVector(int& index,Eigen::VectorXd& d);

	int GetI0() {return mi0;}
	int GetI1() {return mi1;}
	int GetI2() {return mi2;}
	int GetI3() {return mi3;}
protected:
	int mi0,mi1,mi2,mi3;
	double mn,mVoronoiArea;
	Eigen::Vector4d mw;

	Eigen::Vector3d md;
};
};


#endif