#ifndef __COROTATE_FEM_CONSTRAINT_H__
#define	__COROTATE_FEM_CONSTRAINT_H__
#include "Constraint.h"
#include "Tensor.h"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

namespace Eigen {
typedef Matrix<double,12, 1> Vector12d;
typedef Matrix<double,12,12> Matrix12d;
};
namespace FEM
{
class CorotateFEMConstraint : public Constraint
{
public:
	CorotateFEMConstraint(
		const double& stiffness,
		const double& poisson_ratio,
		int i0,int i1,int i2,int i3,
		double volume,const Eigen::Matrix3d& invDm);

	int GetI0() {return mi0;}
	int GetI1() {return mi1;}
	int GetI2() {return mi2;}
	int GetI3() {return mi3;}

	int GetDof() override;
	ConstraintType GetType() override;

private:
	void	ComputeF(const Eigen::VectorXd& x);
	void	ComputeP(Eigen::Matrix3d& P);
	void	ComputedPdF(Tensor3333& dPdF);
	void	ComputeSVD(const Eigen::Matrix3d& F);

	void	EvaluateJMatrix(int index, std::vector<Eigen::Triplet<double>>& J_triplets);
	void	EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets);
	void 	EvaluateDVector(const Eigen::VectorXd& x);
	void 	GetDVector(int& index,Eigen::VectorXd& d);

protected:
	int 				mi0,mi1,mi2,mi3;
	double 				mVol;
	double 				mMu,mLambda;
	double 				mPoissonRatio;
	Eigen::Matrix3d 	mInvDm;
	Eigen::Matrix3d 	mDs;
	Eigen::Matrix3d 	mF;
	Eigen::Matrix3d 	mR,mU,mV,mD;

	Eigen::Matrix3d		md;
	Eigen::Matrix3d		md_volume;
};
};
#endif