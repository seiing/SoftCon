#ifndef __LINEAR_MUSCLE_CONSTRAINT_H__
#define	__LINEAR_MUSCLE_CONSTRAINT_H__

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
class LinearMuscleConstraint : public Constraint
{
public:
	LinearMuscleConstraint(const double& stiffness,
		const Eigen::Vector3d& fiber_direction,
		const double& activation_level, 
		int i0,int i1,int i2,int i3,
		double vol,const Eigen::Matrix3d& invDm,
		// const Eigen::Vector4d& barycentric1,const Eigen::Vector4d& barycentric2,
		double weight);

protected:
	double				mStiffness;
	Eigen::Vector3d		mFiberDirection;
	double 				mActivationLevel;
	int 				mActivationIndex;
	Eigen::Matrix3d 	mddT;

	int 				mi0,mi1,mi2,mi3;
	double 				mVol;
	Eigen::Matrix3d 	mInvDm;
	Eigen::Matrix3d 	mDs;
	Eigen::Matrix3d 	mF;

	Eigen::Vector3d		mp0;

	Eigen::Matrix3d		md;

	double				mWeight;
	std::vector<Eigen::Vector4d> mBarycentric;

public:
	int GetI0() {return mi0;}
	int GetI1() {return mi1;}
	int GetI2() {return mi2;}
	int GetI3() {return mi3;}
	
	int GetDof() override;
	ConstraintType GetType() override;

	void	EvaluateJMatrix(int index, std::vector<Eigen::Triplet<double>>& J_triplets) override;
	void	EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets) override;
	void 	EvaluateDVector(const Eigen::VectorXd& x);
	void 	GetDVector(int& index,Eigen::VectorXd& d);

private:
	void	ComputeF(const Eigen::VectorXd& x);
	void	ComputeP(Eigen::Matrix3d& P);
	void	ComputedPdF(Tensor3333& dPdF);

	void	Computep0();

public:
	void SetActivationLevel(const double& a);
	const double& GetActivationLevel();
	const Eigen::Vector3d& GetFiberDirection();
	int GetActivationIndex(){return mActivationIndex;};
	void SetActivationIndex(const int& i);
	std::vector<Eigen::Vector4d> GetBarycentric() {return mBarycentric;};
	double GetWeight() {return mWeight;};
};
};

#endif