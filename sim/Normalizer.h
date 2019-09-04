#ifndef __NORMALIZER_H__
#define __NORMALIZER_H__
#include <Eigen/Core>
#include <vector>
class Normalizer
{
public: 
	Normalizer(const Eigen::VectorXd& real_val_max,const Eigen::VectorXd& real_val_min,
		const Eigen::VectorXd& norm_val_max,const Eigen::VectorXd& norm_val_min);

public:
	Eigen::VectorXd RealToNorm(const Eigen::VectorXd& val);
	Eigen::VectorXd NormToReal(const Eigen::VectorXd& val);

private:
	int mDim;

	Eigen::VectorXd mRealValMax;	
	Eigen::VectorXd mRealValMin;
	Eigen::VectorXd mRealValDiff;
	Eigen::VectorXd mRealValDiffInv;

	Eigen::VectorXd mNormValMax;	
	Eigen::VectorXd mNormValMin;
	Eigen::VectorXd mNormValDiff;
	Eigen::VectorXd mNormValDiffInv;
};
#endif