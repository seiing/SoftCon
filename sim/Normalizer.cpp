#include "Normalizer.h"
#include <iostream>
Normalizer::
Normalizer(const Eigen::VectorXd& real_val_max,const Eigen::VectorXd& real_val_min,
		const Eigen::VectorXd& norm_val_max,const Eigen::VectorXd& norm_val_min)
{
	mDim = real_val_max.size();

	mRealValMax.resize(mDim);
	mRealValMin.resize(mDim);

	mNormValMax.resize(mDim);
	mNormValMin.resize(mDim);

	mRealValDiff.resize(mDim);
	mNormValDiff.resize(mDim);

	mRealValDiffInv.resize(mDim);
	mNormValDiffInv.resize(mDim);

	mRealValMax = real_val_max;
	mRealValMin	= real_val_min;

	mNormValMax = norm_val_max;
	mNormValMin	= norm_val_min;

	mRealValDiff = mRealValMax-mRealValMin;
	mNormValDiff = mNormValMax-mNormValMin;

	for(int i=0; i<mDim; i++) {
		mRealValDiffInv[i] = 1.0/mRealValDiff[i];
		mNormValDiffInv[i] = 1.0/mNormValDiff[i];
	}
}
Eigen::VectorXd
Normalizer::
RealToNorm(const Eigen::VectorXd& val)
{
	Eigen::VectorXd val_0_1(mDim);
	
	for(int i=0; i<mDim; i++) 
	{	
		val_0_1[i] = (val[i] - mRealValMin[i]) * mRealValDiffInv[i];
		val_0_1[i] = std::min(std::max(val_0_1[i],0.0),1.0);
	}

	Eigen::VectorXd result(mDim);
	for(int i=0; i<mDim; i++) 
	{
		result[i] = mNormValMin[i] + mNormValDiff[i]*val_0_1[i];
	}

	return result;
}
Eigen::VectorXd
Normalizer::
NormToReal(const Eigen::VectorXd& val)
{
	Eigen::VectorXd val_0_1(mDim);
	
	for(int i=0; i<mDim; i++) 
	{	
		val_0_1[i] = (val[i] - mNormValMin[i]) * mNormValDiffInv[i];
		val_0_1[i] = std::min(std::max(val_0_1[i],0.0),1.0);
	}

	Eigen::VectorXd result(mDim);
	for(int i=0; i<mDim; i++) 
	{
		result[i] = mRealValMin[i] + mRealValDiff[i]*val_0_1[i];
	}

	return result;
}