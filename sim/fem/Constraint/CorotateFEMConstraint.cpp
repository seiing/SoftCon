#include "CorotateFEMConstraint.h"
#include <Eigen/SVD>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <iostream>
// #include <ctime>
// #include <chrono>
#define EPS 1E-4
using namespace FEM;

CorotateFEMConstraint::
CorotateFEMConstraint(const double& stiffness,const double& poisson_ratio,
	int i0,int i1,int i2,int i3,double vol,const Eigen::Matrix3d& invDm)
	:Constraint(stiffness),
	mPoissonRatio(poisson_ratio),
	mi0(i0),mi1(i1),mi2(i2),mi3(i3),
	mMu(stiffness/((1.0+poisson_ratio))),
	mLambda(stiffness*poisson_ratio/((1.0+poisson_ratio)*(1-2.0*poisson_ratio))),
	mVol(vol),mInvDm(invDm),mDs(Eigen::Matrix3d::Zero())
{
	mF.setZero();
	mR.setZero();
	mU.setZero();	
	mV.setZero();
}
void
CorotateFEMConstraint::
ComputeF
(const Eigen::VectorXd& x)
{
	Eigen::Vector3d x0(x.block<3,1>(mi0*3,0));

	Eigen::Matrix3d Ds;

	Ds.block<3,1>(0,0) = x.block<3,1>(mi1*3,0)-x0;
	Ds.block<3,1>(0,1) = x.block<3,1>(mi2*3,0)-x0;
	Ds.block<3,1>(0,2) = x.block<3,1>(mi3*3,0)-x0;

	mDs = Ds;
	mF = mDs * mInvDm;

	ComputeSVD(mF);
}
void	
CorotateFEMConstraint::
ComputeP
(Eigen::Matrix3d& P)
{
	P = mMu*(mF - mR)
		+ mLambda*((mR.transpose()*mF-Eigen::Matrix3d::Identity()).trace())*mR;
}
void	
CorotateFEMConstraint::
ComputedPdF
(Tensor3333& dPdF)
{
	Tensor3333 dFdF, dRdF;
	dFdF.SetIdentity();

	for(int i =0;i<3;i++) {
		for(int j=0;j<3;j++) {
			Eigen::Matrix3d M = mU.transpose()*dFdF(i,j)*mV;

			if(fabs(mD(0,0)-mD(1,1)) < EPS && fabs(mD(0,0)-mD(2,2)) < EPS) {
				Eigen::Matrix3d off_diag_M;
				off_diag_M.setZero();
				for(int a=0; a<3; a++) {
					for(int b=0; b<3; b++) {
						if(a==b)
							continue;
						else
							off_diag_M(a,b) = M(a,b) / mD(0,0);
					}
				}

				dRdF(i,j) = mU*off_diag_M*mV.transpose();
			} else {
				Eigen::Vector2d unknown_side, known_side;
				Eigen::Matrix2d known_matrix;
				Eigen::Matrix3d U_tilde, V_tilde;
				U_tilde.setZero(); 
				V_tilde.setZero();
				Eigen::Matrix2d reg;
				reg.setZero();
				reg(0,0) = reg(1,1) = EPS;
				for (unsigned int row = 0; row < 3; row++) {
					for (unsigned int col = 0; col < row; col++) {
						known_side = Eigen::Vector2d(M(col, row), M(row, col));
						known_matrix.block<2, 1>(0, 0) = Eigen::Vector2d(-mD(row,row), mD(col,col));
						known_matrix.block<2, 1>(0, 1) = Eigen::Vector2d(-mD(col,col), mD(row,row));

						if (fabs(mD(row,row) - mD(col,col) < EPS))
							known_matrix += reg;
						else
							assert(fabs(known_matrix.determinant()) > 1E-6);

						unknown_side = known_matrix.inverse() * known_side;
						U_tilde(row, col) = unknown_side[0];
						U_tilde(col, row) = -U_tilde(row, col);
						V_tilde(row, col) = unknown_side[1];
						V_tilde(col, row) = -V_tilde(row, col);
					}
				}
				Eigen::Matrix3d deltaU = mU*U_tilde;
				Eigen::Matrix3d deltaV = V_tilde*mV.transpose();

				dRdF(i, j) = deltaU*mV.transpose() + mU*deltaV;
			}
		}
	}

	Tensor3333 lambda_term;
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			lambda_term(i,j) =
				(dRdF(i,j).transpose()*mF+mR.transpose()*dFdF(i,j)).trace()*mR +
				(mR.transpose()*mF-Eigen::Matrix3d::Identity()).trace()*dRdF(i,j);
		}
	}

	dPdF = (dFdF-dRdF)*mMu + mLambda*lambda_term;
}
void
CorotateFEMConstraint::
ComputeSVD(const Eigen::Matrix3d& F)
{
	// #pragma omp critical 
	// {
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Vector3d D = svd.singularValues();
		
		mD.setZero();	

		mD(0,0) = D[0];
		mD(1,1) = D[1];
		mD(2,2) = D[2];

		mU = svd.matrixU();
		mV = svd.matrixV();
		mR = mU*mV.transpose();
		mF = F;
	// }
}
int
CorotateFEMConstraint::
GetDof()
{
	return 6;
}
ConstraintType
CorotateFEMConstraint::
GetType()
{
	return ConstraintType::COROTATE;
}
void
CorotateFEMConstraint::
EvaluateJMatrix(int index, std::vector<Eigen::Triplet<double>>& J_triplets)
{
	Eigen::MatrixXd Ai(3*3,3*4);
	double d11 = mInvDm(0,0);
	double d12 = mInvDm(0,1);
	double d13 = mInvDm(0,2);
	double d21 = mInvDm(1,0);
	double d22 = mInvDm(1,1);
	double d23 = mInvDm(1,2);
	double d31 = mInvDm(2,0);
	double d32 = mInvDm(2,1);
	double d33 = mInvDm(2,2);

	Ai<<
		-d11-d21-d31,0,0,d11,0,0,d21,0,0,d31,0,0,
		0,-d11-d21-d31,0,0,d11,0,0,d21,0,0,d31,0,
		0,0,-d11-d21-d31,0,0,d11,0,0,d21,0,0,d31,
		-d12-d22-d32,0,0,d12,0,0,d22,0,0,d32,0,0,
		0,-d12-d22-d32,0,0,d12,0,0,d22,0,0,d32,0,
		0,0,-d12-d22-d32,0,0,d12,0,0,d22,0,0,d32,
		-d13-d23-d33,0,0,d13,0,0,d23,0,0,d33,0,0,
		0,-d13-d23-d33,0,0,d13,0,0,d23,0,0,d33,0,
		0,0,-d13-d23-d33,0,0,d13,0,0,d23,0,0,d33;

	Eigen::MatrixXd MuAiT = mMu*mVol*Ai.transpose();
	int idx[4] = {mi0,mi1,mi2,mi3};
	for(int i =0;i<4;i++)
	{
		for(int j=0;j<3;j++)
		{
			//MuAiT.block [i,j] -- 3x3 matrix
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*(index+j)+0, MuAiT(3*i+0, 3*j+0)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*(index+j)+1, MuAiT(3*i+0, 3*j+1)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*(index+j)+2, MuAiT(3*i+0, 3*j+2)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*(index+j)+0, MuAiT(3*i+1, 3*j+0)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*(index+j)+1, MuAiT(3*i+1, 3*j+1)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*(index+j)+2, MuAiT(3*i+1, 3*j+2)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*(index+j)+0, MuAiT(3*i+2, 3*j+0)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*(index+j)+1, MuAiT(3*i+2, 3*j+1)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*(index+j)+2, MuAiT(3*i+2, 3*j+2)));
		}
	}
	index+=3;

	MuAiT = (MuAiT*mPoissonRatio).eval();
	for(int i =0;i<4;i++)
	{
		for(int j=0;j<3;j++)
		{
			//MuAiT.block [i,j] -- 3x3 matrix
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*(index+j)+0, MuAiT(3*i+0, 3*j+0)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*(index+j)+1, MuAiT(3*i+0, 3*j+1)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*(index+j)+2, MuAiT(3*i+0, 3*j+2)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*(index+j)+0, MuAiT(3*i+1, 3*j+0)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*(index+j)+1, MuAiT(3*i+1, 3*j+1)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*(index+j)+2, MuAiT(3*i+1, 3*j+2)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*(index+j)+0, MuAiT(3*i+2, 3*j+0)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*(index+j)+1, MuAiT(3*i+2, 3*j+1)));
			J_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*(index+j)+2, MuAiT(3*i+2, 3*j+2)));
		}
	}
	index+=3;
}
void
CorotateFEMConstraint::
EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets)
{
	Eigen::MatrixXd Ai(3*3,3*4);
	double d11 = mInvDm(0,0);
	double d12 = mInvDm(0,1);
	double d13 = mInvDm(0,2);
	double d21 = mInvDm(1,0);
	double d22 = mInvDm(1,1);
	double d23 = mInvDm(1,2);
	double d31 = mInvDm(2,0);
	double d32 = mInvDm(2,1);
	double d33 = mInvDm(2,2);

	Ai<<
		-d11-d21-d31,0,0,d11,0,0,d21,0,0,d31,0,0,
		0,-d11-d21-d31,0,0,d11,0,0,d21,0,0,d31,0,
		0,0,-d11-d21-d31,0,0,d11,0,0,d21,0,0,d31,
		-d12-d22-d32,0,0,d12,0,0,d22,0,0,d32,0,0,
		0,-d12-d22-d32,0,0,d12,0,0,d22,0,0,d32,0,
		0,0,-d12-d22-d32,0,0,d12,0,0,d22,0,0,d32,
		-d13-d23-d33,0,0,d13,0,0,d23,0,0,d33,0,0,
		0,-d13-d23-d33,0,0,d13,0,0,d23,0,0,d33,0,
		0,0,-d13-d23-d33,0,0,d13,0,0,d23,0,0,d33;

	Eigen::MatrixXd MuAiTAi = mMu*mVol*((Ai.transpose())*Ai);
	int idx[4] = {mi0,mi1,mi2,mi3};
	//MuAiT --- 12x12 matrix
	for(int i =0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			//MuAiTAi.block [i,j] -- 3x3 matrix
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*idx[j]+0, MuAiTAi(3*i+0, 3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*idx[j]+1, MuAiTAi(3*i+0, 3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*idx[j]+2, MuAiTAi(3*i+0, 3*j+2)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*idx[j]+0, MuAiTAi(3*i+1, 3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*idx[j]+1, MuAiTAi(3*i+1, 3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*idx[j]+2, MuAiTAi(3*i+1, 3*j+2)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*idx[j]+0, MuAiTAi(3*i+2, 3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*idx[j]+1, MuAiTAi(3*i+2, 3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*idx[j]+2, MuAiTAi(3*i+2, 3*j+2)));
		}
	}

	MuAiTAi = (MuAiTAi*mPoissonRatio).eval();
	for(int i =0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			//MuAiTAi.block [i,j] -- 3x3 matrix
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*idx[j]+0, MuAiTAi(3*i+0, 3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*idx[j]+1, MuAiTAi(3*i+0, 3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0, 3*idx[j]+2, MuAiTAi(3*i+0, 3*j+2)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*idx[j]+0, MuAiTAi(3*i+1, 3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*idx[j]+1, MuAiTAi(3*i+1, 3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1, 3*idx[j]+2, MuAiTAi(3*i+1, 3*j+2)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*idx[j]+0, MuAiTAi(3*i+2, 3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*idx[j]+1, MuAiTAi(3*i+2, 3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2, 3*idx[j]+2, MuAiTAi(3*i+2, 3*j+2)));
		}
	}
}
void
CorotateFEMConstraint::
EvaluateDVector(const Eigen::VectorXd& x)
{
	ComputeF(x);
	
	md = mR;
	if(mF.determinant()<0)
		md.block<3,1>(0,2) = -mR.block<3,1>(0,2);

	Eigen::Vector3d S = mD.diagonal();
	Eigen::Vector3d D;
	D.setZero();
	double CD;
	for(int i=0;i<5;i++)
	{
		CD = (S[0]+D[0])*(S[1]+D[1])*(S[2]+D[2])-1;
		Eigen::Vector3d gradCD( (S[1]+D[1])*(S[2]+D[2]),
								(S[0]+D[0])*(S[2]+D[2]),
								(S[0]+D[0])*(S[1]+D[1]));

		D = (gradCD.dot(D) -CD)/(gradCD.squaredNorm())*gradCD;
	}

	md_volume = mU*((S+D).asDiagonal())*mV.transpose();
}
void
CorotateFEMConstraint::
GetDVector(int& index,Eigen::VectorXd& d)
{
	d.block<3,1>(3*(index+0),0) = md.block<3,1>(0,0);
	d.block<3,1>(3*(index+1),0) = md.block<3,1>(0,1);
	d.block<3,1>(3*(index+2),0) = md.block<3,1>(0,2);
	index+=3;

	d.block<3,1>(3*(index+0),0) = md_volume.block<3,1>(0,0);
	d.block<3,1>(3*(index+1),0) = md_volume.block<3,1>(0,1);
	d.block<3,1>(3*(index+2),0) = md_volume.block<3,1>(0,2);
	index+=3;
}
