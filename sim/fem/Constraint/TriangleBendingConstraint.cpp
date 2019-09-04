#include "TriangleBendingConstraint.h"

#include <Eigen/SVD>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <iostream>
using namespace FEM;

TriangleBendingConstraint::
TriangleBendingConstraint(double stiffness,int i0,int i1,int i2,int i3,double voronoi_area,double n,const Eigen::Vector4d& w)
	:Constraint(stiffness),mi0(i0),mi1(i1),mi2(i2),mi3(i3),mVoronoiArea(voronoi_area),mn(n),mw(w)
{
}

int
TriangleBendingConstraint::
GetDof()
{
	return 1;
}

void
TriangleBendingConstraint::
EvaluateJMatrix(int index, std::vector<Eigen::Triplet<double>>& J_triplets)
{
	Eigen::MatrixXd Ai(3,12);

	Ai<<
		mw[0],0,0,mw[1],0,0,mw[2],0,0,mw[3],0,0,
		0,mw[0],0,0,mw[1],0,0,mw[2],0,0,mw[3],0,
		0,0,mw[0],0,0,mw[1],0,0,mw[2],0,0,mw[3];

	Eigen::MatrixXd MuAiT = mVoronoiArea*mStiffness*Ai.transpose();

	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+0,3*index+0,MuAiT(3*0+0,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+0,3*index+1,MuAiT(3*0+0,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+0,3*index+2,MuAiT(3*0+0,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+1,3*index+0,MuAiT(3*0+1,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+1,3*index+1,MuAiT(3*0+1,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+1,3*index+2,MuAiT(3*0+1,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+2,3*index+0,MuAiT(3*0+2,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+2,3*index+1,MuAiT(3*0+2,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+2,3*index+2,MuAiT(3*0+2,3*0+2)));

	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+0,3*index+0,MuAiT(3*1+0,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+0,3*index+1,MuAiT(3*1+0,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+0,3*index+2,MuAiT(3*1+0,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+1,3*index+0,MuAiT(3*1+1,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+1,3*index+1,MuAiT(3*1+1,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+1,3*index+2,MuAiT(3*1+1,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+2,3*index+0,MuAiT(3*1+2,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+2,3*index+1,MuAiT(3*1+2,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi1+2,3*index+2,MuAiT(3*1+2,3*0+2)));

	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+0,3*index+0,MuAiT(3*2+0,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+0,3*index+1,MuAiT(3*2+0,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+0,3*index+2,MuAiT(3*2+0,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+1,3*index+0,MuAiT(3*2+1,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+1,3*index+1,MuAiT(3*2+1,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+1,3*index+2,MuAiT(3*2+1,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+2,3*index+0,MuAiT(3*2+2,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+2,3*index+1,MuAiT(3*2+2,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi2+2,3*index+2,MuAiT(3*2+2,3*0+2)));

	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+0,3*index+0,MuAiT(3*3+0,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+0,3*index+1,MuAiT(3*3+0,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+0,3*index+2,MuAiT(3*3+0,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+1,3*index+0,MuAiT(3*3+1,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+1,3*index+1,MuAiT(3*3+1,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+1,3*index+2,MuAiT(3*3+1,3*0+2)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+2,3*index+0,MuAiT(3*3+2,3*0+0)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+2,3*index+1,MuAiT(3*3+2,3*0+1)));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi3+2,3*index+2,MuAiT(3*3+2,3*0+2)));
}
void
TriangleBendingConstraint::
EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets)
{
	Eigen::MatrixXd Ai(3,12);

	Ai<<
		mw[0],0,0,mw[1],0,0,mw[2],0,0,mw[3],0,0,
		0,mw[0],0,0,mw[1],0,0,mw[2],0,0,mw[3],0,
		0,0,mw[0],0,0,mw[1],0,0,mw[2],0,0,mw[3];


	auto MuAiTAi = mVoronoiArea*mStiffness*((Ai.transpose())*Ai);

	int idx[4] = {mi0,mi1,mi2,mi3};

	for(int i =0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0,3*idx[j]+0,MuAiTAi(3*i+0,3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0,3*idx[j]+1,MuAiTAi(3*i+0,3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+0,3*idx[j]+2,MuAiTAi(3*i+0,3*j+2)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1,3*idx[j]+0,MuAiTAi(3*i+1,3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1,3*idx[j]+1,MuAiTAi(3*i+1,3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+1,3*idx[j]+2,MuAiTAi(3*i+1,3*j+2)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2,3*idx[j]+0,MuAiTAi(3*i+2,3*j+0)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2,3*idx[j]+1,MuAiTAi(3*i+2,3*j+1)));
			L_triplets.push_back(Eigen::Triplet<double>(3*idx[i]+2,3*idx[j]+2,MuAiTAi(3*i+2,3*j+2)));
		}
	}
}
void
TriangleBendingConstraint::
EvaluateDVector(const Eigen::VectorXd& x)
{
	Eigen::Vector3d e = Eigen::Vector3d::Zero();
	if( mn > 1E-6 )
	{
		e += mw[0] * x.segment<3>(mi0);
		e += mw[1] * x.segment<3>(mi1);
		e += mw[2] * x.segment<3>(mi2);
		e += mw[3] * x.segment<3>(mi3);

		double l = e.norm();

		if(l>1E-6)
			e *= mn/l;
	}
	md = e;
}
void
TriangleBendingConstraint::
GetDVector(int& index,Eigen::VectorXd& d)
{
	d.segment<3>(3*index) = md;
	index++;
}