#include "World.h"
#define EPS 5E-7
using namespace FEM;
World::
World(
		double time_step,
		int max_iteration,
		double damping_coeff,
		bool is_precessing_collision)
	:mTimeStep(time_step),
	mFrame(0),
	mMaxIteration(max_iteration),
	mDampingCoefficient(damping_coeff),
	mNumVertices(0),
	mConstraintDofs(0),
	mIsInitialized(false),
	mIsCollision(is_precessing_collision)
{
}
void
World::
Initialize()
{
	mTime = 0.0;
	mConstraintDofs = 0;

	for(auto c : mConstraints) 
	{
		mConstraintDofs += c->GetDof();
	}

	mV.resize(3*mNumVertices);
	mV.setZero();

	mIdentityMatrix.resize(3*mNumVertices,3*mNumVertices);
	mMassMatrix.resize(3*mNumVertices,3*mNumVertices);
	mInvMassMatrix.resize(3*mNumVertices,3*mNumVertices);
	
	std::vector<Eigen::Triplet<double>> i_triplets;
	std::vector<Eigen::Triplet<double>> m_triplets;
	std::vector<Eigen::Triplet<double>> inv_m_triplets;
	
	i_triplets.reserve(3*mNumVertices);
	m_triplets.reserve(3*mNumVertices);
	inv_m_triplets.reserve(3*mNumVertices);

	for(int i=0;i<mNumVertices;i++)
	{
		m_triplets.push_back(Eigen::Triplet<double>(3*i+0,3*i+0,mUnitMass[i]));
		m_triplets.push_back(Eigen::Triplet<double>(3*i+1,3*i+1,mUnitMass[i]));
		m_triplets.push_back(Eigen::Triplet<double>(3*i+2,3*i+2,mUnitMass[i]));

		inv_m_triplets.push_back(Eigen::Triplet<double>(3*i+0,3*i+0,1.0/mUnitMass[i]));
		inv_m_triplets.push_back(Eigen::Triplet<double>(3*i+1,3*i+1,1.0/mUnitMass[i]));
		inv_m_triplets.push_back(Eigen::Triplet<double>(3*i+2,3*i+2,1.0/mUnitMass[i]));

		i_triplets.push_back(Eigen::Triplet<double>(3*i+0,3*i+0,1.0));
		i_triplets.push_back(Eigen::Triplet<double>(3*i+1,3*i+1,1.0));
		i_triplets.push_back(Eigen::Triplet<double>(3*i+2,3*i+2,1.0));
	}

	mMassMatrix.setFromTriplets(m_triplets.cbegin(), m_triplets.cend());
	mInvMassMatrix.setFromTriplets(inv_m_triplets.cbegin(), inv_m_triplets.cend());
	mIdentityMatrix.setFromTriplets(i_triplets.cbegin(), i_triplets.cend());

	mExternalForces.resize(3*mNumVertices);
	mExternalForces.setZero();

	mQn.resize(3*mNumVertices);
	mQn.setZero();

	mInitX = mX;
	mInitV = mV;

	PreComputation();

	mIsInitialized = true;

	std::cout<<"Total degree of freedom : "<<mX.rows()<<std::endl;
	std::cout<<"Total constraints : "<<mConstraints.size()<<std::endl;
}
void
World::
Reset()
{
	mX = mInitX;
	mV = mInitV;
	mTime = 0.0;
}
void
World::
AddBody(const Eigen::VectorXd& x0,const std::vector<Constraint*>& c,const double& mass)
{
	int nv=(x0.rows()/3);
	mNumVertices+=nv;

	Eigen::VectorXd tmpX;
	tmpX.resize(mNumVertices*3);
	mX.resize(mNumVertices*3);

	mX.head(tmpX.rows()) = tmpX;
	mX.tail(x0.rows()) = x0;

	mConstraints.insert(mConstraints.end(),c.begin(),c.end());
	double unit_mass = mass/(double)nv;

	for(int i=0;i<nv;i++){
		mUnitMass.push_back(unit_mass);
	}

	if(mIsInitialized)
		Initialize();
}
void
World::
AddConstraint(Constraint* c)
{
	mConstraints.push_back(c);
	if(mIsInitialized){
		mConstraintDofs = 0;
		for(auto c : mConstraints){
			mConstraintDofs += c->GetDof();
		}
		PreComputation();
	}
}
void
World::
RemoveConstraint(Constraint* c)
{
	bool isRemoved = false;
	for(int i=0;i<mConstraints.size();i++)
	{
		if(mConstraints[i]==c) {
			mConstraints.erase(mConstraints.begin()+i);
			isRemoved = true;
			break;
		}
	}

	if(isRemoved) {
		if(mIsInitialized) {
			mConstraintDofs = 0;
			for(auto c : mConstraints){
				mConstraintDofs += c->GetDof();
			}
			PreComputation();
		}
	}
}
void 
World::
TimeStepping(bool isIntegrated)
{
	if(!mIsInitialized) {
		std::cout<<"Engine not initialized."<<std::endl;
		return;
	}

	Eigen::VectorXd x_n1(mNumVertices*3);

	mQn = mX + mTimeStep*mV + (mTimeStep*mTimeStep)*(mInvMassMatrix*mExternalForces);

	x_n1=ProjectiveDynamicsMethod();

	if(mIsCollision) {

	}

	UpdatePositionsAndVelocities(x_n1);
	mV *= mDampingCoefficient;

	if(isIntegrated)
	{	
		mTime += mTimeStep;
		mFrame++;
	} 	
}
void
World::
UpdatePositionsAndVelocities(const Eigen::VectorXd& x_n1)
{
	mV = (x_n1-mX)*(1.0/mTimeStep);
	mX = x_n1;
}
Eigen::VectorXd
World::
ProjectiveDynamicsMethod()
{
	Eigen::VectorXd x_n1(3*mNumVertices);
	Eigen::VectorXd x_n1_new(3*mNumVertices);
	Eigen::VectorXd b(3*mNumVertices);
	Eigen::VectorXd d(3*mConstraintDofs);
	d.setZero();
	b= (1.0/(mTimeStep*mTimeStep))*mMassMatrix*mQn;

	x_n1 =mQn;

	int i;
	for(i=0; i<mMaxIteration; i++) {
		EvaluateDVector(x_n1,d);
		x_n1_new = mDynamicSolver.solve(b+mJ*d);
		if((x_n1_new - x_n1).norm()/x_n1.size() < EPS) {
			break;
		} 
		x_n1 = x_n1_new;
	}
	return x_n1;
}
void
World::
PreComputation()
{
	EvaluateJMatrix(mJ);
	EvaluateLMatrix(mL);
	Eigen::SparseMatrix<double> H2ML = (1.0/(mTimeStep*mTimeStep))*mMassMatrix+mL;
	FactorizeLDLT(H2ML,mDynamicSolver);
	FactorizeLDLT(mL,mQuasiStaticSolver);
}
void
World::
EvaluateJMatrix(Eigen::SparseMatrix<double>& J) 
{
	J.resize(3*mNumVertices,3*mConstraintDofs);
	std::vector<Eigen::Triplet<double>> J_triplets;
	int index = 0;
	for(int i =0;i<mConstraints.size();i++)
	{
		mConstraints[i]->EvaluateJMatrix(index,J_triplets);
		index+=mConstraints[i]->GetDof();
	}
	J.setFromTriplets(J_triplets.cbegin(), J_triplets.cend());
}
void
World::
EvaluateLMatrix(Eigen::SparseMatrix<double>& L) 
{
	L.resize(3*mNumVertices,3*mNumVertices);

	std::vector<Eigen::Triplet<double>> L_triplets;

	for(auto c : mConstraints)
		c->EvaluateLMatrix(L_triplets);

	L.setFromTriplets(L_triplets.cbegin(), L_triplets.cend());
}
void
World::
EvaluateDVector(const Eigen::VectorXd& x,Eigen::VectorXd& d) 
{
	d.resize(mConstraintDofs*3);
	int n = mConstraints.size();
// #pragma omp parallel for
	for(int i=0;i<n;i++)
	{
		mConstraints[i]->EvaluateDVector(x);
	}
	int index = 0;
	for(auto& c : mConstraints)
	{
		c->GetDVector(index,d);
	}
}
void
World::
FactorizeLDLT(const Eigen::SparseMatrix<double>& A,Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>& ldltSolver)
{
	Eigen::SparseMatrix<double> A_prime = A;
	ldltSolver.analyzePattern(A_prime);
	ldltSolver.factorize(A_prime);
	double reg = 1E-6;
	bool success = true;
	while (ldltSolver.info() != Eigen::Success)
	{
	    reg *= 10;
	    A_prime = A + reg*mIdentityMatrix;
	    ldltSolver.factorize(A_prime);
	    success = false;
	}
	if (!success)
	    std::cout << "factorize failure (damping : " << reg<<" )"<<std::endl;
}
void
World::
SetExternalForce(Eigen::VectorXd external_force)
{
	mExternalForces = external_force;
}