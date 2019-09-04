#include "Muscle.h"
Muscle::
Muscle(int num_sampling,std::vector<Eigen::Vector3d> point_list)
	:mNumSampling(num_sampling)
{
	mActions.resize(4);
	mActions.setZero();

	mActionUpperBound.resize(4);
	mActionLowerBound.resize(4);

	mActionUpperBound[0] = 0.25;
	mActionLowerBound[0] = -0.25;

	mActionUpperBound[1] = 1.0;
	mActionLowerBound[1] = 0.0;
	
	mActionUpperBound[2] = 1.0;
	mActionLowerBound[2] = 0.0;

	mActionUpperBound[3] = 10.0;
	mActionLowerBound[3] = 0.0;

	double arc_length =0.0;
	for(int i=0; i<point_list.size()-1; i++) {
		arc_length += (point_list[i+1]-point_list[i]).norm();
	}		

	double sampling_length = arc_length/mNumSampling;
	double tmp_length = 0.0;

	int start_idx = 0;		

	for(int i=0; i<point_list.size(); i++) {
		tmp_length += (point_list[i+1]-point_list[i]).norm();
		if(tmp_length > sampling_length) {
			mStarts.push_back(point_list[start_idx]);
			mEnds.push_back(point_list[i]);
			tmp_length = 0.0;
			start_idx = i;
		}
	}

	mEnds.pop_back();
	mEnds.push_back(point_list[point_list.size()-1]);

	mActivationLevels.resize(mNumSampling);
	mActivationLevels.setZero();
}
void 
Muscle::
Initialize(FEM::Mesh* mesh,double muscle_stiffness)
{
	const auto& vertices = mesh->GetVertices();
	const auto& tetrahedron = mesh->GetTetrahedrons();

	for(int i=0; i<mNumSampling; i++) 
	{
		mSegments.push_back(new MuscleSegment());
		auto segment = mSegments.back();

		Eigen::Vector3d start = mStarts[i]; 
		Eigen::Vector3d end = mEnds[i]; 

		double start_min = 1E5;
		double end_min = 1E5;
		
		int start_idx = -1,end_idx = -1;

		for(int j=0;j<tetrahedron.size();j++)
		{
			int i0 = tetrahedron[j][0];
			int i1 = tetrahedron[j][1];
			int i2 = tetrahedron[j][2];
			int i3 = tetrahedron[j][3];

			Eigen::Vector3d p0 = vertices[i0],p1 = vertices[i1],p2 = vertices[i2],p3 = vertices[i3];

			Eigen::Matrix3d Dm;	
			Dm.block<3,1>(0,0) = p1 - p0;
			Dm.block<3,1>(0,1) = p2 - p0;
			Dm.block<3,1>(0,2) = p3 - p0;

			bool is_under_zero = false;
			if(Dm.determinant()<0.0)
			{
				is_under_zero = true;
				i1 = tetrahedron[j][2];
				i2 = tetrahedron[j][1];
				p1 = vertices[i1];
				p2 = vertices[i2];
				Dm.block<3,1>(0,0) = p1 - p0;
				Dm.block<3,1>(0,1) = p2 - p0;
				Dm.block<3,1>(0,2) = p3 - p0;
			}

			Eigen::Vector3d fiber_direction = (end-start).normalized();
			Eigen::Vector3d center = (p0+p1+p2+p3)/4.0;
			double dist;
			Eigen::Vector3d v = center-start;

			double t = v.dot(fiber_direction);
			Eigen::Vector3d p = start + t*fiber_direction;
			if(std::max((p-start).norm(),(p-end).norm())/(end-start).norm() < 1.0)
				dist = (p-center).norm();
			else
				dist = 1E5;

			dist = std::min(dist,std::min((center-start).norm(),(center-end).norm()));
			double weight = std::exp(-std::abs(dist)/0.005);
	
			if(isInTetra(p0,p1,p2,p3,start,end)) {
				segment->AddMuscleConstraint(new FEM::LinearMuscleConstraint(5E5,fiber_direction,0.0,i0,i1,i2,i3,1.0/6.0*Dm.determinant(),Dm.inverse(),weight));
			}
			else if(weight>0.2) {
				segment->AddMuscleConstraint(new FEM::LinearMuscleConstraint(5E5,fiber_direction,0.0,i0,i1,i2,i3,1.0/6.0*Dm.determinant(),Dm.inverse(),weight));
			}

			double D = Dm.determinant();

			double S0,S1,S2,S3;
			S0 = GetDm(start,p1,p2,p3).determinant()/D;
			S1 = GetDm(start,p2,p3,p0).determinant()/D;
			S2 = GetDm(start,p3,p0,p1).determinant()/D;
			S3 = GetDm(start,p0,p1,p2).determinant()/D;

			double E0,E1,E2,E3;
			E0 = GetDm(end,p1,p2,p3).determinant()/D;
			E1 = GetDm(end,p2,p3,p0).determinant()/D;
			E2 = GetDm(end,p3,p0,p1).determinant()/D;
			E3 = GetDm(end,p0,p1,p2).determinant()/D;

			if( std::abs(S0+S1+S2+S3-1.0)<1E-4)
			{
				start_idx = j;
				segment->SetStart(Eigen::Vector4i(i0,i1,i2,i3),Eigen::Vector4d(S0,S1,S2,S3));
				segment->AddMuscleConstraint(new FEM::LinearMuscleConstraint(5E5,fiber_direction,0.0,i0,i1,i2,i3,1.0/6.0*Dm.determinant(),Dm.inverse(),weight));
			}

			if( std::abs(E0+E1+E2+E3-1.0)<1E-4)
			{
				end_idx = j;
				segment->SetEnd(Eigen::Vector4i(i0,i1,i2,i3),Eigen::Vector4d(E0,E1,E2,E3));
				segment->AddMuscleConstraint(new FEM::LinearMuscleConstraint(5E5,fiber_direction,0.0,i0,i1,i2,i3,1.0/6.0*Dm.determinant(),Dm.inverse(),weight));
			}
		}
		if(segment->GetStartBarycentric().norm()<1E-6)
		{
			segment->SetStart(segment->GetEndIdx(),segment->GetEndBarycentric());
		}
		else if(segment->GetEndBarycentric().norm()<1E-6)
		{
			segment->SetEnd(segment->GetStartIdx(),segment->GetStartBarycentric());
		}
	}
}
void 
Muscle::
SetKey(const Eigen::VectorXd& key)
{
	mKey=key;
}
void 
Muscle::
SetActivationLevels(const Eigen::VectorXd& action,const int& phase)
{
	Eigen::VectorXd new_act(mSegments.size());
	new_act.setZero();

	double key = mKey[phase%mKey.size()];

	double delta_act = action[0];
	double alpha = action[1];
	double beta = 1.0-action[1];
	double prop_speed = action[3];

	Eigen::VectorXd prev_act = mActivationLevels;

	for(int i=0; i<prop_speed; i++) 
	{
		for(int j=0; j<mNumSampling; j++) {
			if(j == 0)  
				new_act[j] = key + delta_act;
			else
				new_act[j] = alpha*prev_act[j-1] + beta*prev_act[j];
		} 

		prev_act = new_act;
	}

	for(int i=0; i<new_act.size(); i++)
		new_act[i] = std::min(std::max(new_act[i],0.0),1.0);

	mActivationLevels = new_act;

	for(int i=0;i<mSegments.size();i++)
		mSegments[i]->SetActivationLevel(mActivationLevels[i]);
}
void 
Muscle::
Reset()
{
	mActivationLevels.setZero();
	
	for(int i =0;i<mNumSampling;i++)
	{
		mSegments[i]->SetActivationLevel(mActivationLevels[i]);
	}
}
bool
isInTetra(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& p3,const Eigen::Vector3d& start,const Eigen::Vector3d& end)
{
	if(isLineTriangleIntersect(p0,p1,p2,start,end)) {
		return true;
	}
	if(isLineTriangleIntersect(p0,p1,p3,start,end)) {
		return true;
	}
	if(isLineTriangleIntersect(p0,p2,p3,start,end)) {
		return true;	
	}
	if(isLineTriangleIntersect(p1,p2,p3,start,end)) {
		return true;
	}

	return false;
}
Eigen::Matrix3d
GetDm(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& p3)
{
	Eigen::Matrix3d Dm;

	Dm.block<3,1>(0,0) = p1 - p0;
	Dm.block<3,1>(0,1) = p2 - p0;
	Dm.block<3,1>(0,2) = p3 - p0;

	if(Dm.determinant()<0.0)
	{
		Dm.block<3,1>(0,0) = p2 - p0;
		Dm.block<3,1>(0,1) = p1 - p0;
		Dm.block<3,1>(0,2) = p3 - p0;
	}	

	return Dm;
}
double
GetSignedVolume(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& p3)
{
	return ((p1-p0).cross(p2-p0)).dot(p3-p0)/6.0;
}
bool
isLineTriangleIntersect(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& start,const Eigen::Vector3d& end)
{
	if(GetSignedVolume(start,p0,p1,p2)*GetSignedVolume(end,p0,p1,p2) < 0.0) {
		if(GetSignedVolume(start,end,p0,p1)>0.0 && GetSignedVolume(start,end,p1,p2)>0.0 && GetSignedVolume(start,end,p2,p0)>0.0) {
			return true;
		}
		if(GetSignedVolume(start,end,p0,p1)<0.0 && GetSignedVolume(start,end,p1,p2)<0.0 && GetSignedVolume(start,end,p2,p0)<0.0) {
			return true;
		}
	}

	return false;
}