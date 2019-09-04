#include "Octopus.h"
#include <fstream>
#define LOCAL	0
#define GLOBAL	1
using namespace FEM;
Octopus::
Octopus(const double& muscle_stiffness,const double& youngs_modulus,const double& poisson_ratio)
	:mMesh(),mMuscleStiffness(muscle_stiffness),mYoungsModulus(youngs_modulus),mPoissonRatio(poisson_ratio)
{
}
void 
Octopus::
SetOctopus(const std::string& path)
{
	std::cout <<"+ Meta Data Path: " << path << std::endl;
	std::ifstream meta_file(path);
	std::string read_line;

	std::getline(meta_file,read_line);
	Eigen::Affine3d T=Eigen::Affine3d::Identity();
	T(0,0) *= 0.05; T(1,1) *= 0.05; T(2,2) *= 0.05;	
	SetMesh(read_line,T);

	std::getline(meta_file,read_line);
	MakeMuscles(std::string(SOFTCON_DIR)+"/data/"+read_line,1.0);

	std::getline(meta_file,read_line);
	std::ifstream ifs(std::string(SOFTCON_DIR)+"/data/"+read_line);
    std::string str;
    std::stringstream ss;
    int index;
    str.clear();
    ss.clear();
    std::getline(ifs,str);
    ss.str(str);
    while(ss>>index) {
        mSamplingIndex.push_back(index);
    }
    ifs.close();

    ss.clear();
	std::getline(meta_file,read_line);
	mCenterIndex = atoi(read_line.c_str());

	ss.clear();
	std::getline(meta_file,read_line);
	mEndEffectorIndex = atoi(read_line.c_str());

	ss.clear();
	std::getline(meta_file,read_line);
	ss.str(read_line.c_str());    
	ss>>mLocalContourIndex[0]>>mLocalContourIndex[1]>>mLocalContourIndex[2];
    meta_file.close();
}
Eigen::Matrix3d
Octopus::
GetReferenceRotation(bool type,const Eigen::VectorXd& x)
{
	int idx0 = mLocalContourIndex[0];
	int idx1 = mLocalContourIndex[1];
	int idx2 = mLocalContourIndex[2];

	Eigen::Vector3d p0 = x.block<3,1>(3*idx0,0);
	Eigen::Vector3d p1 = x.block<3,1>(3*idx1,0);
	Eigen::Vector3d p2 = x.block<3,1>(3*idx2,0);

	Eigen::Vector3d v_x = (p1-p0).normalized();
	Eigen::Vector3d v_y = (p1-p0).cross((p2-p0)).normalized();
	Eigen::Vector3d v_z = v_x.cross(v_y).normalized();

	Eigen::Matrix3d R;
	R(0,0) = v_x[0];  R(0,1) = v_y[0];  R(0,2) = v_z[0];
	R(1,0) = v_x[1];  R(1,1) = v_y[1];  R(1,2) = v_z[1];
	R(2,0) = v_x[2];  R(2,1) = v_y[2];  R(2,2) = v_z[2];

	if (type == LOCAL)
		return R.transpose();
	else
		return R;
}
void
Octopus::
SetInitReferenceRotation(const Eigen::VectorXd& x)
{
	mInitReferenceMatrix=GetReferenceRotation(GLOBAL,x);
}
Eigen::Vector3d
Octopus::
GetUpVector(const Eigen::VectorXd& x)
{
	Eigen::Matrix3d R = GetReferenceRotation(GLOBAL,x);
    auto v_up_simul = R*mInitReferenceMatrix.inverse().col(1);
    return v_up_simul;
}
Eigen::Vector3d
Octopus::
GetForwardVector(const Eigen::VectorXd& x)
{
	Eigen::Matrix3d R = GetReferenceRotation(GLOBAL,x);
    return R.col(1);
}
void 
Octopus::
Initialize(FEM::World* world)
{
	const auto& vertices = mMesh->GetVertices();
	const auto& tetrahedras = mMesh->GetTetrahedrons();

	std::vector<Eigen::Vector3i> triangles;
	std::vector<std::pair<Eigen::Vector3i,int>> surfaces;

	for(const auto& tet : tetrahedras)
	{
		int i0,i1,i2,i3;
		Eigen::Vector3d p0,p1,p2,p3;
		
		i0 = tet[0];
		i1 = tet[1];
		i2 = tet[2];
		i3 = tet[3];
		p0 = vertices[i0];
		p1 = vertices[i1];
		p2 = vertices[i2];
		p3 = vertices[i3];

		Eigen::Matrix3d Dm;

		Dm.block<3,1>(0,0) = p1 - p0;
		Dm.block<3,1>(0,1) = p2 - p0;
		Dm.block<3,1>(0,2) = p3 - p0;
		if(Dm.determinant()<0)
		{
			i0 = tet[0];
			i1 = tet[2];
			i2 = tet[1];
			i3 = tet[3];
			p0 = vertices[i0];
			p1 = vertices[i1];
			p2 = vertices[i2];
			p3 = vertices[i3];
			Dm.block<3,1>(0,0) = p1 - p0;
			Dm.block<3,1>(0,1) = p2 - p0;
			Dm.block<3,1>(0,2) = p3 - p0;
		}
		mConstraints.push_back(new CorotateFEMConstraint(mYoungsModulus,mPoissonRatio,i0,i1,i2,i3,
			1.0/6.0*(Dm.determinant()),Dm.inverse()));
		int sorted_idx[4] ={i0,i1,i2,i3};
		std::sort(sorted_idx,sorted_idx+4);
		triangles.push_back(Eigen::Vector3i(sorted_idx[0],sorted_idx[1],sorted_idx[2]));
		triangles.push_back(Eigen::Vector3i(sorted_idx[0],sorted_idx[1],sorted_idx[3]));
		triangles.push_back(Eigen::Vector3i(sorted_idx[0],sorted_idx[2],sorted_idx[3]));
		triangles.push_back(Eigen::Vector3i(sorted_idx[1],sorted_idx[2],sorted_idx[3]));
	}
	for(int i=0;i<triangles.size();i++)
	{
		Eigen::Vector3i t_i = triangles[i];
		bool unique = true;
		for(int j=0;j<triangles.size();j++)
		{
			if(i==j)
				continue;
			if(t_i.isApprox(triangles[j]))
				unique = false;
		}
		if(unique)
			surfaces.push_back(std::make_pair(t_i,i/4));
	}
	for(int i=0;i<surfaces.size();i++)
	{
		Eigen::Vector3i t_i = surfaces[i].first;
		int tet_index = surfaces[i].second;
		
		int i0 = tetrahedras[tet_index][0], i1 = tetrahedras[tet_index][1], i2 = tetrahedras[tet_index][2], i3 = tetrahedras[tet_index][3];
		Eigen::Vector3d p0 = vertices[i0],p1 = vertices[i1],p2 = vertices[i2],p3 = vertices[i3];
		Eigen::Vector3d center = 0.25*(p0+p1+p2+p3);

		Eigen::Vector3d q0 = vertices[t_i[0]],q1 = vertices[t_i[1]],q2 = vertices[t_i[2]];
		Eigen::Vector3d n = (q1-q0).cross(q2-q1);

		if((center-q0).dot(n)>0.0)
		{
			int j1 = t_i[0];
			t_i[0] = t_i[1];
			t_i[1] = j1;
		}

		mContours.push_back(t_i);
	}

	for(const auto& m : mMuscles)
		for(const auto& s : m->GetSegments()) 
			for(int i=0; i<s->GetMuscleConstraints().size(); i++)
				mConstraints.push_back(s->GetMuscleConstraints()[i]);

	Eigen::VectorXd v(vertices.size()*3);
	for(int i =0;i<vertices.size();i++)
		v.block<3,1>(i*3,0) = vertices[i];

	world->AddBody(v,mConstraints,10.0);

	SetKey("octopus.param");
}
void 
Octopus::
SetKey(std::string filename)
{
	Eigen::MatrixXd mGivenKey,key;

	int row=50, col=mMuscles.size();
	
	mGivenKey.resize(row,col);

	for(int i=0; i<20; i++) {
		mGivenKey.row(i)<<0.025,-0.0625,0.025,-0.0625,0.025,-0.0625,0.025,-0.0625,0.025,-0.0625,0.025,-0.0625,0.025,-0.0625,0.025,-0.0625;
	}
	for(int i=20; i<30; i++) {
		mGivenKey.row(i)<<-0.125,0.03125,-0.125,0.03125,-0.125,0.03125,-0.125,0.03125,-0.125,0.03125,-0.125,0.03125,-0.125,0.03125,-0.125,0.03125;
	}
	for(int i=30; i<50; i++) {
		mGivenKey.row(i)<<0.0,-0.125,0.0,-0.125,0.0,-0.125,0.0,-0.125,0.0,-0.125,0.0,-0.125,0.0,-0.125,0.0,-0.125;
	}

	key.resize(mGivenKey.rows(),mGivenKey.cols());
	key.setZero();

	for(int j=0; j<key.cols(); j++) {
		for(int i=0; i<key.rows(); i++) {
			if(i==0) {
				key(i,j) += mGivenKey(i,j);
			} else {
				key(i,j) += key(i-1,j) + mGivenKey(i,j);
			}

			key(i,j) = std::min(std::max(key(i,j),0.0),1.0);
		}
	}

	for(int i=0; i<key.cols(); i++) {
		mMuscles[i]->SetKey(key.col(i));
	}
}
void
Octopus::
SetActivationLevels(const Eigen::VectorXd& actions,const int& phase)
{
	int idx =0;
	for(int i=0; i<mMuscles.size(); i++)
	{
		mMuscles[i]->SetActivationLevels(actions.segment(idx,4),phase);
		idx+=4;
	}
}
void 
Octopus::
SetMesh(const std::string& path,const Eigen::Affine3d& T)
{
	mScalingMatrix = T;
	mMesh = new OBJMesh(std::string(SOFTCON_DIR)+"/data/"+path,mScalingMatrix);
}
void 
Octopus::
MakeMuscles(const std::string& path,const double& gamma)
{
	std::ifstream ifs(path);
	if(!ifs.is_open()){
		std::cout << "Muscle file doesn't exist." << std::endl;
		exit(0);
	}
	std::string str;
	std::string index;
	std::stringstream ss;
	
	int fiber_type;
	int num_sampling;

	int current_fiber_index = 0;
	
	std::vector<Eigen::Vector3d> point_list;

	while(!ifs.eof())
	{
		str.clear();
		index.clear();
		ss.clear();

		std::getline(ifs,str);
		ss.str(str);

		int fiber_index;
		ss>>fiber_index;

		if(fiber_index != current_fiber_index) {
			fiber_type = 0;
			num_sampling = 50;

			mMuscles.push_back(new Muscle(num_sampling,point_list));
			Muscle* muscle = mMuscles.back();
			muscle->Initialize(mMesh,mMuscleStiffness);
			point_list.clear();
			current_fiber_index = fiber_index;
		}

		int segment_index;
		ss>>segment_index;

		Eigen::Vector3d point;
		ss>>point[0]>>point[1]>>point[2];

		point = mScalingMatrix*point;
		point_list.push_back(point);
	}

	mMuscles.push_back(new Muscle(num_sampling,point_list));
	Muscle* muscle = mMuscles.back();
	muscle->Initialize(mMesh,mMuscleStiffness);
}
double
C_d(double theta)
{
	return -cos(theta*2.0)+1.05;
}
double
C_t(double theta)
{
	return 0.25*(exp(0.8*theta) -1);
}
double 
Cd(double aoa,double a,double c) {
	return (a-c) * (0.5*(-cos(2.0*aoa)+1.0)) + c;
}
double 
_interpolate(double t, double a, double b)
{
    return  (1.0 - t) * a + t * b;
}
double 
Cl(double aoa, double cutoff, double x[5], double y[5]) {
    const   double  xa = x[0],   ya = y[0];
    const   double  xb = x[1],   yb = y[1];
    const   double  xc = x[2],   yc = y[2];
    const   double  xd = x[3],   yd = y[3];
    const   double  xe = x[4],   ye = y[4];

    double  theta = aoa * 180.0 / M_PI;

    if (fabs(theta) > cutoff)
        return  0.0;

    if (xa <= theta && theta < xb)
        return _interpolate((theta - xa) / (xb - xa), ya, yb);
    else if (xb <= theta && theta < xc)
        return _interpolate((theta - xb) / (xc - xb), yb, yc);
    else if (xc <= theta && theta <= xd)
        return _interpolate((theta - xc) / (xd - xc), yc, yd);
    else if (xd <= theta && theta <= xe)
        return _interpolate((theta - xd) / (xe - xd), yd, ye);
    else
    {
        std::cout << "Error: this should not be reached... " << std::endl;
        std::cout << "Theta: " << aoa << "(deg) " << theta << "(rad)" << std::endl;
        std::cout << "x: " << xa << " " << xb << " " << xc << " " << xd << std::endl;
        std::cout << "y: " << ya << " " << yb << " " << yc << " " << yd << std::endl;
        std::cin.get();
    }
    return 0.0;
}
Eigen::VectorXd 
Octopus::
ComputeDragForces(FEM::World* world)
{
	// const Eigen::VectorXd& x = world->GetPositions();
	// const Eigen::VectorXd& x_dot = world->GetVelocities();

	// int n = x.rows();
	// Eigen::VectorXd f = Eigen::VectorXd::Zero(n);
	// Eigen::Vector3d f_sum = Eigen::Vector3d::Zero(); 
	// Eigen::Vector3d avg_vel = Eigen::Vector3d::Zero();

	// for(const auto& con : mContours)
	// {
	// 	int i0 = con[0];
	// 	int i1 = con[1];
	// 	int i2 = con[2];

	// 	Eigen::Vector3d p0 = x.segment<3>(i0*3);
	// 	Eigen::Vector3d p1 = x.segment<3>(i1*3);
	// 	Eigen::Vector3d p2 = x.segment<3>(i2*3);
	// 	Eigen::Vector3d com = (p0+p1+p2)/3.0;

	// 	Eigen::Vector3d v0 = x_dot.segment<3>(i0*3);
	// 	Eigen::Vector3d v1 = x_dot.segment<3>(i1*3);
	// 	Eigen::Vector3d v2 = x_dot.segment<3>(i2*3);

	// 	Eigen::Vector3d v = -(v0+v1+v2)/3.0;
	// 	if(v.norm()<1E-6)
	// 		continue;
	// 	Eigen::Vector3d n = (p1-p0).cross(p2-p1);
		
	// 	double area = 0.5*n.norm();
	// 	n.normalize();
	// 	n = -n;
	
	// 	double theta = atan2(v.dot(n),(v-v.dot(n)*n).norm());
	// 	Eigen::Vector3d d = v.normalized();
	// 	Eigen::Vector3d t = -n;
	// 	Eigen::Vector3d fv = GetForwardVector(x);
	// 	double theta2 = atan2(t.dot(fv),(t-t.dot(fv)*fv).norm());
	// 	Eigen::Vector3d l = d.cross(n);

	// 	l.normalize();
	// 	l = l.cross(d);

	// 	double f_d = 0.5*1000.0*area*v.squaredNorm()*C_d(theta)*0.5;
	// 	double f_t = 0.5*1000.0*area*v.squaredNorm()*C_t(theta2)*std::abs(fv.dot(t))*2;
	// 	Eigen::Vector3d f_i = 0.333*(f_d*d+f_t*t);
	// 	f.segment<3>(i0*3) += f_i;
	// 	f.segment<3>(i1*3) += f_i;
	// 	f.segment<3>(i2*3) += f_i;
	// }
	// double clip_f = 1E3;
	// for(int i =0;i<n;i++)
	// {
	// 	f[i] = std::max(-clip_f,std::min(clip_f,f[i]));
	// }
	// return f;
	Eigen::VectorXd force_drag = Eigen::VectorXd::Zero(3*world->GetNumVertices());
	Eigen::VectorXd force_lift = Eigen::VectorXd::Zero(3*world->GetNumVertices());
	Eigen::VectorXd force_total = Eigen::VectorXd::Zero(3*world->GetNumVertices());
	Eigen::VectorXd velocities = world->GetVelocities();
	Eigen::VectorXd positions = world->GetPositions();

	double max_force = 1.0e03;

	double cl_x[5] = {-90, -10, -5, 15, 90};
	double cl_y[5] = {-0.5, -1.26, 0.0, 1.8, 0.5};
	double cutoff = 85.0;

	for(int i=0; i<mContours.size(); i++)
	{
		Eigen::Vector3d p1 = positions.segment<3>(mContours[i][0]*3);
		Eigen::Vector3d p2 = positions.segment<3>(mContours[i][1]*3);
		Eigen::Vector3d p3 = positions.segment<3>(mContours[i][2]*3);
		Eigen::Vector3d v1 = velocities.segment<3>(mContours[i][0]*3);
		Eigen::Vector3d v2 = velocities.segment<3>(mContours[i][1]*3);
		Eigen::Vector3d v3 = velocities.segment<3>(mContours[i][2]*3);

		Eigen::Vector3d v_water(0.0, 0.0, 0.0);
		Eigen::Vector3d v_rel = v_water - (v1+v2+v3)/3.0;
		if(v_rel.norm() < 1e-6) continue;
		Eigen::Vector3d v_rel_norm = v_rel.normalized();

		Eigen::Vector3d n = ((p2-p1).cross(p3-p1)).normalized();
		Eigen::Vector3d d = -n;
		Eigen::Vector3d d_lift(0.0, 0.0, 0.0);

		Eigen::Vector3d force_drag_per_face(0.0, 0.0, 0.0);
		Eigen::Vector3d force_lift_per_face(0.0, 0.0, 0.0);

		double area = 0.5*((p2-p1).cross(p3-p1)).norm();

		if (area < 1.0e-10)
		{
			std::cerr << "Error: Area is too small, you should check the simulation" << std::endl;
			std::cin.get();
		}

		double d_dot_v_rel_norm = d.dot(v_rel_norm);
		double aoa = 0.5 * M_PI - acos(d_dot_v_rel_norm);
		double f = 1.0;
	
		// Ignore faces which have reverse direction 
		if (d_dot_v_rel_norm > 0.0)
		{
			if (d_dot_v_rel_norm > 1.0 || d_dot_v_rel_norm < -1.0)
			{
				std::cerr << "Error: this should not be reached... " << std::endl;
				std::cerr << "d_dot_v_rel_norm: " << d_dot_v_rel_norm << std::endl;
				std::cerr << "Drag: " << d.transpose() << std::endl;
				std::cerr << "Vel: " << v_rel_norm.transpose() << std::endl;
				std::cin.get();
			} 

			if (fabs(aoa) > 0.5 * M_PI)
			{
				std::cerr << "Error: this should not be reached... " << std::endl;
				std::cerr << "aoa: " << aoa << std::endl;
				std::cin.get();
			}

			force_drag_per_face = 0.5 * 1000.0 * (1.0*area) * Cd(aoa,f,0.0) * v_rel.squaredNorm() * v_rel_norm;
		}

		if (d_dot_v_rel_norm > 0.0)
		{
			d_lift = (v_rel_norm.cross(d)).normalized().cross(v_rel_norm);
			force_lift_per_face = 0.5 * 1000.0 * (1.0*area) * Cl(aoa,cutoff,cl_x,cl_y) * v_rel.squaredNorm() * d_lift;
		}

		if (force_drag_per_face.norm() > max_force)
		{
			std::cerr << "Error: max_force reached..." << std::endl;
			force_drag_per_face = max_force * force_drag_per_face.normalized();
		}

		if (force_lift_per_face.norm() > max_force)
		{
			std::cerr << "Error: max_force reached..." << std::endl;
			force_lift_per_face = max_force * force_lift_per_face.normalized();
		}

		force_total.segment<3>(mContours[i][0]*3) += force_drag_per_face/3.0 + force_lift_per_face/3.0;
		force_total.segment<3>(mContours[i][1]*3) += force_drag_per_face/3.0 + force_lift_per_face/3.0;
		force_total.segment<3>(mContours[i][2]*3) += force_drag_per_face/3.0 + force_lift_per_face/3.0;
	}

	return force_total;
}
void 
Octopus::
SetVertexNormal()
{
	const auto& vertices = mMesh->GetVertices();
	const auto& tetrahedras = mMesh->GetTetrahedrons();

	mVertexNormal.resize(3*vertices.size());
	mVertexNormal.setZero();
	Eigen::VectorXd cnt_list(vertices.size());
	cnt_list.setZero();

	for(int i=0;i<mContours.size();i++)
	{
		int i0 = mContours[i][0];
		int i1 = mContours[i][1];
		int i2 = mContours[i][2];

		Eigen::Vector3d p0 = vertices[i0];
		Eigen::Vector3d p1 = vertices[i1];
		Eigen::Vector3d p2 = vertices[i2];

		Eigen::Vector3d face_normal;
		face_normal = (p1-p0).cross(p2-p1);

		mVertexNormal.block<3,1>(3*i0,0) = cnt_list[i0]/(cnt_list[i0]+1)*mVertexNormal.block<3,1>(3*i0,0) + 1/(cnt_list[i0]+1)*face_normal;
		mVertexNormal.block<3,1>(3*i1,0) = cnt_list[i1]/(cnt_list[i1]+1)*mVertexNormal.block<3,1>(3*i1,0) + 1/(cnt_list[i1]+1)*face_normal;
		mVertexNormal.block<3,1>(3*i2,0) = cnt_list[i2]/(cnt_list[i2]+1)*mVertexNormal.block<3,1>(3*i2,0) + 1/(cnt_list[i2]+1)*face_normal;

		cnt_list[i0] += 1;
		cnt_list[i1] += 1;
		cnt_list[i2] += 1;
	}
}