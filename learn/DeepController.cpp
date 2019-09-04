#include "DeepController.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <cmath>
#include <random>
//always return 1-dim array
np::ndarray toNumPyArray(const std::vector<float>& val)
{
	int n = val.size();
	p::tuple shape = p::make_tuple(n);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape,dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	for(int i=0;i<n;i++)
	{
		dest[i] = val[i];
	}

	return array;
}

//always return 1-dim array
np::ndarray toNumPyArray(const std::vector<int>& val)
{
	int n = val.size();
	p::tuple shape = p::make_tuple(n);
	np::dtype dtype = np::dtype::get_builtin<int>();
	np::ndarray array = np::empty(shape,dtype);

	int* dest = reinterpret_cast<int*>(array.get_data());
	for(int i=0;i<n;i++)
	{
		dest[i] = val[i];
	}

	return array;
}

//always return 1-dim array
np::ndarray toNumPyArray(const std::vector<bool>& val)
{
	int n = val.size();
	p::tuple shape = p::make_tuple(n);
	np::dtype dtype = np::dtype::get_builtin<bool>();
	np::ndarray array = np::empty(shape,dtype);

	bool* dest = reinterpret_cast<bool*>(array.get_data());
	for(int i=0;i<n;i++)
	{
		dest[i] = val[i];
	}

	return array;
}

np::ndarray toNumPyArray(const std::vector<Eigen::VectorXd>& val)
{
	int n =val.size();
	int m = val[0].rows();
	p::tuple shape = p::make_tuple(n,m);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape,dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	int index = 0;
	for(int i=0;i<n;i++)
	{
		for(int j=0;j<m;j++)
		{
			dest[index++] = val[i][j];
		}
	}

	return array;	
}

//always return 1-dim array
np::ndarray toNumPyArray(const Eigen::VectorXd& vec)
{
	int n = vec.rows();
	p::tuple shape = p::make_tuple(n);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape,dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	for(int i =0;i<n;i++)
	{
		dest[i] = vec[i];
	}

	return array;
}
//always return 2-dim array
np::ndarray toNumPyArray(const Eigen::MatrixXd& matrix)
{
	int n = matrix.rows();
	int m = matrix.cols();

	p::tuple shape = p::make_tuple(n,m);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape,dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	int index = 0;
	for(int i=0;i<n;i++)
	{
		for(int j=0;j<m;j++)
		{
			dest[index++] = matrix(i,j);
		}
	}

	return array;
}
p::dict toNumPyDict(const std::map<std::string,double>& map)
{
	np::dtype dtype = np::dtype::get_builtin<float>();
	p::dict dict;

	for(auto const& iter: map) {
    	dict[iter.first] = iter.second;
    	// std::cout << iter.first << "\t" << iter.second << std::endl;
	}

	return dict;
}
Eigen::VectorXd toEigenVector(const np::ndarray& array,int n)
{
	Eigen::VectorXd vec(n);

	float* srcs = reinterpret_cast<float*>(array.get_data());

	for(int i=0;i<n;i++)
	{
		vec[i] = srcs[i];
	}
	return vec;
}
Eigen::MatrixXd toEigenMatrix(const np::ndarray& array,int n,int m)
{
	Eigen::MatrixXd mat(n,m);

	float* srcs = reinterpret_cast<float*>(array.get_data());

	int index = 0;
	for(int i=0;i<n;i++)
	{
		for(int j=0;j<m;j++)
		{
			mat(i,j) = srcs[index++];
		}
	}
	return mat;
}

DeepController::
DeepController(std::string name)
{
	std::cout<<"==================================================="<<std::endl;
	std::cout<<"Class to Python is "<<name<<std::endl;
	std::cout<<"Project dir : "<<SOFTCON_DIR<<std::endl;
	mEnvironment = new Environment();
}
np::ndarray
DeepController::
GetStates()
{
	Eigen::VectorXd states;	
	states = mEnvironment->GetStates();

	return toNumPyArray(states);
}
void
DeepController::
SetActions(np::ndarray a,int n)
{
	Eigen::VectorXd a_eigen = toEigenVector(a,n);
	mEnvironment->SetActions(a_eigen);
}
void
DeepController::
Step()
{
	mEnvironment->Step();
}
p::dict 
DeepController::
GetRewards()
{
	auto reward =mEnvironment->GetRewards();
	return toNumPyDict(reward);
}
void 
DeepController::
Reset()
{
	mEnvironment->Reset();
}
bool
DeepController::
isEndOfEpisode()
{
	bool eoe;	
	eoe = mEnvironment->isEndOfEpisode();
	return eoe;
}
void
DeepController::
UpdateRandomTargetVelocity()
{
	mEnvironment->UpdateRandomTargetVelocity();
}
np::ndarray
DeepController::
GetTargetVelocity()
{
	Eigen::Vector3d target_velocity = mEnvironment->GetTargetVelocity();
	Eigen::VectorXd target_velocity_eigen(3);
	target_velocity_eigen = target_velocity;

	return toNumPyArray(target_velocity_eigen);
}
np::ndarray
DeepController::
GetNormUpperBound()
{
	Eigen::VectorXd norm_upper_bound = mEnvironment->GetNormUpperBound();
	return toNumPyArray(norm_upper_bound);	
}
np::ndarray
DeepController::
GetNormLowerBound()
{
	Eigen::VectorXd norm_lower_bound = mEnvironment->GetNormLowerBound();
	return toNumPyArray(norm_lower_bound);
}
using namespace boost::python;
BOOST_PYTHON_MODULE(deeprl)
{
	Py_Initialize();
	np::initialize();

	class_<DeepController>("Env",init<std::string>())
		.def("GetStates",&DeepController::GetStates)
		.def("SetActions",&DeepController::SetActions)
		.def("Step",&DeepController::Step)
		.def("GetRewards",&DeepController::GetRewards)
		.def("isEndOfEpisode",&DeepController::isEndOfEpisode)
		.def("Reset",&DeepController::Reset)
		.def("GetTimeStep",&DeepController::GetTimeStep)
		.def("GetTargetVelocity",&DeepController::GetTargetVelocity)
		.def("UpdateRandomTargetVelocity",&DeepController::UpdateRandomTargetVelocity)
		.def("GetSimulationHz",&DeepController::GetSimulationHz)
		.def("GetControlHz",&DeepController::GetControlHz)
		.def("GetNormUpperBound",&DeepController::GetNormUpperBound)
		.def("GetNormLowerBound",&DeepController::GetNormLowerBound)
		;
}