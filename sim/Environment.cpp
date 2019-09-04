#include "Environment.h"
#include <chrono>
#include <random>
#include <fstream>
#define LOCAL   0
#define GLOBAL  1
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine generator(seed);
std::uniform_real_distribution<double> l_distribution(0.75,0.75);
std::uniform_real_distribution<double> angle_distribution(0.0,0.0);
std::uniform_real_distribution<double> vx_distribution(-1.0,1.0);
std::uniform_real_distribution<double> vy_distribution(-1.0,1.0);
std::uniform_real_distribution<double> vz_distribution(-1.0,1.0);
Environment::
Environment()
    :mPhase(0)
{
	mSimulationHz = 240;
    mControlHz = 30;

    mSoftWorld = new FEM::World(
        1.0/mSimulationHz,
        50,
        0.9999,
        false
    );

    mOctopus = new Octopus(5E5,2E5,0.4);
    mOctopus->SetOctopus(std::string(SOFTCON_DIR)+"/data/octopus.meta");
    mOctopus->Initialize(mSoftWorld);
    mSoftWorld->Initialize();

    mOctopus->SetInitReferenceRotation(mSoftWorld->GetPositions());

    InitializeActions();

    mTargetVelocity.setZero();

    mAverageVelocity.setZero();
    mAverageVelocityDeque.clear();

    UpdateRandomTargetVelocity();
}
void
Environment::
Step()
{
    Eigen::VectorXd external_force = mOctopus->ComputeDragForces(mSoftWorld);
    mSoftWorld->SetExternalForce(external_force);

    mSoftWorld->TimeStepping();

    Eigen::Vector3d v_front = Eigen::Vector3d(0,0,0);
    if(mAverageVelocityDeque.size() > mSimulationHz) {
        v_front = mAverageVelocityDeque.front();
        mAverageVelocityDeque.pop_front();
    }
    Eigen::Vector3d v_center = mSoftWorld->GetVelocities().block<3,1>(3*mOctopus->GetCenterIndex(),0);
    mAverageVelocityDeque.push_back(v_center);
    mAverageVelocity = mAverageVelocity - (v_front)/mSimulationHz + v_center/mSimulationHz;
}
void 
Environment::
SetPhase(const int& phase)
{
    mPhase = phase;
}
void 
Environment::
Reset()
{
    mAverageVelocityDeque.clear();
    mAverageVelocity.setZero();
    for(int i=0;i<mOctopus->GetMuscles().size();i++)
    {
        mOctopus->GetMuscles()[i]->Reset();
    }
    mSoftWorld->Reset();
    mPhase = 0;
}




// DeepRL
const Eigen::VectorXd&
Environment::
GetStates()
{
    const Eigen::VectorXd& x = mSoftWorld->GetPositions();
    const Eigen::VectorXd& v = mSoftWorld->GetVelocities();
    Eigen::Vector3d center_position = x.block<3,1>(3*mOctopus->GetCenterIndex(),0);
    
    Eigen::Matrix3d R = mOctopus->GetReferenceRotation(LOCAL,x);

    Eigen::Vector3d local_target_velocity = R*mTargetVelocity;
    Eigen::Vector3d local_average_velocity = R*mAverageVelocity;

    const std::vector<int>& sampling_index = mOctopus->GetSamplingIndex();
    Eigen::VectorXd local_position(3*sampling_index.size());
    Eigen::VectorXd local_velocity(3*sampling_index.size());

    for(int i=0; i<sampling_index.size(); i++) {
        local_position.block<3,1>(3*i,0) = R*x.block<3,1>(3*sampling_index[i],0);
        local_velocity.block<3,1>(3*i,0) = R*v.block<3,1>(3*sampling_index[i],0);
    }

    int num_states = local_target_velocity.size() + local_average_velocity.size()
        + local_position.size() + local_velocity.size();

    mStates.resize(num_states);
    mStates.setZero();

    mStates<<local_target_velocity,local_average_velocity,
        local_position,local_velocity;

    return mStates;
}
void
Environment::
InitializeActions()
{
    const auto& muscles = mOctopus->GetMuscles();

    int num_action =4*muscles.size();
    mActions.resize(num_action);

    Eigen::VectorXd real_lower_bound(num_action);
    Eigen::VectorXd real_upper_bound(num_action);

    int cnt =0;
    for(const auto& m : muscles) 
    {
        real_lower_bound.segment(cnt,4) = m->GetActionLowerBound();
        real_upper_bound.segment(cnt,4) = m->GetActionUpperBound();
        cnt+=4;
    }

    mNormLowerBound.resize(real_lower_bound.size());
    mNormUpperBound.resize(real_upper_bound.size());

    mNormLowerBound.setOnes();
    mNormLowerBound *= -5.0;
    mNormUpperBound.setOnes();
    mNormUpperBound *= 5.0;

    mNormalizer = new Normalizer(real_upper_bound,real_lower_bound,
        mNormUpperBound,mNormLowerBound);
}
void 
Environment::
SetActions(const Eigen::VectorXd& actions)
{
    Eigen::VectorXd real_actions = mNormalizer->NormToReal(actions);
    mOctopus->SetActivationLevels(real_actions,mPhase);
    mPhase+=1;
}
std::map<std::string,double>
Environment::
GetRewards()
{
    std::map<std::string,double> reward_map;

    double d = (mAverageVelocity-mTargetVelocity).norm();
    double reward_target = exp(-d*d/0.05);

    Eigen::Vector3d v_face = mOctopus->GetForwardVector(mSoftWorld->GetPositions());
    Eigen::Vector3d v_tar_dir = mTargetVelocity.normalized();
    auto d_direction = (v_face - v_tar_dir).norm();
    double reward_direction = exp(-fabs(d_direction)/0.1);

    double w_target = 1.0;
    double w_direction = 2.0;

    double reward = w_target*reward_target + w_direction*reward_direction;

    reward_map["target"] = w_target*reward_target;
    reward_map["direction"] = w_direction*reward_direction;
    reward_map["total"] = reward;

    return reward_map;
}
bool
Environment::
isEndOfEpisode()
{
    bool eoe =false; 

    return eoe;
}
void
Environment::
UpdateRandomTargetVelocity()
{
    Eigen::Vector3d v,axis;

    double length = l_distribution(generator);

    v = length*mOctopus->GetForwardVector(mSoftWorld->GetPositions()).normalized();

    double angle = angle_distribution(generator);
    axis[0] = vx_distribution(generator);
    axis[1] = vy_distribution(generator);
    axis[2] = vz_distribution(generator);

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(angle,axis.normalized());

    mTargetVelocity = R*v;
}