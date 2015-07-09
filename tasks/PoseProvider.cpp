/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PoseProvider.hpp"
#include "graph_slam/GraphSlamDebugTypes.hpp"

using namespace localization;

PoseProvider::PoseProvider(std::string const& name)
    : PoseProviderBase(name)
{
}

PoseProvider::PoseProvider(std::string const& name, RTT::ExecutionEngine* engine)
    : PoseProviderBase(name, engine)
{
}

PoseProvider::~PoseProvider()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PoseProvider.hpp for more detailed
// documentation about them.

bool PoseProvider::configureHook()
{
    if (! PoseProviderBase::configureHook())
        return false;
    
    last_body2world = Eigen::Affine3d::Identity();
    last_odometry2body = Eigen::Affine3d::Identity();
    have_valid_pose = false;
    
    return true;
}
bool PoseProvider::startHook()
{
    if (! PoseProviderBase::startHook())
        return false;
    return true;
}
void PoseProvider::updateHook()
{
    PoseProviderBase::updateHook();
    
    graph_slam::PoseProviderUpdate pose_update;
    if (_pose_provider_update.readNewest(pose_update) == RTT::NewData) 
    {
	last_body2world = pose_update.body2world.toTransform();
	last_odometry2body = pose_update.body2odometry.toTransform().inverse();
	have_valid_pose = true;
    }
    
    base::samples::RigidBodyState odometry_pose;
    if (have_valid_pose && _odometry_samples.readNewest(odometry_pose) == RTT::NewData)
    {
	odometry_pose.setTransform(last_body2world * (last_odometry2body * odometry_pose.getTransform()));
	odometry_pose.targetFrame = _outputFrameName.get();
	_pose_samples.write(odometry_pose);
    }
}
void PoseProvider::errorHook()
{
    PoseProviderBase::errorHook();
}
void PoseProvider::stopHook()
{
    PoseProviderBase::stopHook();
}
void PoseProvider::cleanupHook()
{
    PoseProviderBase::cleanupHook();
}
