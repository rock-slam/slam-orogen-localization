/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "VelodyneInMLS.hpp"
#include <velodyne_lidar/pointcloudConvertHelper.hpp>

using namespace localization;

VelodyneInMLS::VelodyneInMLS(std::string const& name)
    : VelodyneInMLSBase(name)
{
}

VelodyneInMLS::VelodyneInMLS(std::string const& name, RTT::ExecutionEngine* engine)
    : VelodyneInMLSBase(name, engine)
{
}

VelodyneInMLS::~VelodyneInMLS()
{
}

void VelodyneInMLS::lidar_samplesTransformerCallback(const base::Time &ts, const ::velodyne_lidar::MultilevelLaserScan &lidar_samples_sample)
{
    Eigen::Affine3d laser2body;
    if (!_laser2body.get(ts, laser2body))
    {
        std::cerr << "skip, have no laser2body transformation sample!" << std::endl;
        return;
    }
    envire::TransformWithUncertainty body2odometry;
    if (!_body2odometry.get(ts, body2odometry, true))
    {
        std::cerr << "skip, have no body2odometry transformation sample!" << std::endl;
        return;
    }

    if((last_body2odometry.getTransform().inverse() * body2odometry.getTransform()).translation().norm() > _icp_match_interval)
    {
        // filter point cloud
        velodyne_lidar::MultilevelLaserScan filtered_lidar_sample;
        velodyne_lidar::ConvertHelper::filterOutliers(lidar_samples_sample, filtered_lidar_sample, _maximum_angle_to_neighbor, _minimum_valid_neighbors);
        
        // add new vertex to graph
        std::vector< Eigen::Vector3d > pointcloud;
        velodyne_lidar::ConvertHelper::convertScanToPointCloud(filtered_lidar_sample, pointcloud, laser2body);

        // align pointcloud to map
        alignPointcloud(pointcloud, body2odometry);
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VelodyneInMLS.hpp for more detailed
// documentation about them.

bool VelodyneInMLS::configureHook()
{
    if (! VelodyneInMLSBase::configureHook())
        return false;
    return true;
}
bool VelodyneInMLS::startHook()
{
    if (! VelodyneInMLSBase::startHook())
        return false;
    return true;
}
void VelodyneInMLS::updateHook()
{
    VelodyneInMLSBase::updateHook();
}
void VelodyneInMLS::errorHook()
{
    VelodyneInMLSBase::errorHook();
}
void VelodyneInMLS::stopHook()
{
    VelodyneInMLSBase::stopHook();
}
void VelodyneInMLS::cleanupHook()
{
    VelodyneInMLSBase::cleanupHook();
}
