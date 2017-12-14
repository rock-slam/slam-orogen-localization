/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "VelodyneInMLS.hpp"
#include <depth_map_preprocessing/Filters.hpp>

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

void VelodyneInMLS::lidar_samplesTransformerCallback(const base::Time &ts, const ::base::samples::DepthMap &lidar_samples_sample)
{
    Eigen::Affine3d laser2body;
    if (!_velodyne2body.get(ts, laser2body))
    {
        RTT::log(RTT::Error) << "skip, have no laser2body transformation sample!" << RTT::endlog();
        new_state = TaskBase::MISSING_TRANSFORMATION;
        return;
    }

    if(newICPRunPossible(ts))
    {
        // filter point cloud
        base::samples::DepthMap filtered_lidar_sample = lidar_samples_sample;
        depth_map_preprocessing::Filters::filterMinDistance(filtered_lidar_sample, 1.0);
        depth_map_preprocessing::Filters::filterOutliers(filtered_lidar_sample, _maximum_angle_to_neighbor.value(), _minimum_valid_neighbors.value());
        
        // add new vertex to graph
        std::vector< Eigen::Vector3d > pointcloud;
        filtered_lidar_sample.convertDepthMapToPointCloud(pointcloud, laser2body);

        // align pointcloud to map
        PCLPointCloudPtr pcl_pointcloud(new PCLPointCloud());
        convertBaseToPCLPointCloud(pointcloud, *pcl_pointcloud);
        alignPointcloud(ts, pcl_pointcloud);
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VelodyneInMLS.hpp for more detailed
// documentation about them.

bool VelodyneInMLS::configureHook()
{
    if (! VelodyneInMLSBase::configureHook())
        return false;
    
    body_frame = _body_frame.value();

    _transformer.registerTransformCallback(_body2odometry, boost::bind(&Task::integrateOdometry, this, _1, _2));
    
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
