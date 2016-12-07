/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PointcloudInMLS.hpp"

using namespace localization;



PointcloudInMLS::PointcloudInMLS(std::string const& name)
    : PointcloudInMLSBase(name)
{
}

PointcloudInMLS::PointcloudInMLS(std::string const& name, RTT::ExecutionEngine* engine)
    : PointcloudInMLSBase(name, engine)
{
}

PointcloudInMLS::~PointcloudInMLS()
{
}

void PointcloudInMLS::pointcloud_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &pointcloud_samples_sample)
{
    Eigen::Affine3d pointcloud2body;
    if (!_pointcloud2body.get(ts, pointcloud2body))
    {
        RTT::log(RTT::Error) << "skip, have no pointcloud2body transformation sample!" << RTT::endlog();
        new_state = TaskBase::MISSING_TRANSFORMATION;
        return;
    }
    
    if(newICPRunPossible(ts))
    {
        PCLPointCloudPtr pcl_pointcloud(new PCLPointCloud());
        if(!pointcloud2body.matrix().isApprox(Eigen::Matrix4d::Identity()))
        {
            // apply transformation
            std::vector<base::Vector3d> transformed_pointcloud;
            transformed_pointcloud.reserve(pointcloud_samples_sample.points.size());
            for(std::vector<base::Vector3d>::const_iterator it = pointcloud_samples_sample.points.begin(); it != pointcloud_samples_sample.points.end(); it++)
            {
                transformed_pointcloud.push_back(pointcloud2body * (*it));
            }
            convertBaseToPCLPointCloud(transformed_pointcloud, *pcl_pointcloud);
        }
        else
            convertBaseToPCLPointCloud(pointcloud_samples_sample.points, *pcl_pointcloud);

        alignPointcloud(ts, pcl_pointcloud);
    }
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PointcloudInMLS.hpp for more detailed
// documentation about them.

bool PointcloudInMLS::configureHook()
{
    if (! PointcloudInMLSBase::configureHook())
        return false;
    
    body_frame = _body_frame.get();

    _transformer.registerTransformCallback(_body2odometry, boost::bind(&Task::integrateOdometry, this, _1, _2));
    
    return true;
}
bool PointcloudInMLS::startHook()
{
    if (! PointcloudInMLSBase::startHook())
        return false;
    return true;
}
void PointcloudInMLS::updateHook()
{
    //process all callbacks
    PointcloudInMLSBase::updateHook();
}
void PointcloudInMLS::errorHook()
{
    PointcloudInMLSBase::errorHook();
}
void PointcloudInMLS::stopHook()
{
    PointcloudInMLSBase::stopHook();
}
void PointcloudInMLS::cleanupHook()
{
    PointcloudInMLSBase::cleanupHook();
}
