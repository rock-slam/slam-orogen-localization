/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace localization;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void MLSEventHandler::handle( const envire::Event& event )
{
    envire::MultiLevelSurfaceGrid* mls_grid = dynamic_cast<envire::MultiLevelSurfaceGrid*>(event.a.get());
    if(event.type == envire::event::ITEM && 
       (event.operation == envire::event::ADD || event.operation == envire::event::UPDATE) && 
        mls_grid)
    {
        task->updateICPModelFromMap(mls_grid);
    }
}


void Task::updateICPModelFromMap(envire::MultiLevelSurfaceGrid* mls_grid)
{
    map_pointcloud->clear();
    Eigen::Affine3f map2world_affine(map2world.getTransform());
    float vertical_distance = (mls_grid->getScaleX() + mls_grid->getScaleY()) * 0.5;
    if(vertical_distance <= 0.0)
        vertical_distance = 0.1;

    // create pointcloud from mls
    for(size_t m=0;m<mls_grid->getWidth();m++)
    {
        for(size_t n=0;n<mls_grid->getHeight();n++)
        {
            for( envire::MLSGrid::iterator cit = mls_grid->beginCell(m,n); cit != mls_grid->endCell(); cit++ )
            {
                envire::MLSGrid::SurfacePatch p( *cit );
                
                double x, y;
                mls_grid->fromGrid(m, n, x, y);
                pcl::PointXYZ point;
                point.x = x;
                point.y = y;
                if(p.isHorizontal())
                {
                    point.z = p.mean;
                    point.getVector3fMap() = map2world_affine * point.getVector3fMap();
                    map_pointcloud->push_back(point);
                }
                else if(p.isVertical())
                {
                    float min_z = (float)p.getMinZ(0);
                    float max_z = (float)p.getMaxZ(0);
                    for(float z = min_z; z <= max_z; z += vertical_distance)
                    {
                        point.z = z;
                        point.getVector3fMap() = map2world_affine * point.getVector3fMap();
                        map_pointcloud->push_back(point);
                    }
                }
            }
        }
    }
    
    if(map_pointcloud->size())
        icp->setInputTarget(map_pointcloud);
}

void Task::alignPointcloud(const std::vector<base::Vector3d>& sample_pointcloud, const envire::TransformWithUncertainty& body2odometry)
{
    PCLPointCloudPtr pcl_pointcloud(new PCLPointCloud());
    std::vector<bool> mask;
    computeSampleMask(mask, sample_pointcloud.size(), max_input_sample_count);
    pcl::PointXYZ point;
    for(unsigned i = 0; i < sample_pointcloud.size(); i++)
    {
        if(mask[i])
        {
            point.getVector3fMap() = sample_pointcloud[i].cast<float>();
            pcl_pointcloud->push_back(point);
        }
    }
    alignPointcloud(pcl_pointcloud, body2odometry);
}

void Task::alignPointcloud(const std::vector<Eigen::Vector3d>& sample_pointcloud, const envire::TransformWithUncertainty& body2odometry)
{
    PCLPointCloudPtr pcl_pointcloud(new PCLPointCloud());
    std::vector<bool> mask;
    computeSampleMask(mask, sample_pointcloud.size(), max_input_sample_count);
    pcl::PointXYZ point;
    for(unsigned i = 0; i < sample_pointcloud.size(); i++)
    {
        if(mask[i])
        {
            point.getVector3fMap() = sample_pointcloud[i].cast<float>();
            pcl_pointcloud->push_back(point);
        }
    }
    alignPointcloud(pcl_pointcloud, body2odometry);
}

void Task::alignPointcloud(const PCLPointCloudPtr sample_pointcoud, const envire::TransformWithUncertainty& body2odometry)
{
    if(!icp->getInputTarget().get())
        return;

    Eigen::Affine3d odometry_delta = last_odometry2body.getTransform() * body2odometry.getTransform();
    Eigen::Affine3d transformation_guess = last_body2world.getTransform() * odometry_delta;

    icp->setInputSource(sample_pointcoud);

    // Perform the alignment
    PCLPointCloud cloud_source_registered;
    icp->align(cloud_source_registered, transformation_guess.matrix().cast<float>());
    if(icp->hasConverged() && icp->getFitnessScore() <= _max_icp_fitness_score)
    {
        Eigen::Affine3f transformation(icp->getFinalTransformation());
        
        last_body2world.setTransform(Eigen::Affine3d(transformation));

        last_body2odometry = body2odometry;
        last_odometry2body = body2odometry.inverse();
    }
    else
    {
        RTT::log(RTT::Info) << "ICP alignment failed, perhaps a model update is necessary." << RTT::endlog();
        new_state = ICP_ALIGNMENT_FAILED;
    }

}

void Task::computeSampleMask(std::vector<bool>& mask, unsigned pointcloud_size, unsigned samples_count)
{
    mask.clear();
    if(samples_count == 0)
        return;
    if(samples_count >= pointcloud_size)
    {
        mask.resize(pointcloud_size, true);
        return;
    }
    
    mask.resize(pointcloud_size, false);
    unsigned samples_drawn = 0;
    
    while(samples_drawn < samples_count)
    {
        unsigned index = rand() % pointcloud_size;
        if(mask[index] == false)
        {
            mask[index] = true;
            samples_drawn++;
        }
    }
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;

    // set inital transformations
    last_body2world = envire::TransformWithUncertainty::Identity();
    last_body2odometry = envire::TransformWithUncertainty::Identity();
    last_odometry2body = envire::TransformWithUncertainty::Identity();
    map2world = envire::TransformWithUncertainty::Identity();
    
    // reset map point cloud
    map_pointcloud.reset(new PCLPointCloud());

    // setup environment
    env.reset(new envire::Environment());
    env->addEventHandler(new MLSEventHandler(this));

    // set icp config
    icp.reset(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
    icp->setMaxCorrespondenceDistance(2.5);
    icp->setMaximumIterations(50);
    icp->setTransformationEpsilon(1e-5);
    icp->setEuclideanFitnessEpsilon(1.0);
    icp->setCorrespondenceRandomness(20);
    icp->setMaximumOptimizerIterations(20);
    icp->setRotationEpsilon(2e-3);
    max_input_sample_count = 10000;

    if(!_environment_path.get().empty())
        env->unserialize(_environment_path);

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    // read envire events
    envire::OrocosEmitter::Ptr binary_event;
    while(_envire_map.read(binary_event) == RTT::NewData)
    {
        env->applyEvents(*binary_event);
    }
    
    new_state = RUNNING;
    TaskBase::updateHook();

    // write adjusted pose
    base::samples::RigidBodyState odometry_sample;
    if(_odometry_samples.readNewest(odometry_sample) == RTT::NewData)
    {
        Eigen::Affine3d odometry_delta = last_odometry2body.getTransform() * odometry_sample.getTransform();
        odometry_sample.setTransform(last_body2world.getTransform() * odometry_delta);
        _pose_samples.write(odometry_sample);
    }

    // write state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
