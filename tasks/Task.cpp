/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/samples/Pointcloud.hpp>
#include <pcl/pcl_config.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <base-logging/Logging.hpp>
#include <base/TimeMark.hpp>
#include <pcl/filters/voxel_grid.h>


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

void Task::setModelPointCloud(const PCLPointCloudPtr& pc)
{
    model_cloud = pc;
    icp->setInputTarget(model_cloud);
}

void Task::alignPointcloud(const base::Time& ts, const PCLPointCloudPtr& sample_pointcoud, const envire::TransformWithUncertainty& body2odometry)
{
    if(sample_pointcoud->empty())
    {
        LOG_WARN_S << "Input cloud is empty!";
        return;
    }
    if(!model_cloud.get())
    {
        LOG_WARN_S << "Model cloud is missing!";
        return;
    }

    // sub sample measurement
    PCLPointCloudPtr input_pointcloud;
    if(_subsampling.value() == localization::VoxelGrid)
    {
        input_pointcloud.reset(new PCLPointCloud);
        pcl::VoxelGrid<PCLPoint> voxel_grid;
        voxel_grid.setLeafSize(_subsampling_resolution.value().x(),_subsampling_resolution.value().y(),_subsampling_resolution.value().z());
        voxel_grid.setInputCloud(sample_pointcoud);
        voxel_grid.filter(*input_pointcloud);
    }
    else
        input_pointcloud = sample_pointcoud;


    Eigen::Affine3d odometry_delta = last_odometry2body.getTransform() * body2odometry.getTransform();
    Eigen::Affine3d transformation_guess = last_body2world.getTransform() * odometry_delta;

    base::TimeMark icp_run("ICP alignment");
    Eigen::Affine3d icp_result;
    double icp_score;
    LOG_INFO_S << "Run ICP optimization";
    if(performICPOptimization(input_pointcloud, transformation_guess, icp_result, icp_score))
    {
        new_state = RUNNING;
        LOG_INFO_S << "ICP alignment successful. ICP score: " << icp_score;

        last_body2world.setTransform(icp_result);
        last_body2odometry = body2odometry;
        last_odometry2body = body2odometry.inverse();
        
        LOG_INFO_S << "Got new ICP match " << last_body2world.getTransform().translation().transpose();

        icp_debug.successful_alignments++;
        
        //write out current odometry sample
        writeNewPose(ts);
    }
    else
    {
        LOG_WARN_S << "ICP alignment failed, perhaps a model update is necessary. ICP score: " << icp_score;
        new_state = ICP_ALIGNMENT_FAILED;
        icp_debug.failed_alignments++;
    }

    LOG_INFO_S << icp_run;

    icp_debug.time = ts;
    icp_debug.last_fitness_score = icp_score;
    icp_debug.icp_alignment_time = icp_run.passed().toSeconds();
    _icp_debug_information.write(icp_debug);
    
    // write debug pointcloud
    if(_write_debug_pointcloud)
    {
        base::samples::Pointcloud debug_cloud;
        convertPCLToBasePointCloud(*model_cloud, debug_cloud.points);
        debug_cloud.colors.resize(debug_cloud.points.size(), base::Vector4d(0.9,0.9,0.9,1.));

        PCLPointCloud transformed_measurement;
        std::vector<Eigen::Vector3d> aligned_cloud;
        if(new_state != ICP_ALIGNMENT_FAILED)
        {
            pcl::transformPointCloud(*input_pointcloud, transformed_measurement, last_body2world.getTransform());
            convertPCLToBasePointCloud(transformed_measurement, aligned_cloud);
            std::vector<Eigen::Vector4d> aligned_cloud_color(aligned_cloud.size(), base::Vector4d(0.,1.,0.,1.));
            debug_cloud.points.insert(debug_cloud.points.end(), aligned_cloud.begin(), aligned_cloud.end());
            debug_cloud.colors.insert(debug_cloud.colors.end(), aligned_cloud_color.begin(), aligned_cloud_color.end());
        }

        PCLPointCloud transformed_measurement_guess;
        std::vector<Eigen::Vector3d> aligned_cloud_guess;
        pcl::transformPointCloud(*input_pointcloud, transformed_measurement_guess, transformation_guess);
        convertPCLToBasePointCloud(transformed_measurement_guess, aligned_cloud_guess);
        base::Vector4d guess_color(1.,1.,0.,0.8);
        if(new_state == ICP_ALIGNMENT_FAILED)
            guess_color = base::Vector4d(1.,0.,0.,0.8);
        std::vector<Eigen::Vector4d> aligned_cloud_guess_color(aligned_cloud_guess.size(), guess_color);
        debug_cloud.points.insert(debug_cloud.points.end(), aligned_cloud_guess.begin(), aligned_cloud_guess.end());
        debug_cloud.colors.insert(debug_cloud.colors.end(), aligned_cloud_guess_color.begin(), aligned_cloud_guess_color.end());

        _debug_map_pointcloud.write(debug_cloud);
    }
}

bool Task::performICPOptimization(const PCLPointCloudPtr& sample_pointcoud, const Eigen::Affine3d& transformation_guess, Eigen::Affine3d& result, double &icp_score)
{
    last_icp_match = base::Time::now();
    icp->setInputSource(sample_pointcoud);

    PCLPointCloud cloud_source_registered;
    #if PCL_VERSION_COMPARE(<, 1, 9, 0)
        /** Moves the model to the current measurement frame.
         *  This is nessecary due to a bug in the PCL GICP implementation.
         *  It is fixed in PCL 1.9 onwards */
        PCLPointCloudPtr transformed_model(new PCLPointCloud());
        pcl::transformPointCloud(*model_cloud, *transformed_model, transformation_guess.inverse());
        icp->setInputTarget(transformed_model);
        /** ** */

        // Perform the alignment
        icp->align(cloud_source_registered);
    #else
        // Perform the alignment
        icp->align(cloud_source_registered, transformation_guess.matrix().cast<float>());
    #endif

    icp_score = icp->getFitnessScore();
    if(icp->hasConverged() && icp_score <= gicp_config.max_mean_square_error)
    {
        #if PCL_VERSION_COMPARE(<, 1, 9, 0)
            result = transformation_guess * Eigen::Affine3d(icp->getFinalTransformation().cast<double>());
        #else
            result = Eigen::Affine3d(icp->getFinalTransformation().cast<double>());
        #endif
        return true;
    }
    return false;
}

void Task::writeNewPose(const base::Time &curTime)
{
    //comput delta between odometry position at the time of the last ICP match and the current odometry position
    base::samples::RigidBodyState sample_out;
    sample_out.invalidate();
    sample_out.time = curTime;
    sample_out.setTransform(last_body2world.getTransform());
    sample_out.sourceFrame = body_frame;
    sample_out.targetFrame = world_frame;
    _pose_samples.write(sample_out);
}

bool Task::newICPRunPossible(const Eigen::Affine3d& body2odometry) const
{
    if((last_body2odometry.getTransform().inverse() * body2odometry).translation().norm() > gicp_config.icp_match_interval ||
        (base::Time::now() - last_icp_match).toSeconds() > gicp_config.icp_match_interval_time)
        return true;
    return false;
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
    last_body2world.setTransform(_start_pose.value().toTransform());
    last_body2odometry = envire::TransformWithUncertainty::Identity();
    last_odometry2body = envire::TransformWithUncertainty::Identity();
    init_odometry = true;
    
    last_icp_match.microseconds = 0;
    
    // reset map point cloud
    model_cloud.reset();

    // set icp config
    gicp_config = _gicp_configuration.get();
    icp.reset(new pcl::GeneralizedIterativeClosestPoint<PCLPoint, PCLPoint>());
    icp->setMaxCorrespondenceDistance(gicp_config.max_correspondence_distance);
    icp->setMaximumIterations(gicp_config.maximum_iterations);
    icp->setTransformationEpsilon(gicp_config.transformation_epsilon);
    icp->setCorrespondenceRandomness(gicp_config.correspondence_randomness);
    icp->setMaximumOptimizerIterations(gicp_config.maximum_optimizer_iterations);
    icp->setRotationEpsilon(gicp_config.rotation_epsilon);
    
    // load initial pointcloud
    if(!_ply_path.value().empty())
    {
        PCLPointCloudPtr pcl_cloud(new PCLPointCloud());
        pcl::PLYReader ply_reader;
        if(ply_reader.read(_ply_path.value(), *pcl_cloud) >= 0)
            setModelPointCloud(pcl_cloud);
        else
            LOG_ERROR_S << "Failed to load PLY model point cloud!";
    }

    world_frame = _output_frame_name.value();

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
    // receive new model cloud
    base::samples::Pointcloud pointcloud;
    while(_model_pointcloud.readNewest(pointcloud, false) == RTT::NewData)
    {
        PCLPointCloudPtr pcl_pc(new PCLPointCloud());
        convertBaseToPCLPointCloud(pointcloud.points, *pcl_pc);
        setModelPointCloud(pcl_pc);
    }
    
    TaskBase::updateHook();

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
