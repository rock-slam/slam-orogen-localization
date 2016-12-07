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
    base::Vector6d process_noise;
    process_noise << 0.0025, 0.0025, 0.0025, 0.0016, 0.0016, 0.0016;
    _process_noise_diagonal.set(process_noise);
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
    base::Vector6d process_noise;
    process_noise << 0.0025, 0.0025, 0.0025, 0.0016, 0.0016, 0.0016;
    _process_noise_diagonal.set(process_noise);
}

Task::~Task()
{
}

void Task::setModelPointCloud(const PCLPointCloudPtr& pc)
{
    model_cloud = pc;
    icp->setInputTarget(model_cloud);
}

void Task::alignPointcloud(const base::Time& ts, const PCLPointCloudPtr& sample_pointcoud)
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

    odometry_at_last_icp = last_odometry2body;
    last_icp_match = ts;

    Eigen::Affine3d transformation_guess;
    transformation_guess = ukf->mu().orientation;
    transformation_guess.pretranslate( ukf->mu().position );

    base::TimeMark icp_run("ICP alignment");
    Eigen::Affine3d icp_result;
    double icp_score;
    LOG_INFO_S << "Run ICP optimization";
    if(performICPOptimization(input_pointcloud, transformation_guess, icp_result, icp_score))
    {
        new_state = RUNNING;
        LOG_INFO_S << "ICP alignment successful. ICP score: " << icp_score;
        LOG_INFO_S << "Got new ICP match " << icp_result.translation().transpose();
        icp_debug.successful_alignments++;

        WPoseState icp_result_;
        icp_result_.position = TranslationType(icp_result.translation());
        icp_result_.orientation = RotationType(icp_result.linear());
        PoseCovariance icp_cov = 0.01 * PoseCovariance::Identity();
        ukf->update(icp_result_, boost::bind(measurementUpdate<WPoseState>, _1), icp_cov);
    }
    else
    {
        LOG_WARN_S << "ICP alignment failed, perhaps a model update is necessary. ICP score: " << icp_score;
        new_state = ICP_ALIGNMENT_FAILED;
        icp_debug.failed_alignments++;
    }

    LOG_INFO_S << icp_run;

    //write out current pose sample
    writeCurrentState(ts);

    icp_debug.time = ts;
    icp_debug.last_fitness_score = icp_score;
    icp_debug.icp_alignment_time = (double)icp_run.cycles() / (double)CLOCKS_PER_SEC;
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
            Eigen::Affine3d current_pose;
            current_pose = ukf->mu().orientation;
            current_pose.pretranslate( ukf->mu().position );
            pcl::transformPointCloud(*input_pointcloud, transformed_measurement, current_pose);
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

    icp_score = icp->getFitnessScore(gicp_config.max_correspondence_distance);
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

void Task::writeCurrentState(const base::Time &curTime)
{
    //comput delta between odometry position at the time of the last ICP match and the current odometry position
    base::samples::RigidBodyState sample_out;
    sample_out.invalidate();
    sample_out.time = curTime;
    sample_out.position = ukf->mu().position;
    sample_out.orientation = ukf->mu().orientation;
    sample_out.cov_position = ukf->sigma().block(0,0,3,3);
    sample_out.cov_orientation = ukf->sigma().block(3,3,3,3);
    sample_out.sourceFrame = body_frame;
    sample_out.targetFrame = world_frame;
    _pose_samples.write(sample_out);
}

bool Task::newICPRunPossible(const base::Time& current_time) const
{
    if(!odometry_at_last_icp.matrix().allFinite() ||
      (last_odometry2body * odometry_at_last_icp).translation().norm() > gicp_config.icp_match_interval ||
      (current_time - last_icp_match).toSeconds() > gicp_config.icp_match_interval_time)
        return true;
    return false;
}

void Task::integrateOdometry(const base::Time& ts, const transformer::Transformation& tr)
{
    Eigen::Affine3d body2odometry;
    if (!tr.get(ts, body2odometry))
    {
        RTT::log(RTT::Error) << "skip, have no body2odometry transformation sample!" << RTT::endlog();
        new_state = TaskBase::MISSING_TRANSFORMATION;
        return;
    }

    if(init_odometry)
    {
        init_odometry = false;
        last_odometry2body = body2odometry.inverse();
        return;
    }

    Eigen::Affine3d odometry_delta = last_odometry2body * body2odometry;
    double delta_t = (ts - last_odometry_time).toSeconds();
    last_odometry2body = body2odometry.inverse();
    last_odometry_time = ts;

    // UKF prediction step
    WPoseState odometry_delta_;
    odometry_delta_.position = TranslationType(odometry_delta.translation());
    odometry_delta_.orientation = RotationType(odometry_delta.linear());
    ukf->predict(boost::bind(processModel<WPoseState>, _1, odometry_delta_),  ukfom::ukf<WPoseState>::cov(delta_t * filter_process_noise));
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

    // set icp config
    gicp_config = _gicp_configuration.get();
    icp.reset(new pcl::GeneralizedIterativeClosestPoint<PCLPoint, PCLPoint>());
    icp->setMaxCorrespondenceDistance(gicp_config.max_correspondence_distance);
    icp->setMaximumIterations(gicp_config.maximum_iterations);
    icp->setTransformationEpsilon(gicp_config.transformation_epsilon);
    icp->setCorrespondenceRandomness(gicp_config.correspondence_randomness);
    icp->setMaximumOptimizerIterations(gicp_config.maximum_optimizer_iterations);
    icp->setRotationEpsilon(gicp_config.rotation_epsilon);

    // initialize filter
    double pos_var = pow(gicp_config.max_correspondence_distance, 2.);
    double rot_var = 2.5; // sigma of 90 degree
    PoseCovariance initial_cov = PoseCovariance::Zero();
    initial_cov.block(0,0,3,3) = Eigen::Vector3d(pos_var, pos_var, pos_var).asDiagonal();
    initial_cov.block(3,3,3,3) = Eigen::Vector3d(rot_var, rot_var, rot_var).asDiagonal();
    WPoseState initial_state;
    initial_state.position = RotationType::vect_type(_start_pose.value().position);
    initial_state.orientation = MTK::SO3<double>(_start_pose.value().orientation);
    ukf.reset(new ukfom::ukf<WPoseState>(initial_state, initial_cov));
    filter_process_noise = _process_noise_diagonal.value().asDiagonal();

    // set inital transformations
    last_odometry2body = Eigen::Affine3d::Identity();
    odometry_at_last_icp = Eigen::Affine3d(base::unknown<double>() * Eigen::Matrix4d::Ones());
    init_odometry = true;
    
    last_icp_match.microseconds = 0;
    
    // reset map point cloud
    model_cloud.reset();
    
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

    // apply external pose update
    base::samples::RigidBodyState pose_update;
    while(_pose_update.read(pose_update, false) == RTT::NewData)
    {
        WPoseState pose_update_;
        pose_update_.position = TranslationType(pose_update.position);
        pose_update_.orientation = MTK::SO3<double>(pose_update.orientation);
        PoseCovariance pose_update_cov = PoseCovariance::Zero();
        pose_update_cov.block(0,0,3,3) = pose_update.cov_position;
        pose_update_cov.block(3,3,3,3) = pose_update.cov_orientation;
        ukf->update(pose_update_, boost::bind(measurementUpdate<WPoseState>, _1), pose_update_cov);
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
