/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef LOCALIZATION_TASK_TASK_HPP
#define LOCALIZATION_TASK_TASK_HPP

#include "localization/TaskBase.hpp"
#include <boost/shared_ptr.hpp>
#include <pcl/registration/gicp.h>
#include <transformer/Transformer.hpp>

#include <mtk/types/SOn.hpp>
#include <mtk/types/vect.hpp>
#include <ukfom/mtkwrap.hpp>
#include <mtk/startIdx.hpp>
#include <mtk/build_manifold.hpp>
#include <ukfom/ukf.hpp>

namespace localization
{
    typedef pcl::PointXYZ PCLPoint;
    typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
    typedef typename PCLPointCloud::Ptr PCLPointCloudPtr;

    // defines the UKF filter state
    typedef ukfom::mtkwrap< MTK::SO3<double> > RotationType;
    typedef ukfom::mtkwrap<RotationType::vect_type> TranslationType;
    MTK_BUILD_MANIFOLD(PoseState,
        ((TranslationType, position))
        ((RotationType, orientation))
    )
    typedef ukfom::mtkwrap<PoseState> WPoseState;
    typedef Eigen::Matrix<PoseState::scalar, PoseState::DOF, PoseState::DOF> PoseCovariance;

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','localization::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        Eigen::Affine3d last_odometry2body;
        Eigen::Affine3d odometry_at_last_icp;
        boost::shared_ptr< pcl::GeneralizedIterativeClosestPoint<PCLPoint, PCLPoint> > icp;
        boost::shared_ptr< ukfom::ukf<WPoseState> > ukf;
        PoseCovariance filter_process_noise;
        PCLPointCloudPtr model_cloud;
        base::Time last_icp_match;
        base::Time last_odometry_time;
        States last_state;
        States new_state;
        GICPConfiguration gicp_config;
        ICPDebugInformation icp_debug;
        bool init_odometry;

        std::string body_frame;
        std::string world_frame;

        /**
         * Updates the model point cloud
         */
        void setModelPointCloud(const PCLPointCloudPtr& pc);

        /**
        * Checks if a new icp run should be made.
        */
        bool newICPRunPossible() const;

        /**
         * Aligns the given sample pointcloud to the current map model.
         */
        void alignPointcloud(const base::Time& ts, const PCLPointCloudPtr& sample_pointcoud);

        /**
         * Runs the ICP alignment
         */
        bool performICPOptimization(const PCLPointCloudPtr& sample_pointcoud, const Eigen::Affine3d& transformation_guess, Eigen::Affine3d& result, double &icp_score);

        /**
         * Updates the current position
         * */
        void writeCurrentState(const base::Time &curTime);

        /**
         * Helper methods to convert form base pointcloud to PCL pointcloud and vise versa
         */
        template<typename PCLPoint, typename Scalar, int Options>
        void convertPCLToBasePointCloud(const pcl::PointCloud<PCLPoint>& pcl_pc, std::vector< Eigen::Matrix<Scalar,3,1,Options> >& base_pc) const;
        template<typename PCLPoint, typename Scalar, int Options>
        void convertBaseToPCLPointCloud(const std::vector< Eigen::Matrix<Scalar,3,1,Options> >& base_pc, pcl::PointCloud<PCLPoint>& pcl_pc) const;

    public:

        /**
         * Integrates the odometry information as a delta pose step
         */
        void integrateOdometry(const base::Time &ts, const transformer::Transformation &tr);

        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "localization::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
        ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };

template<typename PCLPoint, typename Scalar, int Options>
void Task::convertBaseToPCLPointCloud(const std::vector< Eigen::Matrix<Scalar,3,1,Options> >& base_pc, pcl::PointCloud< PCLPoint >& pcl_pc) const
{
    pcl_pc.clear();
    pcl_pc.resize(base_pc.size());
    for(unsigned i = 0; i < base_pc.size(); i++)
        pcl_pc[i].getVector3fMap() = base_pc[i].template cast<float>();
}

template<typename PCLPoint, typename Scalar, int Options>
void Task::convertPCLToBasePointCloud(const pcl::PointCloud< PCLPoint >& pcl_pc, std::vector< Eigen::Matrix<Scalar,3,1,Options> >& base_pc) const
{
    base_pc.clear();
    base_pc.resize(pcl_pc.size());
    for(unsigned i = 0; i < pcl_pc.size(); i++)
        base_pc[i] = pcl_pc[i].getVector3fMap().template cast<Scalar>();
}

template <typename PoseType>
PoseType
measurementUpdate(const PoseType &state)
{
    return state;
}

template <typename PoseType>
PoseType
processModel(const PoseType &state, const PoseType &pose_delta)
{
    PoseType new_state(state);
    new_state.position.boxplus(new_state.orientation * pose_delta.position);
    new_state.orientation.boxplus(MTK::SO3<double>::log(pose_delta.orientation));
    return new_state;
}

}

#endif