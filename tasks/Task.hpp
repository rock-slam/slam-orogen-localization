/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef LOCALIZATION_TASK_TASK_HPP
#define LOCALIZATION_TASK_TASK_HPP

#include "localization/TaskBase.hpp"

#include <envire/Orocos.hpp>
#include <envire/core/EventHandler.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/registration/gicp.h>

namespace localization {

    class Task;
    typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
    typedef typename PCLPointCloud::Ptr PCLPointCloudPtr;

    class MLSEventHandler : public envire::EventHandler
    {
    public:
        MLSEventHandler(Task* task) {this->task = task;}
    protected:
        virtual void handle( const envire::Event& event );
        
        Task* task;
    };

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
    friend class MLSEventHandler;
    protected:
        envire::TransformWithUncertainty last_body2odometry;
        envire::TransformWithUncertainty last_odometry2body;
        envire::TransformWithUncertainty last_body2world;
        envire::TransformWithUncertainty map2world;
        boost::shared_ptr<envire::Environment> env;
        boost::shared_ptr< pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> > icp;
        PCLPointCloudPtr map_pointcloud;
        unsigned max_input_sample_count;

        /**
         * Computes a pointcloud from a given MLS grid.
         * The incoming sample pointclouds will be aligned to this model pointcould.
         */
        void updateICPModelFromMap(envire::MultiLevelSurfaceGrid* mls_grid);

        /**
         * Aligns the given sample pointcloud to the current map model.
         * A current odometry sample is mandatory for this step.
         */
        void alignPointcloud(const std::vector<base::Vector3d>& sample_pointcloud, const envire::TransformWithUncertainty& body2odometry);
        void alignPointcloud(const std::vector<Eigen::Vector3d>& sample_pointcloud, const envire::TransformWithUncertainty& body2odometry);
        void alignPointcloud(const PCLPointCloudPtr sample_pointcoud, const envire::TransformWithUncertainty& body2odometry);

        /**
         * Creates a mask to sub sample a pointcloud.
         */
        void computeSampleMask(std::vector<bool>& mask, unsigned pointcloud_size, unsigned samples_count);

    public:
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
}

#endif

