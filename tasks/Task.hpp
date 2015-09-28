/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ARUCO_MARKER_CONVERSION_TASK_TASK_HPP
#define ARUCO_MARKER_CONVERSION_TASK_TASK_HPP

#include "aruco_marker_conversion/TaskBase.hpp"
#include <string.h>

namespace aruco_marker_conversion {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Camera-position in the world-frame, based on the detected markers
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','aruco_marker_multiplexer::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
	MarkerConfig config;

	int get_aruco_id(const std::string &string);
	int get_apriltag_id(const std::string &string);
	
	base::Matrix3d get_position_cov( const base::Affine3d &body2world, const base::Affine3d &marker2body, const base::Affine3d &marker2world);
	base::Matrix3d get_orientation_cov();
	
	double get_avg_yaw();
	
	std::list<double> vehicle_yaws;
    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "aruco_marker_multiplexer::Task", TaskCore::TaskState initial_state = Stopped);

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

        /** Default deconstructor of Task
         */
	~Task();

        bool configureHook();


        bool startHook();

        void updateHook();


    };
}

#endif

