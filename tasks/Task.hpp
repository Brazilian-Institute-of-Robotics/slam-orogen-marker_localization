/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef MARKER_LOCALIZATION_TASK_TASK_HPP
#define MARKER_LOCALIZATION_TASK_TASK_HPP

#include "marker_localization/TaskBase.hpp"
#include <string.h>

namespace marker_localization {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Camera-position in the world-frame, based on the detected markers
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','marker_localization::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
	MarkerConfig config;

        void computeHeading(const std::vector<base::samples::RigidBodyState>& markers, const base::Affine3d& body2world);
        void computeHeading(base::Affine3d aruco_first2body, base::Affine3d aruco_second2body, base::Affine3d aruco_first2world, base::Affine3d aruco_second2world);
	int get_aruco_id(const std::string &string);
	int get_apriltag_id(const std::string &string);
        bool isMarkerKnown(int id);
        std::vector<ArucoMarker>::const_iterator getMarkerInfo(int id);
	
	base::Matrix3d get_position_cov( const base::Affine3d &body2world, const base::Affine3d &marker2body, const base::Affine3d &marker2world);
	base::Matrix3d get_orientation_cov();
        base::Matrix3d orientation_from_euler(const base::Vector3d& euler) const;

        base::Affine3d get_camera_to_body(const std::string camera_frame);
	
        base::Affine3d body2world_orientation;
    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "marker_localization::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        bool configureHook();


        bool startHook();

        void updateHook();


    };
}

#endif

