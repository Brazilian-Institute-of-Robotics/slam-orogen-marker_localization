/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace aruco_marker_conversion;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
  
  ArucoMarker marker;
  marker.id = 30;
  marker.marker2world.position = base::Vector3d::Zero();
  marker.marker2world.orientation = base::Quaterniond(Eigen::AngleAxisd(0, base::Vector3d::UnitZ()) );
  config.known_marker.push_back(marker);
  
  config.body2camera.position = base::Vector3d::Zero();
  config.body2camera.orientation = base::Quaterniond(Eigen::AngleAxisd(0, base::Vector3d::UnitZ()) );
  
  _marker_config.set(config);
  
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
  
  ArucoMarker marker;
  marker.id = 30;
  marker.marker2world.position = base::Vector3d::Zero();
  marker.marker2world.orientation = base::Quaterniond(Eigen::AngleAxisd(0, base::Vector3d::UnitZ()) );
  config.known_marker.push_back(marker);
  
  config.body2camera.position = base::Vector3d::Zero();
  config.body2camera.orientation = base::Quaterniond(Eigen::AngleAxisd(0, base::Vector3d::UnitZ()) );  
  
  _marker_config.set(config);
  
  
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    
    config = _marker_config.get();
    
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
    TaskBase::updateHook();
    
    
    std::vector<base::samples::RigidBodyState> rbs_vector;
    while( _marker_poses.read(rbs_vector) == RTT::NewData)
    {
      
      base::Affine3d body2cam = base::Affine3d::Identity();
      body2cam.translation() = config.body2camera.position;
      body2cam.linear() = config.body2camera.orientation.toRotationMatrix();
      
      for(std::vector<base::samples::RigidBodyState>::iterator it = rbs_vector.begin(); it != rbs_vector.end(); it++)
      {	
	//Extract aruco-id from frame_string
	int begin = it->sourceFrame.find("aruco_id_");
	int end = it->sourceFrame.find("_frame");
	
	if(begin != std::string::npos && end != std::string::npos && begin < end){
	  
	  //Get the string between "aruco_id_" and "_frame"
	  std::string id_string = it->sourceFrame.substr( begin + 9, end - (begin + 9));
	  int id = atoi( id_string.c_str() );  
	  
	  for(std::vector<ArucoMarker>::iterator it_marker = config.known_marker.begin(); it_marker != config.known_marker.end(); it_marker++){
	    
	    if(it_marker->id == id){
	      base::samples::RigidBodyState rbs_out;
	      base::Affine3d aruco2cam = base::Affine3d::Identity();
	      base::Affine3d aruco2world = base::Affine3d::Identity();
	      base::Affine3d cam2world = base::Affine3d::Identity();	      
	      base::Affine3d body2world = base::Affine3d::Identity();
	      
	      aruco2world.translation() = it_marker->marker2world.position;
	      aruco2world.linear() = it_marker->marker2world.orientation.toRotationMatrix();
	      
	      aruco2cam.translation() = it->position;
	      aruco2cam.linear() = it->orientation.toRotationMatrix();	      
	      
	      body2world = aruco2world * (aruco2cam.inverse() * body2cam.inverse() );
	      
	      //body2world = cam2body.inverse() * cam2world;
	      
	      rbs_out.time = it->time;
	      rbs_out.position = body2world.translation();
	      rbs_out.cov_position = base::Matrix3d::Identity() * (_position_variance_const.get() 
		+ ( it->position.norm() * _position_variance_linear.get()) );
	      
	      rbs_out.orientation = base::Quaterniond(body2world.linear());
	      rbs_out.cov_orientation = base::Matrix3d::Identity() * _orientation_variance_const.get();
	      
	      _camera_pose.write(rbs_out);
	      
	    }
	     
	  } 
	    
	  
	}	
	
      }
      
      
    }    
    
}
