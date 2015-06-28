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
  
  config.camera2body.position = base::Vector3d::Zero();
  config.camera2body.orientation = base::Quaterniond(Eigen::AngleAxisd(0, base::Vector3d::UnitZ()) );
  
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
  
  config.camera2body.position = base::Vector3d::Zero();
  config.camera2body.orientation = base::Quaterniond(Eigen::AngleAxisd(0, base::Vector3d::UnitZ()) );  
  
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
    base::samples::RigidBodyState rbs_in;
    bool new_rbs  = false, new_vector = false;
    
    while( _marker_pose.read(rbs_in) == RTT::NewData){
      new_rbs = true;
    }
    
    while( _marker_poses.read(rbs_vector) == RTT::NewData)
    {
      new_vector = true;
    }
    
    if( new_vector){
      
      if(new_rbs)
	rbs_vector.push_back(rbs_in);
      
    }else{
      
      if(new_rbs){
	rbs_vector.clear();
	rbs_vector.push_back(rbs_in);
      }else{
	return;
      }
      
    }
      
      base::Affine3d cam2body = base::Affine3d::Identity();
      cam2body.translation() = config.camera2body.position;
      cam2body.linear() = config.camera2body.orientation.toRotationMatrix();
      
      double min_yaw = M_PI;
      base::Orientation best_ori;
      
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

	      //Initialize transformations
	      base::samples::RigidBodyState rbs_out, rbs_ori;
	      base::Affine3d aruco2cam = base::Affine3d::Identity();
	      base::Affine3d aruco2world = base::Affine3d::Identity();
	      base::Affine3d aruco2body = base::Affine3d::Identity();	      
	      base::Affine3d body2world = base::Affine3d::Identity();
	      
	      aruco2world.translation() = it_marker->marker2world.position;
	      aruco2world.linear() = it_marker->marker2world.orientation.toRotationMatrix();
	      
	      aruco2cam.translation() = it->position;
	      aruco2cam.linear() = it->orientation.toRotationMatrix();	      
	      
	      
	      aruco2body = cam2body * aruco2cam;
	      body2world = aruco2world * aruco2body.inverse();
	      
	      rbs_out.time = it->time;
	      rbs_out.position = body2world.translation();
	      rbs_out.cov_position = base::Matrix3d::Identity() * (_position_variance_const.get() 
		+ ( it->position.norm() * _position_variance_linear.get()) );
	      
	      rbs_out.orientation = base::Quaterniond(body2world.linear());
	      rbs_out.orientation.normalize();
	      rbs_out.cov_orientation = base::Matrix3d::Identity() * _orientation_variance_const.get();
	      
	      base::samples::RigidBodyState rbs_a2b;
	      base::samples::RigidBodyState rbs_b2w;
	      
	      rbs_a2b.setTransform(aruco2body);
	      rbs_b2w.setTransform(body2world);
	      
	      _aruco2body.write(rbs_a2b);
	      _body2world.write(rbs_b2w);
	      
	      _pose_output.write(rbs_out);
	      
		if(!it_marker->position_only){
		  
		  if(!_best_orientation_only){
		    rbs_ori = rbs_out;
		    rbs_ori.position = base::Vector3d::Constant(NAN);
		    _orientation_output.write( rbs_ori);
		  }else{
		        
		    base::Vector3d negativeZ( 0.0, 0.0,-1.0);
		    base::Vector3d marker_normal = aruco2body * negativeZ;
		    double marker_view_angle = std::atan2( marker_normal.y(), marker_normal.x() );
		    
		    if( marker_view_angle < min_yaw && marker_view_angle < _marker_orientation_threshold.get() ){
		      best_ori = rbs_out.orientation;
		      min_yaw = marker_view_angle;
		    }
		    
		  }
		  
		}
	      
	    }//End if
	     
	  }//End known_marker-loop 
	    
	  
	}//End check marker-id	
	
      }//End detected marker loop
      
      
      if(_best_orientation_only.get() && min_yaw < M_PI){
	base::samples::RigidBodyState rbs;
	rbs.time = rbs_vector.begin()->time;
	rbs.orientation = best_ori;
	rbs.cov_orientation = base::Matrix3d::Identity() * _orientation_variance_const.get();
	_orientation_output.write(rbs);
      }  
    
}
