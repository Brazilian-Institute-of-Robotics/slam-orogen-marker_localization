/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace aruco_marker_conversion;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
  
  ArucoMarker marker;
  marker.id = 30;
  marker.marker2world.position = base::Vector3d::Zero();
  marker.marker2world.euler_orientation = base::Vector3d::Zero();
  config.known_marker.push_back(marker);
  
  config.camera2body.position = base::Vector3d::Zero();
  config.camera2body.euler_orientation = base::Vector3d::Zero();
  
  _marker_config.set(config);
  
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
  
  ArucoMarker marker;
  marker.id = 30;
  marker.marker2world.position = base::Vector3d::Zero();
  marker.marker2world.euler_orientation = base::Vector3d::Zero();
  config.known_marker.push_back(marker);
  
  config.camera2body.position = base::Vector3d::Zero();
  config.camera2body.euler_orientation = base::Vector3d::Zero();
  
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

	if(!config.docking_station.empty())
	{
      base::Affine3d dock2world = base::Affine3d::Identity();
      dock2world.translation() = config.dock2world.position;
      dock2world.linear() = orientation_from_euler(config.dock2world.euler_orientation);
		for(unsigned i = 0; i < config.docking_station.size(); i++)
		{
			DockingStation marker_d = config.docking_station[i];
	        ArucoMarker marker;
	        marker.id = marker_d.id;
	        marker.position_only = false;
	        marker.marker2world.position = dock2world * marker_d.marker2dock.position;
            base::Orientation marker2world_ori = base::Orientation(dock2world.linear()) * marker_d.marker2dock.orientation;
	        base::Vector3d euler = base::getEuler(marker2world_ori);
	        marker.marker2world.euler_orientation = base::Vector3d(euler.z(), euler.y(), euler.x());
	        config.known_marker.push_back(marker);
		}
	    //config.docking_station.clear();
	}
    _marker_config.set(config);

    body2world_orientation.matrix() = base::NaN<double>() * Eigen::Matrix4d::Ones();
    
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

    base::samples::RigidBodyState body2world_orientation_rbs;
    if(_body2world_orientaiton.readNewest(body2world_orientation_rbs) == RTT::NewData)
        body2world_orientation = base::Affine3d(body2world_orientation_rbs.orientation);

    if(_use_body_orientation.value() && !base::isnotnan(body2world_orientation.matrix()))
    {
        std::cout << "Waiting for body orientation in world" << std::endl;
        return;
    }
    
    std::vector<base::samples::RigidBodyState> rbs_vector;
    base::samples::RigidBodyState rbs_in;
    vehicle_yaws.clear();
    bool new_rbs  = false, new_vector = false;
    
    if( _marker_pose.readNewest(rbs_in) == RTT::NewData){
      new_rbs = true;
    }
    
    if( _marker_poses.readNewest(rbs_vector) == RTT::NewData)
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
      cam2body.linear() = orientation_from_euler(config.camera2body.euler_orientation);
      
      double min_yaw = M_PI;
      base::Orientation best_ori;
      
      for(std::vector<base::samples::RigidBodyState>::iterator it = rbs_vector.begin(); it != rbs_vector.end(); it++)
      {	
	
	int id = get_aruco_id( it->sourceFrame);
	
	if( id == -1){
	  id = get_apriltag_id( it->sourceFrame);
	}
	
	if( id != -1 ){	  
	  
	  for(std::vector<ArucoMarker>::iterator it_marker = config.known_marker.begin(); it_marker != config.known_marker.end(); it_marker++){
	    
	    if(it_marker->id == id){

	      //Initialize transformations
	      base::samples::RigidBodyState rbs_out, rbs_ori;
	      base::Affine3d aruco2cam = base::Affine3d::Identity();
	      base::Affine3d aruco2world = base::Affine3d::Identity();
	      base::Affine3d aruco2body = base::Affine3d::Identity();	      
	      base::Affine3d body2world = base::Affine3d::Identity();
	      
	      aruco2world.translation() = it_marker->marker2world.position + config.marker_offset ;
	      aruco2world.linear() = orientation_from_euler(it_marker->marker2world.euler_orientation);
	      
              aruco2cam = it->getTransform();
	      
	      //Apply transformation-chain
	      aruco2body = cam2body * aruco2cam;
              if(_use_body_orientation.value())
              {
                  body2world = base::Affine3d(body2world_orientation);
                  body2world.translation() = aruco2world.translation() - body2world_orientation * aruco2body.translation();
              }
              else
                  body2world = aruco2world * aruco2body.inverse();
	      
	      //Construct rigidBodyState
	      rbs_out.time = it->time;
              rbs_out.setTransform(body2world);
	      rbs_out.orientation.normalize();
	      
	      rbs_out.cov_position =  get_position_cov( body2world, aruco2body, aruco2world);	      	      
	      rbs_out.cov_orientation = get_orientation_cov();
	      
	      base::samples::RigidBodyState rbs_a2b;
	      base::samples::RigidBodyState rbs_b2w;
	      
	      rbs_a2b.setTransform(aruco2body);
	      rbs_b2w.setTransform(body2world);
	      
	      _aruco2body.write(rbs_a2b);
	      _body2world.write(rbs_b2w);
	      
	      _pose_output.write(rbs_out);
	      
		if(!it_marker->position_only){
		  
		  if(!_best_orientation_only){
		    vehicle_yaws.push_back( base::getYaw(rbs_out.orientation) );
		  }else{
		        
		    base::Vector3d negativeZ( 0.0, 0.0,-1.0);
		    base::Vector3d marker_normal = rbs_a2b.orientation * negativeZ;
		    double marker_view_angle = std::fabs( std::atan2( marker_normal.y(), marker_normal.x() ) );
		    std::cout << "Marker view angle: " << marker_view_angle << std::endl; 

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
	rbs.cov_orientation = get_orientation_cov( );
	_orientation_output.write(rbs);
      }else if(vehicle_yaws.size() > _minimum_orientation_markers.get()){
	base::samples::RigidBodyState rbs;
	rbs.time = rbs_vector.begin()->time;
	double yaw = get_avg_yaw();
	rbs.orientation = base::Orientation( Eigen::AngleAxisd(yaw, base::Vector3d::UnitZ()) );
	rbs.cov_orientation = get_orientation_cov( );
	_orientation_output.write(rbs);
      }
    
}

int Task::get_aruco_id(const std::string &string){

  //Extract aruco-id from frame_string
  int begin = string.find("aruco_id_");
  int end = string.find("_frame");
	
  if(begin != std::string::npos && end != std::string::npos && begin < end){
	  
	  //Get the string between "aruco_id_" and "_frame"
	  std::string id_string = string.substr( begin + 9, end - (begin + 9));
	  return atoi( id_string.c_str() );    
  }
  
  return -1;
}
  
int Task::get_apriltag_id(const std::string &string){

  //Extract aruco-id from frame_string
  int begin = string.find("apriltag_id_");
  int end = string.find("_frame");
	
  if(begin != std::string::npos && end != std::string::npos && begin < end){
	  
	  //Get the string between "aruco_id_" and "_frame"
	  std::string id_string = string.substr( begin + 12, end - (begin + 12));
	  return atoi( id_string.c_str() );    
  }
  
  return -1;  

}


base::Matrix3d Task::get_position_cov( const base::Affine3d &body2world, const base::Affine3d &marker2body, const base::Affine3d &marker2world)
{

/*  
base::Matrix3d cov = base::Matrix3d::Identity();
cov(0,0) = _position_variance_range.get();
cov(1,1) = std::sin(_position_variance_angle.get()) * marker2body.translation().norm();

return (body2world.linear() * cov * body2world.linear().transpose() )
    + (base::Matrix3d::Identity() * _position_variance_const.get()) ;  
*/

  base::Vector3d marker_pos = marker2world.translation();
  base::Vector3d body_pos = body2world.translation();
  base::Vector3d rotation_vector = body_pos - marker_pos;

   //Calculate covariance rotation
    base::Vector3d a = base::Vector3d::UnitX();
    base::Vector3d b = rotation_vector.normalized();
    base::Vector3d v = a.cross(b);
    double s = v.norm();
    double c = a.dot(b);
    
    base::Matrix3d screw_v = base::Matrix3d::Zero();
    screw_v(0,1) = - v(2);
    screw_v(0,2) = v(1);
    screw_v(1,0) = v(2);
    screw_v(1,2) = -v(0);
    screw_v(2,0) = -v(1);
    screw_v(2,1) = v(0);    
    
    base::Matrix3d rot = base::Matrix3d::Identity() + screw_v
      + ( (screw_v * screw_v) * ((1-c)/(s*s))  ); 

    base::Matrix3d cov = base::Matrix3d::Identity();
    cov(0,0) = _position_variance_range.get();
    cov(1,1) = std::sin(_position_variance_angle.get()) * marker2body.translation().norm(); 
    cov(2,2) = std::sin(_position_variance_angle.get()) * marker2body.translation().norm();   
      
    return (body2world.linear() * cov * body2world.linear().transpose() )
	+ (base::Matrix3d::Identity() * _position_variance_const.get()) ;        
      
}


base::Matrix3d Task::get_orientation_cov()
{

  return base::Matrix3d::Identity() * _orientation_variance_const.get();
}

base::Matrix3d Task::orientation_from_euler(const base::Vector3d& euler) const
{
    return (Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

double Task::get_avg_yaw(){
  
  if(vehicle_yaws.size() < 1)
    return NAN;
  
  double mean = vehicle_yaws.front();
  double avg_mean_diff = 0.0;
  
  for(int i = 0; i < 10; i++){
    
    avg_mean_diff = 0.0;
  
    for(std::list<double>::iterator it = vehicle_yaws.begin(); it != vehicle_yaws.end(); it++){
    
      double mean_diff = *it - mean;
      
      while(mean_diff < -M_PI)
	mean_diff += 2.0 * M_PI;
      
      while(mean_diff > M_PI)
	mean_diff -= 2.0 * M_PI;

      avg_mean_diff += (1.0/vehicle_yaws.size()) * mean_diff;
      
    }      
    
    mean += avg_mean_diff;
    
    while(mean < -M_PI)
      mean += 2.0 * M_PI;
      
    while(mean > M_PI)
      mean -= 2.0 * M_PI;    
    
  }
  
  return mean;
  
}
