/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <boost/lexical_cast.hpp>

using namespace marker_localization;

bool isEqual(const ArucoMarker& marker, int id)
{
    return marker.id == id;
}

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{  
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
}

Task::~Task()
{
}

void Task::computeHeading(const std::vector< base::samples::RigidBodyState >& markers, const base::Affine3d& body2world)
{
    std::vector<base::samples::RigidBodyState>::const_iterator marker1 = markers.end();
    std::vector<base::samples::RigidBodyState>::const_iterator marker2 = markers.end();
    for(std::vector<base::samples::RigidBodyState>::const_iterator it = markers.begin(); it != markers.end(); it++)
    {
        int id = get_apriltag_id(it->sourceFrame);
        if(std::find(config.ids_heading.begin(), config.ids_heading.end(), id) != config.ids_heading.end())
        {

            if(marker1 == markers.end())
            {
                marker1 = it;
            }
            else if(marker2 == markers.end())
            {
                marker2 = it;
            }

            if(marker1 != markers.end() && marker2 != markers.end())
                break;
        }
    }

    if(marker1 != markers.end() && marker2 != markers.end())
    {
        base::Orientation rollpitch2world = base::removeYaw(base::Orientation(body2world.linear()));
        std::vector<ArucoMarker>::const_iterator known_marker1 = std::find_if(config.known_marker.begin(), config.known_marker.end(), boost::bind(isEqual, _1, get_apriltag_id(marker1->sourceFrame)));
        std::vector<ArucoMarker>::const_iterator known_marker2 = std::find_if(config.known_marker.begin(), config.known_marker.end(), boost::bind(isEqual, _1, get_apriltag_id(marker2->sourceFrame)));
        if(known_marker1 != config.known_marker.end() && known_marker2 != config.known_marker.end())
        {
            base::Affine3d aruco1InBody = rollpitch2world * (get_camera_to_body(marker1->targetFrame) * marker1->getTransform());
            base::Affine3d aruco2InBody = rollpitch2world * (get_camera_to_body(marker2->targetFrame) * marker2->getTransform());
            base::Affine3d aruco1InWorld = base::Affine3d::Identity();
            aruco1InWorld.translation() = known_marker1->marker2world.position;
            aruco1InWorld.linear() = orientation_from_euler(known_marker1->marker2world.euler_orientation);
            base::Affine3d aruco2InWorld = base::Affine3d::Identity();
            aruco2InWorld.translation() = known_marker2->marker2world.position;
            aruco2InWorld.linear() = orientation_from_euler(known_marker2->marker2world.euler_orientation);
            computeHeading(aruco1InBody, aruco2InBody, aruco1InWorld, aruco2InWorld);
        }
        else
            std::cerr << "Couldn't find markers selected in ids_heading!" << std::endl;
    }
}

void Task::computeHeading(base::Affine3d aruco_first2body, base::Affine3d aruco_second2body, base::Affine3d aruco_first2world, base::Affine3d aruco_second2world)
{
    Eigen::Vector3d first_norm = aruco_first2body.translation().normalized();
    Eigen::Vector3d second_norm = aruco_second2body.translation().normalized();
    base::Angle angle = base::Angle::fromRad(atan2(first_norm.y(),first_norm.x()) - atan2(second_norm.y(),second_norm.x()));

    Eigen::Vector2d left_p;
    Eigen::Vector2d right_p;
    Eigen::Vector2d left_p_world;
    Eigen::Vector2d right_p_world;
    if(angle.rad == 0.0)
    {
        std::cerr << "cannot compute angle" << std::endl;
        return;
    }
    else if(angle.rad > 0.0)
    {
        left_p = aruco_first2body.translation().block(0,0,2,1);
        right_p = aruco_second2body.translation().block(0,0,2,1);
        left_p_world = aruco_first2world.translation().block(0,0,2,1);
        right_p_world = aruco_second2world.translation().block(0,0,2,1);
    }
    else
    {
        left_p = aruco_second2body.translation().block(0,0,2,1);
        right_p = aruco_first2body.translation().block(0,0,2,1);
        left_p_world = aruco_second2world.translation().block(0,0,2,1);
        right_p_world = aruco_first2world.translation().block(0,0,2,1);
    }

    Eigen::Vector2d baseline_normal = Eigen::Rotation2Dd(M_PI_2) * (right_p - left_p);
    Eigen::Vector2d baseline_normal_w = Eigen::Rotation2Dd(M_PI_2) * (right_p_world - left_p_world);

    base::Angle bodyInBaseline = base::Angle::fromRad(-atan2(baseline_normal.normalized().y(),baseline_normal.normalized().x()));
    base::Angle baselineInWorld = base::Angle::fromRad(atan2(baseline_normal_w.normalized().y(),baseline_normal_w.normalized().x()));
    base::Angle bodyInWorld = baselineInWorld + bodyInBaseline;

    base::Affine3d orientation2world(Eigen::AngleAxisd(bodyInWorld.getRad(), Eigen::Vector3d::UnitZ()));
    base::samples::RigidBodyState orientation2world_rbs;
    orientation2world_rbs.initSane();
    orientation2world_rbs.time = base::Time::now();
    orientation2world_rbs.setTransform(orientation2world);

    _orientation2world.write(orientation2world_rbs);
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
    
    std::vector<base::samples::RigidBodyState> rbs_vector;
    std::vector<base::samples::RigidBodyState> rbs_vector_sample;
    vehicle_yaws.clear();
    
    while( _marker_poses.read(rbs_vector_sample) == RTT::NewData)
    {
        for(unsigned i = 0; i < rbs_vector_sample.size(); i++)
        {
            rbs_vector.push_back(rbs_vector_sample[i]);
        }
    }
    
    base::samples::RigidBodyState rbs_sample;
    while( _marker_pose.read(rbs_sample) == RTT::NewData)
    {
        rbs_vector.push_back(rbs_sample);
    }
    
    if(rbs_vector.empty())
        return;

    base::samples::RigidBodyState body2world_orientation_rbs;
    if(_body2world_orientaiton.readNewest(body2world_orientation_rbs) == RTT::NewData)
        body2world_orientation = base::Affine3d(body2world_orientation_rbs.orientation);

    if(_use_body_orientation.value() && !base::isnotnan(body2world_orientation.matrix()))
    {
        std::cout << "Waiting for body orientation in world" << std::endl;
        return;
    }

    computeHeading(rbs_vector, body2world_orientation);

      double min_yaw = M_PI;
      base::Orientation best_ori;
      
      for(std::vector<base::samples::RigidBodyState>::iterator it = rbs_vector.begin(); it != rbs_vector.end(); it++)
      {	
	
	int id = get_aruco_id( it->sourceFrame);
	
	if( id == -1){
	  id = get_apriltag_id( it->sourceFrame);
	}
	
	if( id != -1 )
        {
          base::Affine3d cam2body = get_camera_to_body(it->targetFrame);


          if(id == 300)
          {
              base::Affine3d aruco2body = cam2body * it->getTransform();
              base::Orientation aruco_orientation = base::Orientation(aruco2body.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
              base::Angle aruco_yaw = base::Angle::fromRad(base::getYaw(aruco_orientation));
              base::Angle body_yaw = base::Angle::fromRad(base::getYaw(base::Orientation(body2world_orientation.linear())));

              _orientation_offset.write((body_yaw - aruco_yaw) - base::Angle::fromDeg(90.0));
          }

	  
	  for(std::vector<ArucoMarker>::iterator it_marker = config.known_marker.begin(); it_marker != config.known_marker.end(); it_marker++){
	    
	    if(it_marker->id == id){

	      //Initialize transformations
	      base::samples::RigidBodyState rbs_out, rbs_ori;
	      base::Affine3d aruco2cam = base::Affine3d::Identity();
	      base::Affine3d aruco2world = base::Affine3d::Identity();
	      base::Affine3d aruco2body = base::Affine3d::Identity();
	      base::Affine3d body2world = base::Affine3d::Identity();
	      
	      aruco2world.translation() = it_marker->marker2world.position;
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
              rbs_out.sourceFrame = "body_";
              rbs_out.sourceFrame += boost::lexical_cast<std::string>(id);
              rbs_out.targetFrame = "world";
	      
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
      }else if((int)vehicle_yaws.size() > _minimum_orientation_markers.get()){
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
  size_t begin = string.find("aruco_id_");
  size_t end = string.find("_frame");
	
  if(begin != std::string::npos && end != std::string::npos && begin < end){
	  
	  //Get the string between "aruco_id_" and "_frame"
	  std::string id_string = string.substr( begin + 9, end - (begin + 9));
	  return atoi( id_string.c_str() );    
  }
  
  return -1;
}
  
int Task::get_apriltag_id(const std::string &string){

  //Extract aruco-id from frame_string
  size_t begin = string.find("apriltag_id_");
  size_t end = string.find("_frame");
	
  if(begin != std::string::npos && end != std::string::npos && begin < end){
	  
	  //Get the string between "aruco_id_" and "_frame"
	  std::string id_string = string.substr( begin + 12, end - (begin + 12));
	  return atoi( id_string.c_str() );    
  }
  
  return -1;  

}


base::Matrix3d Task::get_position_cov( const base::Affine3d &body2world, const base::Affine3d &marker2body, const base::Affine3d &marker2world)
{
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

base::Affine3d Task::get_camera_to_body(const std::string camera_frame)
{
    base::Affine3d cam2body = base::Affine3d::Identity();
    for(unsigned i = 0; i < config.cameras2body.size(); i++)
    {
        if(config.cameras2body[i].camera_frame == camera_frame)
        {
            cam2body.translation() = config.cameras2body[i].position;
            cam2body.linear() = orientation_from_euler(config.cameras2body[i].euler_orientation);
        }
    }
    return cam2body;
}
