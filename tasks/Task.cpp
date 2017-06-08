/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <boost/lexical_cast.hpp>
#include <limits>

using namespace marker_localization;

bool isEqual(const ArucoMarker& marker, int id)
{
    return marker.id == id;
}

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

void Task::computeHeading(const std::vector< base::samples::RigidBodyState >& markers, const base::Affine3d& body2world)
{
    if (config.ids_heading.empty())
        return;
    
    std::vector<base::samples::RigidBodyState>::const_iterator marker1 = markers.end();
    std::vector<base::samples::RigidBodyState>::const_iterator marker2 = markers.end();
    for(std::vector<base::samples::RigidBodyState>::const_iterator it = markers.begin(); it != markers.end(); it++)
    {
        int id = get_tag_id(it->sourceFrame);
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
        std::vector<ArucoMarker>::const_iterator known_marker1 = std::find_if(config.known_marker.begin(), config.known_marker.end(), boost::bind(isEqual, _1, get_tag_id(marker1->sourceFrame)));
        std::vector<ArucoMarker>::const_iterator known_marker2 = std::find_if(config.known_marker.begin(), config.known_marker.end(), boost::bind(isEqual, _1, get_tag_id(marker2->sourceFrame)));
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

bool Task::isMarkerKnown(int id)
{
    for (std::vector<ArucoMarker>::const_iterator it = config.known_marker.begin(); it != config.known_marker.end(); it++)
    {
        if (it->id == id)
            return true;
    }
    return false;
}

std::vector< ArucoMarker >::const_iterator Task::getMarkerInfo(int id)
{
    for (std::vector<ArucoMarker>::const_iterator it = config.known_marker.begin(); it != config.known_marker.end(); it++)
    {
        if (it->id == id)
            return it;
    }
    return config.known_marker.end();
}

void Task::filterMarkers(const std::vector< base::samples::RigidBodyState >& markers, std::vector< base::samples::RigidBodyState >& filtered_markers,
                         double max_angle_in_fov, double max_rotation_of_marker, double max_distance) const
{
    for(std::vector< base::samples::RigidBodyState >::const_iterator it = markers.begin(); it != markers.end(); it++)
    {
        // check distance
        if(it->position.norm() > max_distance)
            continue;

        // check angle in camera FOV
        double angle_in_fov = acos(Eigen::Vector3d::UnitZ().dot(it->position.normalized()));
        if(angle_in_fov > max_angle_in_fov)
            continue;

        // check rotation of marker
        Eigen::Vector3d normal = it->orientation * Eigen::Vector3d::UnitZ();
        double rotation_of_marker = acos(normal.dot(-Eigen::Vector3d::UnitZ()));
        if(rotation_of_marker > max_rotation_of_marker)
            continue;

        filtered_markers.push_back(*it);
    }
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
            marker.priority = marker_d.priority;
            marker.marker2world.position = dock2world * marker_d.marker2dock.position;
            base::Orientation marker2world_ori = base::Orientation(dock2world.linear()) * marker_d.marker2dock.orientation;
            base::Vector3d euler = base::getEuler(marker2world_ori);
            marker.marker2world.euler_orientation = base::Vector3d(euler.z(), euler.y(), euler.x());
            config.known_marker.push_back(marker);
        }
        //config.docking_station.clear();
    }
    _marker_config.set(config);

    known_marker_labels.clear();
    known_marker_labels.push_back("apriltag_");
    known_marker_labels.push_back("apriltag_id_");
    known_marker_labels.push_back("arucotag_");
    known_marker_labels.push_back("aruco_");
    known_marker_labels.push_back("aruco_id_");
    
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    
    body2world_orientation.matrix() = base::NaN<double>() * Eigen::Matrix4d::Ones();

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    
    std::vector<base::samples::RigidBodyState> rbs_vector;
    std::vector<base::samples::RigidBodyState> rbs_vector_sample;
    
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

    std::vector<base::samples::RigidBodyState> filtered_markers;
    filterMarkers(rbs_vector, filtered_markers, _max_angle_in_fov.value(), _max_rotation_of_marker.value(), _max_distance.value());

    computeHeading(filtered_markers, body2world_orientation);

    int highest_priority = std::numeric_limits< int >::min();
    double marker_distance = std::numeric_limits< double >::max();
    std::vector<base::samples::RigidBodyState>::const_iterator best_marker = filtered_markers.end();

    for(std::vector<base::samples::RigidBodyState>::const_iterator it = filtered_markers.begin(); it != filtered_markers.end(); it++)
    {
        // extract id
        int id = get_tag_id( it->sourceFrame);
        if( id != -1 && isMarkerKnown(id) )
        {
            std::vector<ArucoMarker>::const_iterator marker_info = getMarkerInfo(id);
            
            if (marker_info->priority > highest_priority)
            {
                highest_priority = marker_info->priority;
                marker_distance = it->position.norm();
                best_marker = it;
            }
            else if (marker_info->priority == highest_priority && it->position.norm() < marker_distance)
            {
                marker_distance = it->position.norm();
                best_marker = it;
            }
        }
    }

    if(best_marker != filtered_markers.end())
    {
        // extract id
        int id = get_tag_id(best_marker->sourceFrame);
        
        base::Affine3d cam2body = get_camera_to_body(best_marker->targetFrame);
        std::vector<ArucoMarker>::const_iterator marker_info = getMarkerInfo(id);

        //Initialize transformations
        base::samples::RigidBodyState rbs_out, rbs_ori;
        base::Affine3d aruco2cam = base::Affine3d::Identity();
        base::Affine3d aruco2world = base::Affine3d::Identity();
        base::Affine3d aruco2body = base::Affine3d::Identity();
        base::Affine3d body2world = base::Affine3d::Identity();

        aruco2world.translation() = marker_info->marker2world.position;
        aruco2world.linear() = orientation_from_euler(marker_info->marker2world.euler_orientation);

        aruco2cam = best_marker->getTransform();

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
        rbs_out.time = best_marker->time;
        rbs_out.setTransform(body2world);
        rbs_out.orientation.normalize();
        rbs_out.sourceFrame = "body_";
        rbs_out.sourceFrame += boost::lexical_cast<std::string>(id);
        rbs_out.targetFrame = "world";

        rbs_out.cov_position =  get_position_cov( body2world, aruco2body, aruco2world);
        rbs_out.cov_orientation = get_orientation_cov();

        _pose_samples.write(rbs_out);
    }
}

int Task::get_tag_id(const std::string& label)
{
    size_t begin = std::string::npos;
    int offset = -1;
    for(std::vector<std::string>::const_iterator it = known_marker_labels.begin(); it != known_marker_labels.end(); it++)
    {
        begin = label.find(*it);
        if(begin != std::string::npos)
            offset = begin + it->size();
    }
    size_t end = label.find("_frame");
    int id = -1;
    if(offset != -1 && end != std::string::npos && offset < end)
    {
        std::string id_string = label.substr( offset, end - offset );
        try
        {
            id = boost::lexical_cast<int>( id_string.c_str() );
        }
        catch(const boost::bad_lexical_cast&)
        {
            id = -1;
        }
    }

    if(id == -1)
        std::cerr << "Couldn't find marker ID in label " << label << std::endl;

    return id;
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
