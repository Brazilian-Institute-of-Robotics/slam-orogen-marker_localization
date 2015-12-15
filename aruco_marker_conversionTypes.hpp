#ifndef aruco_marker_conversion_TYPES_HPP
#define aruco_marker_conversion_TYPES_HPP

#include <vector>
#include <base/samples/RigidBodyState.hpp>
#include <base/Eigen.hpp>

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace aruco_marker_conversion {

  struct Transformation
  {
      std::string camera_frame;
      base::Vector3d position; // 3D position
      base::Vector3d euler_orientation; // orientation in euler form, applied in zyx order

      Transformation() : position(base::Vector3d::Zero()), euler_orientation(base::Vector3d::Zero()) {}
  };
  
  struct ArucoMarker{
    int id; //Marker ID
    Transformation marker2world; //Pose of the marker in world
    bool position_only; //Use the marker only for the position-estimation, ignore orientation
  };

  struct DockingStation{
    int id; //Marker ID
    base::Pose marker2dock; //Pose of the marker in world
  };
  
  struct MarkerConfig{
    std::vector<ArucoMarker> known_marker; //List of all known marker-positions
    std::vector<DockingStation> docking_station; //List of all known marker-positions
    Transformation dock2world;
    std::vector<int> ids_heading;
    std::vector<Transformation> cameras2body; //Position of the camera in world-frame
    base::Vector3d marker_offset; //z-Offset to the marker-position
    
    MarkerConfig() : marker_offset(base::Vector3d::Zero()) {}
  };
  
}

#endif

