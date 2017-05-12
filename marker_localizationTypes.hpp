#ifndef marker_localization_TYPES_HPP
#define marker_localization_TYPES_HPP

#include <vector>
#include <base/samples/RigidBodyState.hpp>
#include <base/Eigen.hpp>

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace marker_localization {

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
    int priority; // a higher priority is favored in case multiple markers a visible

    ArucoMarker() : priority(0) {}
  };

  struct DockingStation{
    int id; //Marker ID
    base::Pose marker2dock; //Pose of the marker in world
    int priority; // a higher priority is favored in case multiple markers a visible
    
    DockingStation() : priority(0) {}
  };
  
  struct MarkerConfig{
    std::vector<ArucoMarker> known_marker; //List of all known marker-positions
    std::vector<DockingStation> docking_station; //List of all known marker-positions
    Transformation dock2world;
    std::vector<int> ids_heading;
    std::vector<Transformation> cameras2body; //Position of the camera in world-frame
    
    MarkerConfig() {}
  };
  
}

#endif

