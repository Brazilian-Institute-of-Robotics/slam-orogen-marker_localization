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
  
  struct ArucoMarker{
    int id; //Marker ID
    base::Pose marker2world; //Pose of the marker in world
  };
  
  struct MarkerConfig{
    std::vector<ArucoMarker> known_marker; //List of all known marker-positions
    base::Pose body2camera; //Position of the camera in world-frame
    
  };
  
}

#endif

