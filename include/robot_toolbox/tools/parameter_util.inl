// Robotics Toolbox - Parameter Tools - Utility
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Parameter Tools contains helper and utility functions 
//      related to reading and validate parameters
//
// Version:
//  0.2 -   Split Parameter Tools implementation
//          into multiple files utilizing inlining
//          [12.01.2024]  -   Jan T. Olsen
//  0.1 -   Initial Version
//          [19.11.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include guard:
// -------------------------------
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
#ifndef PARAM_TOOL_UTIL_INL
#define PARAM_TOOL_UTIL_INL

// Include Header-files:
// -------------------------------
    // Standard
    #include <vector>
    #include <map>
    #include <stdexcept>

    // Boost
    #include <boost/optional.hpp>
    #include <boost/core/demangle.hpp>

    // Ros
    #include <ros/ros.h>

    // Ros Messages
    #include "sensor_msgs/JointState.h"

    // Robotics Toolbox
    #include "robot_toolbox/tools/common.h"
    #include "robot_toolbox/tools/convert.h"
    #include "robot_toolbox/tools/map.h"
    #include "robot_toolbox/tools/xmlrpc_converter.inl"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
    
} // End of namespace Toolbox
