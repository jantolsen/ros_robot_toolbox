// Robotics Toolbox
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Collection of several useful tool libraries
//
// Version:
//  0.1 - Initial Version
//        [06.01.2023]  -   Jan T. Olsen
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
#ifndef TOOLBOX_H
#define TOOLBOX_H

// Include Header-files:
// -------------------------------

    // Class Header-File
    #include "robot_toolbox/tools/common.h"
    #include "robot_toolbox/tools/kinematics.h"
    #include "robot_toolbox/tools/map.h"
    #include "robot_toolbox/tools/math.h"
    #include "robot_toolbox/tools/visual.h"
    #include "robot_toolbox/tools/trajectory.h"
    #include "robot_toolbox/tools/parameter.h"
    // #include "robot_toolbox/tools/xmlrpc_converter.h"
    // #include "robot_toolbox/tools/param_helper.h"
    // #include "robot_toolbox/tools/param_converter.h"
    // #include "robot_toolbox/tools/param_loader.h"
    #include "robot_toolbox/tools/param.h"
#endif // TOOLBOX_H 