// Robotics Toolbox - Common Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Common Tools contains helper and utility members 
//      useful for Robotics applications
//
// Version:
//  0.2 - Split Convert functions out of Common
//        [06.01.2023]  -   Jan T. Olsen
//  0.1 - Initial Version
//        [06.01.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robot_toolbox/tools/common.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Common Tool - Members:
// -------------------------------

    // Constants
    // -------------------------------
        // X-Axis
        AxisType AXIS_X = 
        {
            .id = AXIS_ID_X,
            .name = "x_axis", 
            .unit_vec = Eigen::Vector3d::UnitX()
        };

        // Y-Axis
        AxisType AXIS_Y = 
        {
            .id = AXIS_ID_Y,
            .name = "y_axis", 
            .unit_vec = Eigen::Vector3d::UnitY()
        };

        // Z-Axis
        AxisType AXIS_Z = 
        {
            .id = AXIS_ID_Z,
            .name = "z_axis", 
            .unit_vec = Eigen::Vector3d::UnitZ()
        };

} // End Namespace: Robotics Toolbox