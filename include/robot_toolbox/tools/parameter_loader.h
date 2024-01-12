// Robotics Toolbox - Parameter Loader
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Parameter Loader contains utility functions for loading and validating 
//      parameter-data from given parameter-container obtained from parameter-server. 
//      Functionality is implemented as template-class
//      with member function implemented with template specialization(s).
//
// Version:
//  0.2 -   Split Parameter Tools implementation
//          into multiple files utilizing template-classes 
//          and inline implementation.
//          [12.01.2024]  -   Jan T. Olsen
//  0.1 -   Initial Version
//          [17.12.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include guard:
// -------------------------------
#ifndef PARAMETER_LOADER_H
#define PARAMETER_LOADER_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <iostream>
    #include <string>
    #include <vector>
    #include <stdexcept>

    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include "robot_toolbox/tools/common.h"
    #include "robot_toolbox/tools/convert.h"
    // #include "robot_toolbox/tools/param_helper.h"
    // #include "robot_toolbox/tools/param_converter.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
    // Parameter Loader Class
    // (Template: Primary/Default)
    // -------------------------------
    /** \brief Parameter Loader Class
    *
    * Parameter Loader contains utility functions for loading and validating 
    * parameter-data from given parameter-container obtained from parameter-server.
    * Functionality is implemented as template-class
    * with member function implemented with template specialization(s).
    */
    template<typename ItemType>
    class ParameterLoader
    {
        
    }; // Class-End: ParameterTest()
} // End of namespace "Toolbox"

#endif // PARAMETER_LOADER_H 