// Robotics Toolbox - Parameter Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Parameter Tools contains helper and utility functions 
//      related to reading and validate parameters
//
// Version:
//  0.1 - Initial Version
//        [19.11.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robot_toolbox/tools/parameter.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Parameter Tool Class - Members:
// -------------------------------

    // Constants
    // -------------------------------
    const std::string Parameter::class_prefix = "Toolbox::Parameter::";


    // Check Parameter Member
    // -------------------------------
    bool Parameter::hasMember(
        const XmlRpc::XmlRpcValue& param, 
        const std::string member)
    {
        // Function return
        return true;
    }


    // Check and Compare Parameter Type
    // -------------------------------
    bool Parameter::compareType(
        const XmlRpc::XmlRpcValue& param, 
        const XmlRpc::XmlRpcValue::Type type)
    {
        // Function return
        return true;
    }

} // End Namespace: Robotics Toolbox