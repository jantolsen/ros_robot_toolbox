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

// Include guard:
// -------------------------------
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
#ifndef PARAMETER_TOOL_H
#define PARAMETER_TOOL_H

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
    #include "robot_toolbox/tools/xmlrpc_converter.h"
    

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{

// Parameter Tool Class
// -------------------------------
class Parameter
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:

        // Check Parameter Member
        // -------------------------------
        /** \brief Check the supplied parameter for containing the member parameter
        * 
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * \param member     Member item to search for within parameter [std::string]
        * 
        * \return Function result: Supplied parameter contains specified member (true/false)
        */
        static bool checkMember(
            const XmlRpc::XmlRpcValue& param, 
            const std::string& member);


        // Check and Compare Parameter Type
        // -------------------------------
        /** \brief Check if supplied parameter equals specified parameter data-type.
        * 
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * \param type       Data-type to compare parameter against [XmlRpc::XmlRpcValue::Type]
        * 
        * \return Function result: Supplied parameter matches type of specified data-type (true/false)
        */
        static bool checkDataType(
            const XmlRpc::XmlRpcValue& param, 
            const XmlRpc::XmlRpcValue::Type& type);


        // Check and Compare Parameter Size
        // -------------------------------
        /** \brief Check if supplied parameter equals specified size.
        * 
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * \param size       Size to compare parameter against [int]
        * 
        * \return Function result: Supplied paramter matches specified size (true/false)
        */
        static bool checkSize(
            const XmlRpc::XmlRpcValue& param, 
            const int& size);


        // Check Parameter
        // -------------------------------
        // (Function Overloading)
        /** \brief Check that supplied parameter contains member and equals data-type.
        * 
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * \param member     Member item to search for within parameter [std::string]
        * \param type       Data-type to compare parameter against [XmlRpc::XmlRpcValue::Type]
        * 
        * \return Function result: Supplied parameter contains member and equals data-type (true/false)
        */
        static bool checkParameter(
            const XmlRpc::XmlRpcValue& param, 
            const std::string& member, 
            const XmlRpc::XmlRpcValue::Type& type);


        // Check Parameter
        // -------------------------------
        // (Function Overloading)
        /** \brief Check that supplied parameter contains member, equals data-type and size.
        * 
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * \param member     Member item to search for within parameter [std::string]
        * \param type       Data-type to compare parameter against [XmlRpc::XmlRpcValue::Type]
        * \param size       Size to compare parameter against [int]
        * 
        * \return Function result: Supplied parameter contains member, equals data-type and size (true/false)
        */
        static bool checkParameter(
            const XmlRpc::XmlRpcValue& param, 
            const std::string& member, 
            const XmlRpc::XmlRpcValue::Type& type,
            const int& size);

        
        // Get Name of Parameter Data-Type 
        // -------------------------------
        /** \brief Get and converts Parameter Data-Type to a readable type-name
        * 
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * 
        * \return Name of parameter-type represented as string [std::string]
        */
        static std::string getParamTypeName(
            const XmlRpc::XmlRpcValue& param);


        // Get Name of Parameter Data-Type 
        // -------------------------------
        /** \brief Get and converts Parameter Data-Type to a readable type-name
        * 
        * \param param_type Parameter data-type to be checked [XmlRpc::XmlRpcValue::Type]
        * 
        * \return Name of parameter-type represented as string  [std::string]
        */
        static std::string getParamTypeName(
            const XmlRpc::XmlRpcValue::Type& param_type);

    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

        
    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:
    
        // Prefix message for class
        static const std::string CLASS_PREFIX;


};  // End Class: Parameters
} // End Namespace: Robotics Toolbox
#endif // PARAMETER_TOOL_H