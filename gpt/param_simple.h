// Robotics Toolbox - Parameter Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Parameter Tools contains helper and utility functions 
//      related to loading, reading and validating parameter-data
//      parameter-data obtained from parameter-server.
//
// Version:
//  0.2 -   Split Parameter Tools implementation
//          into multiple files utilizing template-classes 
//          and inline implementation.
//          [12.01.2024]  -   Jan T. Olsen
//  0.1 -   Initial Version
//          [19.11.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include guard:
// -------------------------------
#ifndef PARAM_TOOL_H
#define PARAM_TOOL_H

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

    // Robotics Toolbox
    #include "robot_toolbox/tools/common.h"
    #include "robot_toolbox/tools/convert.h"
    

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{

    // Parameter Tool Class
    // -------------------------------
    class ParameterTool
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:

            static bool checkMember(
                const XmlRpc::XmlRpcValue& param, 
                const std::string& member);


            static bool checkDataType(
                const XmlRpc::XmlRpcValue& param, 
                const XmlRpc::XmlRpcValue::Type& type);


            static bool checkSize(
                const XmlRpc::XmlRpcValue& param, 
                const int& size);


            static std::string getParamTypeName(
                const XmlRpc::XmlRpcValue::Type& param_type);

            
            static std::string getParamTypeName(
                const XmlRpc::XmlRpcValue& param);


        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:
            // Prefix message for class
            static const std::string CLASS_PREFIX;

    }; // Class-End: Parameter()


    class ParameterConverter
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:

            template<typename ParamType>
            static boost::optional<ParamType> convertParamXmlValue(
                const XmlRpc::XmlRpcValue& param);


            template<typename ItemType>
            static boost::optional<ItemType> convertParamType(
                const XmlRpc::XmlRpcValue& param_xml);

        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:
            // Prefix message for class
            static const std::string CLASS_PREFIX;
            
    }; // Class-End: Loader()

} // End of namespace "Toolbox"

// Include implementation-file:
// -------------------------------
#include <robot_toolbox/tools/param_converter.inl>

#endif // PARAM_TOOL_H 