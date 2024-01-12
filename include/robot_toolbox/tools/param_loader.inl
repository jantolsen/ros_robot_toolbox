// Robotics Toolbox - Parameter Loader
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Parameter Loader contains utility functions for loading and validating 
//      parameter-data from given parameter-container obtained from parameter-server. 
//      Functionality is implemented as template-class (utilizing struct)
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
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
#ifndef PARAM_LOADER_INL
#define PARAM_LOADER_INL

// Include Header-files:
// -------------------------------
    // Standard
    #include <iostream>
    #include <string>
    #include <vector>

    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include <robot_toolbox/toolbox.h>

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
    // Parameter Loader Class
    // (Template: Primary/Default)
    // -------------------------------
        // Constants
        // (Template Specialization: Primary/Default)
        // -------------------------------
        template<typename ParamType>
        const std::string ParameterLoader<ParamType>::CLASS_PREFIX = "Toolbox::ParameterLoader::";


    // // Parameter Loader Class
    // // -------------------------------
    // // (Template Specialization: String)
    // template<>
    // struct ParameterLoader<std::string>
    // {
    //     // Get Parameter Data
    //     // -------------------------------
    //     // (Function Overloading)
    //     // (Template Specialization: String)
    //     static boost::optional<std::string> getParamData(
    //         const XmlRpc::XmlRpcValue& param_xml,
    //         const std::string& param_name,
    //         const std::vector<std::string>& validation_set)
    //     {
    //         // TBD
            
    //         // Function return
    //         return boost::none;
    //     } // Function-End: getParamData<std::string>()
    // }; // Struct-End: ParameterLoader<std::string>()

} // End of namespace "Toolbox"
#endif // PARAM_LOADER_INL 