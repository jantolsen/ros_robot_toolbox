// Robotics Toolbox - Parameter Converter
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Parameter Converter contains utility functions for conversion 
//      of XmlRpc::XmlRpcValue to fundamental types. 
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
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
#ifndef PARAM_CONVERTER_H
#define PARAM_CONVERTER_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <iostream>
    #include <string>
    #include <vector>

    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include "robot_toolbox/tools/common.h"
    #include "robot_toolbox/tools/convert.h"
    #include "robot_toolbox/tools/param_helper.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
    // Parameter Converter Class
    // (Template: Primary/Default)
    // -------------------------------
    /** \brief Parameter Converter Class
    *
    * Parameter Converter contains utility functions for conversion 
    * of XmlRpc::XmlRpcValue to fundamental types. 
    * Functionality is implemented as template-class
    * with member function implemented with template specialization(s).
    */
    template<typename ParamType>
    struct ParameterConverter
    {
        // // Public Class members
        // // -------------------------------
        // // Accessible for everyone
        // public:

            // Convert XmlRpcValue Parameter
            // (Template: Primary/Default)
            // -------------------------------
            /** \brief Convert XmlRpcValue to fundemental type.
            *
            * Parameter is checked for data-type and explicitly cast to a fundamental type.
            * Invalid data-type will result in function return false
            * 
            * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
            * 
            * \return Function return: Successful: parameter value [ParamType] / Unsuccessful: false [bool]
            */
            static boost::optional<ParamType> convert(
                const XmlRpc::XmlRpcValue& param)
            {
                // Unsupported type!
                return boost::none;
            } // Function-End: convert()

        // // Protected Class members
        // // -------------------------------
        // // Accessible within the class which defines them, 
        // // and classes which inherits from the parent class
        // protected:

            
        // // Private Class members
        // // -------------------------------
        // // Accessible only for the class which defines them
        // private:
            // Prefix message for class
            // static const std::string CLASS_PREFIX;

    }; // Class-End: ParameterConverter()
} // End of namespace "Toolbox"

// Include implementation-file:
// -------------------------------
#include <robot_toolbox/tools/param_converter.inl>

#endif // PARAM_CONVERTER_H 