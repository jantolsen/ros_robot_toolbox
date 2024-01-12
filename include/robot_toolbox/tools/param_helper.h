// Robotics Toolbox - Parameter Helper
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Parameter Helper contains utility functions for reading and validating 
//      parameter-data obtained from parameter-server.
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
#ifndef PARAM_HELPER_H
#define PARAM_HELPER_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <iostream>
    #include <string>
    #include <vector>

    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include <robot_toolbox/tools/common.h>
    #include <robot_toolbox/tools/convert.h>

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
    // Parameter Helper Class
    // -------------------------------
    /** \brief Parameter Helper Class
    *
    * Parameter Helper contains utility functions for reading and validating 
    * parameter-data obtained from parameter-server.
    */
    class ParameterHelper
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

    }; // Class-End: ParamHelper()
} // End of namespace "Toolbox"

#endif // PARAM_HELPER_H 