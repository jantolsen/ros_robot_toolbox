// Robotics Toolbox - Parameter Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Parameter Tools contains utility functions for
//      loading, convertion and validation of 
//      parameter-data from parameter-container
//      obtained from the parameter-server. 
//
// Version:
//  0.2 -   Overual of Parameter Tools implementation.
//          Introduced inline implementation.
//          [12.01.2024]  -   Jan T. Olsen
//  0.1 -   Initial Version
//          [19.11.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include guard:
// -------------------------------
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

    // Robotics Toolbox
    #include "robot_toolbox/tools/common.h"
    #include "robot_toolbox/tools/convert.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{

    // Parameter Tool Class
    // -------------------------------
    /** \brief Parameter Tool
    *
    * Toolbox for Robotics
    * Parameter Tools contains utility functions for
    * loading, convertion and validation of 
    * parameter-data from parameter-container
    * obtained from the parameter-server.
    */
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


            // Get Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Get Parameter Data.
            *
            * Get parameter-data from given parameter-container. 
            * The parameter-name is used to as a key to search through the parameter-container.
            * Succesful search and type conversion the function returns with parameter-data.
            * If parameter-name is not found in the parameter-container or if parameter is configured incorrectly, 
            * an error message is given and a runtime expection is thrown.
            * 
            * \param param_xml          Parameter-container to search through [const XmlRpc::XmlRpcValue&]
            * \param param_name         Parameter-name to search for [const std::string&]
            * 
            * \return Function return: parameter data [typename ParamData]
            */
            template<typename ParamData>
            static ParamData getParamData(
                const XmlRpc::XmlRpcValue& param_xml,
                const std::string& param_name);


            // Load Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Load Parameter Data.
            *
            * Loads parameter-data from given parameter-container and checks it against validation set. 
            * The parameter-name is used to as a key to search through the parameter-container.
            * Succesful search, type conversion and validation check the function returns with parameter-data.
            * If parameter-name is not found in the parameter-container or if parameter is configured incorrectly, 
            * an error message is given and a runtime expection is thrown.
            *
            * \param param_xml      Parameter-container to search through [const XmlRpc::XmlRpcValue&]
            * \param param_name     Parameter-name to search for [const std::string&]
            * \param data_set       Data-set vector used to validate the parameter-data [const std::vector<typename ParamData>&]
            * 
            * \return Function return: parameter data [typename ParamData]
            */
            template<typename ParamData>
            static ParamData getParamData(
                const XmlRpc::XmlRpcValue& param_xml,
                const std::string& param_name,
                const std::vector<ParamData>& data_set);


            // Load Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Load Parameter Data.
            *
            * Loads parameter-data from given parameter-container
            * and uses it as a key to find the corresponding paired value in the given map.
            * The parameter-name is used to as a key to search through the parameter-container.
            * Succesful search, type conversion and the paired parameter-data is found within the map
            * the function returns with related paired value of parameter-data.
            * If parameter-name is not found in the parameter-container or if parameter is configured incorrectly, 
            * an error message is given and a runtime expection is thrown.
            * 
            * \param param_xml      Parameter-container to search through [const XmlRpc::XmlRpcValue&]
            * \param param_name     Parameter-name to search for [const std::string&]
            * \param data_map       Data-map to search for the related paired value of parameter-data [std::map<MapKey, MapValue>]
            * 
            * \return Function return: parameter data [typename ParamData]
            */
            template<typename ParamData, typename MapKey, typename MapValue>
            static ParamData getParamData(
                const XmlRpc::XmlRpcValue& param_xml,
                const std::string& param_name,
                const std::map<MapKey, MapValue>& data_map);


            // Convert Parameter Type
            // -------------------------------
            /** \brief Convert parameter to fundemental type.
            *
            * Generic parameter convert/cast function.
            * Given parameter (XmlRpcValue) is checked for data-type and explicitly cast to a fundamental type.
            * Successful convertion returns with casted item of fundemental type
            * Invalid data-type will result in error message and function return false
            * 
            * \param param_xml  Parameter to be converted [XmlRpc::XmlRpcValue]
            * 
            * \return Function return: Successful: parameter data [ParamType] / Unsuccessful: false [bool]
            */
            template<typename ParamType>
            static boost::optional<ParamType> convertParamType(
                const XmlRpc::XmlRpcValue& param);

        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

            // Load Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Load Parameter Data.
            *
            * Loads parameter-data from given parameter-container. 
            * The parameter-name is used to as a key to search through the parameter-container.
            * Succesful search and type conversion the function returns with parameter-data.
            * If parameter-name is not found in the parameter-container or if parameter is configured incorrectly, 
            * an error message is given and function returns false.
            * 
            * \param param_xml          Parameter-container to search through [const XmlRpc::XmlRpcValue&]
            * \param param_name         Parameter-name to search for [const std::string&]
            * 
            * \return Function return: parameter data [typename ItemType]
            */
            template<typename ParamData>
            static boost::optional<ParamData> loadParamData(
                const XmlRpc::XmlRpcValue& param_xml,
                const std::string& param_name);


            // Load Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Load Parameter Data.
            *
            * Loads parameter-data from given parameter-container and checks it against validation set. 
            * The parameter-name is used to as a key to search through the parameter-container.
            * Succesful search, type conversion and validation check the function returns with parameter-data.
            * If parameter-name is not found in the parameter-container or if parameter is configured incorrectly, 
            * an error message is given and function returns false.
            *
            * \param param_xml      Parameter-container to search through [const XmlRpc::XmlRpcValue&]
            * \param param_name     Parameter-name to search for [const std::string&]
            * \param data_set       Data-set vector used to validate the parameter-data [const std::vector<typename ParamData>&]
            * 
            * \return Function return: parameter data [typename ItemType]
            */
            template<typename ParamData>
            static boost::optional<ParamData> loadParamData(
                const XmlRpc::XmlRpcValue& param_xml,
                const std::string& param_name,
                const std::vector<ParamData>& data_set);


            // Load Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Load Parameter Data.
            *
            * Loads parameter-data from given parameter-container
            * and uses it as a key to find the corresponding paired value in the given map.
            * The parameter-name is used to as a key to search through the parameter-container.
            * Succesful search, type conversion and the paired parameter-data is found within the map
            * the function returns with related paired value of parameter-data.
            * If parameter-name is not found in the parameter-container or if parameter is configured incorrectly, 
            * an error message is given and function returns false.
            * 
            * \param param_xml      Parameter-container to search through [const XmlRpc::XmlRpcValue&]
            * \param param_name     Parameter-name to search for [const std::string&]
            * \param data_map       Data-map to search for the related paired value of parameter-data [std::map<MapKey, MapValue>]
            * 
            * \return Function return: parameter data [typename ItemType]
            */
            template<typename ParamData, typename MapKey, typename MapValue>
            static boost::optional<ParamData> loadParamData(
                const XmlRpc::XmlRpcValue& param_xml,
                const std::string& param_name,
                const std::map<MapKey, MapValue>& data_map);

        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:
            // Prefix message for class
            static const std::string CLASS_PREFIX;

    }; // Class-End: Parameter()
} // End of namespace "Toolbox"

// Include implementation-file:
// -------------------------------
#include <robot_toolbox/tools/parameter.inl>

#endif // PARAMETER_TOOL_H 