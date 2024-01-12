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
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
#ifndef PARAM_LOADER_H
#define PARAM_LOADER_H

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
    #include "robot_toolbox/tools/param_helper.h"
    #include "robot_toolbox/tools/param_converter.h"

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
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:

            // Get Parameter Data
            // (Template: Primary/Default)
            // -------------------------------
            // (Function Overloading)
            /** \brief Get Parameter Data.
            *
            * Gets parameter-data from given parameter-container. 
            * The parameter-name is used to as a key to search through the parameter-container.
            * If parameter-data is found within given parameter-container 
            * and conversion of data-type is successful, the function returns with parameter-data.
            * If function fails to find parameter-data or if its configured incorrectly,
            * an error message is given and a runtime expection is thrown.
            * 
            * \param param_xml          Parameter-container to search through [const XmlRpc::XmlRpcValue&]
            * \param param_name         Parameter-name to search for [const std::string&]
            * 
            * \return Function return: parameter data [typename ItemType]
            */
            static boost::optional<ItemType> getParamData(
                const XmlRpc::XmlRpcValue& param_xml,
                const std::string& param_name)
            {
                // Check for parameter-member in given parameter-data
                if(!ParameterHelper::checkMember(param_xml, param_name))
                {
                    // Parameter validation failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        << ": Failed! Parameter [" << param_name <<"] is missing");

                    // Function return
                    return boost::none;
                } 

                // Convert parameter-member to respective item-type
                boost::optional<ItemType> result = convertParamType(param_xml[param_name]);
                if(!result)
                {
                    // Parameter validation failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        << ": Failed! Parameter [" << param_name <<"] is configured incorrectly");

                    // Function return
                    return boost::none;
                }

                // Function return
                return result;
            } // Function-End: getParamData()


            // Get Parameter Data
            // (Template: Primary/Default)
            // -------------------------------
            // (Function Overloading)
            /** \brief Get Parameter Data.
            *
            * Gets parameter-data from given parameter-container
            * and checks it against given validation set. 
            * The parameter-name is used to as a key to search through the parameter-container.
            * If parameter-data is found within given parameter-container,
            * conversion of data-type is successful, and validation check is passed,
            * the function returns with parameter-data.
            * If function fails to find parameter-data, or if its configured incorrectly,
            * or if validation check fails, 
            * an error message is given and a runtime expection is thrown.
            *
            * \param param_xml      Parameter-container to search through [const XmlRpc::XmlRpcValue&]
            * \param param_name     Parameter-name to search for [const std::string&]
            * \param validation_set Validation-set vector to compare against the parameter-data [const std::vector<typename ItemType>&]
            * 
            * \return Function return: parameter data [typename ItemType]
            */
            static boost::optional<ItemType> getParamData(
                const XmlRpc::XmlRpcValue& param_xml,
                const std::string& param_name,
                const std::vector<ItemType>& validation_set)
            {
                // Call overloading function
                boost::optional<ItemType> result = getParamData(param_xml, param_name);
                if(!result)
                {
                    // Function return
                    return boost::none;
                }

                // Check if parameter-data is within validation-set
                if(!validateAgainstSet(result.value(), validation_set))
                {
                    // Parameter validation failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        << ": Failed! Parameter [" << param_name <<"] with value [" << result.value() << "] is NOT a valid entry");

                    // Function return
                    return boost::none;
                }

                // Function return
                return result;
            } // Function-End: getParamData()


            // Get Parameter Data
            // (Template: Primary/Default)
            // -------------------------------
            /** \brief Get Parameter Data.
            *
            * Gets parameter-data from given parameter-container 
            * and uses it as a key to find the corresponding paired value in the given map. 
            * The parameter-name is used to as a key to search through the parameter-container.
            * If parameter-data is found within given parameter-container,
            * conversion of data-type is successful, and paired value of the parameter-data is found within the map,
            * the function returns with related paired value of parameter-data.
            * If function fails to find parameter-data, or if its configured incorrectly,
            * of if given parameter-data is not found in parameter-container, 
            * an error message is given and a runtime expection is thrown.
            * 
            * \param param_xml      Parameter-container to search through [const XmlRpc::XmlRpcValue&]
            * \param param_name     Parameter-name to search for [const std::string&]
            * \param item_map       Item-map to search for the related paired value of parameter-data [std::map<MapKey, MapValue>]
            * 
            * \return Function return: parameter data [typename ItemType]
            */
            template<typename MapKey, typename MapValue>
            static boost::optional<ItemType> getParamData(
                const XmlRpc::XmlRpcValue& param_xml,
                const std::string& param_name,
                const std::map<MapKey, MapValue>& item_map)
            {
                // Unsupported type!
                return boost::none;
            } // Function-End: getParamData()


        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

            // Convert Parameter Type
            // (Template: Primary/Default)
            // -------------------------------
            /** \brief Convert Parameter Type.
            *
            * Generic parameter convert/cast function.
            * Given parameter is explicitly casted to fundamental type, using template specilization. 
            * Parameter data-type is checked for valid type match with desired item-type.
            * Function returns casted item if successful convertion. 
            * Invalid data-type will result in error message and function return false
            *
            * \param param_xml  Parameter to be converted [XmlRpc::XmlRpcValue]
            * 
            * \return Function return: Successful: item [typename ItemType] / Unsuccessful: false [bool]
            */
            static boost::optional<ItemType> convertParamType(
                const XmlRpc::XmlRpcValue& param_xml)
            {
                // Convert given parameter to respective item-type
                boost::optional<ItemType> result_param = ParameterConverter<ItemType>::convert(param_xml);
                if(!result_param)
                {
                    // Get human-readable type-names of given parameter and typename item-type
                    std::string param_xml_name = ParameterHelper::getParamTypeName(param_xml.getType());
                    std::string item_type_name = Convert::getTypeName(typeid(ItemType));

                    // Report to terminal
                    ROS_ERROR_STREAM(CLASS_PREFIX  << __FUNCTION__
                        << ": Failed! Parameter value (" << param_xml << ")"
                        << " [XmlRpcValueType: " << param_xml_name << "]"
                        << " does NOT convert to a [" << item_type_name << "]");

                    // Function return
                    return boost::none;
                }

                // Return converted parameter data as item-type
                return result_param;
            } // Function-End: convertParamType()
            

            // Throw Error Exception
            // -------------------------------
            /** \brief Throw Error Exception.
            *
            * Helper function for printing ros-error and throwing runtime exception.
            * Function takes an error-message as argument and prints it as ros-error to terminal.
            * and then throws a runtime exception with the same error-message.
            *
            * \param error_msg  Error message to print [std::string&]
            */
            static void throwErrorException(
                const std::string& error_msg)
            {
                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! " << error_msg);

                // Throw runtime exception
                throw std::runtime_error("Runtime exception! " + CLASS_PREFIX + __FUNCTION__ 
                    + ": Failed! " + error_msg);
            } // Function-End: throwErrorException()


        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:
            // Prefix message for class
            static const std::string CLASS_PREFIX;

    }; // Class-End: ParameterLoader()


    // Parameter Class
    // -------------------------------
    /** \brief Parameter Loader Class
    *
    * Parameter Loader contains utility functions for loading and validating 
    * parameter-data from given parameter-container obtained from parameter-server.
    * Functionality is implemented as template-class
    * with member function implemented with template specialization(s).
    */
    class ParameterTest
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:
            // Load Parameter Item-Value
            // -------------------------------
            /** \brief Load Parameter Item-Value data.
            *
            * Gets parameter value from given parameter data and validates it against supplied item-map.
            * If parameter-member is found within parameter-data, conversion of data-type 
            * and validation of parameter against supplied map is successful,
            * the function returns with the acquired parameter-data.
            * If given parameter-member is not found in parameter data, configured incorrectly, 
            * or not part of given item-map, is given and a runtime expection is thrown.
            *
            * \param item_map       Item-Map to act as lookup and validation for parameter-member [std::map<ItemKey, ItemValue>]
            * \param param_xml      Parameter data to be checked [const XmlRpc::XmlRpcValue&]
            * \param param_member   Parameter member to search for [const std::string&]
            * 
            * \return Function return: parameter data [typename ItemType]
            */
            template<typename ItemType>
            static ItemType loadParamData(
                const XmlRpc::XmlRpcValue& param_xml,
                const std::string& param_member)
            {
                // Get parameter data 
                auto result = ParameterLoader<ItemType>::getParamData(param_xml, param_member);
                if (!result)
                {
                    // Parameter validation failed
                    ROS_ERROR_STREAM("Toolbox::ParameterTest::" << __FUNCTION__ 
                        << ": Failed! Parameter [" << param_member <<"] is missing or configured incorrectly");

                    // Throw runtime exception
                    throw std::runtime_error("Runtime exception! ");
                }

                // Return parameter value
                return result.value();
            } // Function end: loadParamItemValue()


            // Load Parameter Item-Value
            // -------------------------------
            /** \brief Load Parameter Item-Value data.
            *
            * Gets parameter value from given parameter data and validates it against supplied item-map.
            * If parameter-member is found within parameter-data, conversion of data-type 
            * and validation of parameter against supplied map is successful,
            * the function returns with the acquired parameter-data.
            * If given parameter-member is not found in parameter data, configured incorrectly, 
            * or not part of given item-map, is given and a runtime expection is thrown.
            *
            * \param item_map       Item-Map to act as lookup and validation for parameter-member [std::map<ItemKey, ItemValue>]
            * \param param_xml      Parameter data to be checked [const XmlRpc::XmlRpcValue&]
            * \param param_member   Parameter member to search for [const std::string&]
            * 
            * \return Function return: parameter data [typename ItemType]
            */
            template<typename ItemType>
            static ItemType loadParamData(
                const XmlRpc::XmlRpcValue& param_xml,
                const std::string& param_member,
                const std::vector<ItemType>& validation_set)
            {
                // Get parameter data 
                auto result = ParameterLoader<ItemType>::getParamData(param_xml, param_member, validation_set);
                if (!result)
                {
                    // Parameter validation failed
                    ROS_ERROR_STREAM("Toolbox::ParameterTest::" << __FUNCTION__ 
                        << ": Failed! Parameter [" << param_member <<"] is missing or configured incorrectly");

                    // Throw runtime exception
                    throw std::runtime_error("Runtime exception! ");
                }

                // Return parameter value
                return result.value();
            } // Function end: loadParamItemValue()
    }; // Class-End: ParameterTest()
} // End of namespace "Toolbox"

// Include implementation-file:
// -------------------------------
#include <robot_toolbox/tools/param_loader.inl>

#endif // PARAM_LOADER_H 