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
#ifndef PARAM_LOADER_H       
#define PARAM_LOADER_H

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
    #include <robot_toolbox/tools/parameter.h>

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
    // Parameter Loader Class/Struct
    // -------------------------------
    // (Template Specialization Primary/Default)
    /** \brief Parameter Loader Class/Struct
    *
    * Parameter Loader contains utility functions for loading and validating 
    * parameter-data from given parameter-container obtained from parameter-server.
    * Functionality is implemented as template-class (utilizing struct)
    * with member function implemented with template specialization(s).
    */
    template<typename ItemType>
    class ParameterLoader
    {
        // // Public Class members
        // // -------------------------------
        // // Accessible for everyone
        // public:

        // // Protected Class members
        // // -------------------------------
        // // Accessible within the class which defines them, 
        // // and classes which inherits from the parent class
        // protected:

        // // Private Class members
        // // -------------------------------
        // // Accessible only for the class which defines them
        // private:
        //     // Prefix message for class
        //     static const std::string CLASS_PREFIX;

        // Prefix for terminal output
        // -------------------------------
        /** \brief Class Prefix function.
        *
        * Returns a prefix-string to be used for terminal output 
        * and debugging purposes.
        * 
        * \return Function return: Prefix [std::string]
        */
        static std::string prefix()
        { 
            return "Toolbox::ParameterLoader::"; 
        } // Function-End: prefix()


        // Get Parameter Data
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
            if(!Toolbox::Parameter::checkMember(param_xml, param_name))
            {
                // Parameter validation failed!
                throwErrorException("Parameter [" + param_name + "] is missing");

                // Function return
                return boost::none;
            } 

            // Convert parameter-member to respective item-type
            boost::optional<ItemType> result = convertParamType<ItemType>(param_xml[param_name]);
            if(!result)
            {
                // Parameter validation failed!
                throwErrorException("Parameter [" + param_name + "] is configured incorrectly");

                // Function return
                return boost::none;
            }

            // Function return
            return result;
        } // Function-End: getParamData()


        // Get Parameter Data
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
            // Unsupported type!
            return boost::none;
        } // Function-End: getParamData()


        // Get Parameter Data
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


        // Convert Parameter Type
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
                std::string param_xml_name = Parameter::getParamTypeName(param_xml.getType());
                std::string item_type_name = Convert::getTypeName(typeid(ItemType));

                // Report to terminal
                ROS_ERROR_STREAM(Prefix()  << __FUNCTION__
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
            ROS_ERROR_STREAM(prefix() + __FUNCTION__ + ": Failed! " + error_msg);

            // Throw runtime exception
            throw std::runtime_error("Runtime exception! " + prefix() + __FUNCTION__ + ": Failed! " + error_msg);
        } // Function-End: throwErrorException()


    }; // Struct-End: ParameterLoader()

} // End of namespace "Toolbox"

// Include implementation-file:
// -------------------------------
#include <robot_toolbox/tools/param_loader.inl>

#endif // PARAM_LOADER_H 