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
    // Parameter Helper
    // -------------------------------
    struct ParameterHelper
    {
        // Prefix for terminal output
        // -------------------------------
        static std::string Prefix()
        {
            return "Toolbox::ParameterHelper::";
        } // Function-End: Prefix()

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
            const std::string& member)
        {
            // Check parameter for specified member
            if(!param.hasMember(member))
            {
                // Report to terminal
                ROS_ERROR_STREAM(Prefix() << __FUNCTION__ 
                    << ": Failed! Given Parameter member [" << member << "] was NOT found");

                // Function return
                return false;
            }
            
            // Function return
            return true;
        } // Function-End: checkMember()


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
            const XmlRpc::XmlRpcValue::Type& type)
        {
            // Check parameter against specified type
            if(param.getType() != type)
            {
                // Report to terminal
                ROS_ERROR_STREAM(Prefix()  << __FUNCTION__ 
                    << ": Failed! Given Parameter data-type [" << getParamTypeName(param.getType()) << "]"
                    << " does NOT match comparing type [" << getParamTypeName(type) << "]");

                // Function return
                return false;
            }

            // Function return
            return true;
        } // Function-End: checkType()


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
            const int& size)
        {
            // Determine that the supplied parameter has a size
            // by checking the data-type
            switch (param.getType())
            {
                // Valid: Data-type with a size
                case XmlRpc::XmlRpcValue::TypeString:
                case XmlRpc::XmlRpcValue::TypeBase64:
                case XmlRpc::XmlRpcValue::TypeArray:
                case XmlRpc::XmlRpcValue::TypeStruct:
                    break;
                // Invalid: Data-type does not have size
                default:
                    // Report to terminal
                    ROS_ERROR_STREAM(Prefix()  << __FUNCTION__ 
                        << ": Failed! Given Parameter data-type does NOT have a size "
                        << " [" << getParamTypeName(param.getType()) << "]");

                    // Function return
                    return false;
            }

            // Check supplied parameter's size against specified size
            if(param.size() != size)
            {
                // Report to terminal
                ROS_ERROR_STREAM(Prefix()  << __FUNCTION__ 
                    << ": Failed! Given Parameter size [" << param.size() << "]" 
                    << " does NOT match comparing size [" << size << "]");

                // Function return
                return false;
            }

            // Function return
            return true;
        } // Function-End: checkSize()

        // Get Name of Parameter Data-Type 
        // -------------------------------
        // (Function Overloading)
        /** \brief Get and converts Parameter Data-Type to a readable type-name
        * 
        * \param param  Parameter to be evaluated [XmlRpc::XmlRpcValue]
        * 
        * \return Name of parameter-type represented as string [std::string]
        */
        static std::string getParamTypeName(
            const XmlRpc::XmlRpcValue& param)
        {
            // Call overloading function
            return getParamTypeName(param.getType());
        }


        // Get Name of Parameter Data-Type 
        // -------------------------------
        // (Function Overloading)
        /** \brief Get and converts Parameter Data-Type to a readable type-name
        * 
        * \param param_type Parameter data-type to be evaluated [XmlRpc::XmlRpcValue::Type]
        * 
        * \return Name of parameter-type represented as string  [std::string]
        */
        static std::string getParamTypeName(
            const XmlRpc::XmlRpcValue::Type& param_type)
        {
            // Determine data-type and represent it as string  
            switch (param_type)
            {
                // Boolean
                case XmlRpc::XmlRpcValue::TypeBoolean:
                    // Function return
                    return "Type-Boolean";
                // Integer
                case XmlRpc::XmlRpcValue::TypeInt:
                    // Function return
                    return "Type-Int";
                // Double
                case XmlRpc::XmlRpcValue::TypeDouble:
                    // Function return
                    return "Type-Double";
                // String
                case XmlRpc::XmlRpcValue::TypeString:
                    // Function return
                    return "Type-String";
                // Date-Time
                case XmlRpc::XmlRpcValue::TypeDateTime:
                    // Function return
                    return "Type-DateTime";
                // Base64/Bytes
                case XmlRpc::XmlRpcValue::TypeBase64:
                    // Function return
                    return "Type-Base64/Bytes";
                // Array
                case XmlRpc::XmlRpcValue::TypeArray:
                    // Function return
                    return "Type-Array";
                // Struct
                case XmlRpc::XmlRpcValue::TypeStruct:
                    // Function return
                    return "Type-Struct";
                // Invalid
                case XmlRpc::XmlRpcValue::TypeInvalid:
                default:
                    // Function return
                    return "Type-Invalid";
            }
        } // Function-End: getParamTypeName()
    }; // Struct-End: ParameterHelper()
    

    // Parameter Loader
    // (Specialization Primary/Default)
    // -------------------------------
    // (Template Specialization)
    template<typename ItemType>
    struct ParameterLoader
    {
        // Prefix for terminal output
        // -------------------------------
        static std::string Prefix()
        { 
            return "Toolbox::ParameterLoader::"; 
        } // Function-End: Prefix()

        // Get Parameter Data
        // -------------------------------
        /** \brief Get Parameter Data.
        *
        * Gets parameter-data from given parameter-container. 
        * The parameter-name is used to search through the parameter-container.
        * If parameter-data is found within given parameter-container 
        * and conversion of data-type is successful, the function return with parameter-data.
        * If the given parameter-data is not found in parameter-container or if configured incorrectly,
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
            // Unsupported type!
            return boost::none;
        } // Function-End: getParamData()


        // Get Parameter Data
        // -------------------------------
        /** \brief Get Parameter Data.
        *
        * Gets parameter-data from given parameter-container
        * and checks it against given validation set. 
        * The parameter-name is used to search through the parameter-container.
        * If parameter-data is found within given parameter-container, 
        * conversion of data-type is successful, and validation check is passed, 
        * the function returns with parameter-data.
        * If the given parameter-data is not found in parameter-container or if configured incorrectly,
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
        * The parameter-name is used to search through the parameter-container.
        * If parameter-data is found within given parameter-container, 
        * conversion of data-type is successful, and the paired value of the parameter-data is found within the map, 
        * the function returns with related paired value of parameter-data.
        * If the given parameter-data is not found in parameter-container or if configured incorrectly,
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
            boost::optional<ItemType> result_param = Toolbox::XmlRpcValueConverter<ItemType>::convert(param_xml);
            if(!result_param)
            {
                // Get human-readable type-names of given parameter and typename item-type
                std::string param_xml_name = ParameterHelper::getParamTypeName(param_xml.getType());
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
    }; // Struct-End: ParameterLoader()


    // Parameter Loader
    // (Specialization Primary/Default)
    // -------------------------------
    // (Template Specialization)
    template<>
    struct ParameterLoader<std::string>
    {
        // Get Parameter Data
        // -------------------------------
        /** \brief Get Parameter Data.
        *
        * Gets parameter-data from given parameter-container
        * and checks it against given validation set. 
        * The parameter-name is used to search through the parameter-container.
        * If parameter-data is found within given parameter-container, 
        * conversion of data-type is successful, and validation check is passed, 
        * the function returns with parameter-data.
        * If the given parameter-data is not found in parameter-container or if configured incorrectly,
        * an error message is given and a runtime expection is thrown.
        * 
        * \param param_xml      Parameter-container to search through [const XmlRpc::XmlRpcValue&]
        * \param param_name     Parameter-name to search for [const std::string&]
        * \param validation_set Validation-set vector to compare against the parameter-data [const std::vector<std::string>&]
        * 
        * \return Function return: parameter data [std::string]
        */
        static boost::optional<std::string> getParamData(
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_name,
            const std::vector<std::string>& validation_set)
        {
            // Unsupported type!
            return boost::none;
        } // Function-End: getParamData<std::string>()
    }; // Struct-End: ParameterLoader<std::string>()

} // End Namespace: Robotics Toolbox
#endif // PARAM_TOOL_H