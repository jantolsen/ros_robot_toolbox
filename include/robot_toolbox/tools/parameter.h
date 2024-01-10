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

        
        // Load Parameter Data
        // -------------------------------
        /** \brief Load Parameter Data to given Target variable.
        *
        * Gets parameter value from given parameter data and loads values into target variable.
        * Function returns true for successful loading of parameter data.
        * If given parameter-member is not found in parameter data or if configured incorrectly,
        * an error message is given and function return false.
        *
        * \param target         Reference to target variable to assign the parameter value to [typename ItemType]
        * \param param_xml      Parameter data to be checked [const XmlRpc::XmlRpcValue&]
        * \param param_member   Parameter member to search for [const std::string&]
        * 
        * \return  Function return: Successful/Unsuccessful (true/false) [bool]
        */
        template<typename ItemType>
        static bool loadParamData(
            ItemType& target,
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member)
        {
            // Get parameter data 
            auto result = getParamData<ItemType>(param_xml, param_member);
            if (!result)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is missing or configured incorrectly");

                // Function return
                return false;
            }

            // Assign parameter value to target variable
            target = result.value();

            // Function return
            return true;
        } // Function end: loadParamData()


        // Load Parameter Data
        // -------------------------------
        /** \brief Load Parameter Data.
        *
        * Gets parameter value from given parameter data.
        * If parameter-member is found within parameter-data, and conversion of data-type is successful, 
        * the function returns with the acquired parameter-data.
        * If given parameter-member is not found in parameter data or if configured incorrectly,
        * an error message is given and a runtime expection is thrown.
        *
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
            auto result = getParamData<ItemType>(param_xml, param_member);
            if (!result)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is missing or configured incorrectly");

                // Throw runtime exception
                throw std::runtime_error("Runtime exception! " + CLASS_PREFIX + __FUNCTION__ 
                    + ": Failed! Parameter [" + param_member + "] is missing or configured incorrectly");
            }

            // Return parameter value
            return result.value();
        } // Function end: loadParamData()


        // Load Parameter Item-Key
        // -------------------------------
        /** \brief Load Parameter Item-Key to given Target variable.
        *
        * Gets parameter value from given parameter data, 
        * validates the parameter data against supplied item-map and loads values into target variable.
        * Function returns true for successful loading of parameter data.
        * If given parameter-member is not found in parameter data, configured incorrectly, 
        * or not part of given item-map, an error message is given and function return false.
        *
        * \param target         Reference to target variable to assign the parameter value to [typename ItemType]
        * \param item_map       Item-Map to act as lookup and validation for parameter-member [std::map<ItemKey, ItemValue>]
        * \param param_xml      Parameter data to be checked [const XmlRpc::XmlRpcValue&]
        * \param param_member   Parameter member to search for [const std::string&]
        * 
        * \return  Function return: Successful/Unsuccessful (true/false) [bool]
        */
        template<typename ItemType, typename ItemKey, typename ItemValue>
        static bool loadParamItemKey(
            ItemType& target,
            const std::map<ItemKey, ItemValue>& item_map,
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member)
        {
            // Get parameter item key 
            auto result = getParamItemKey<ItemType>(item_map, param_xml, param_member);
            if (!result)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is missing or configured incorrectly");

                // Function return
                return false;
            }

            // Assign parameter value to target variable
            target = result.value();

            // Function return
            return true;
        } // Function end: loadParamItemKey()


        // Load Parameter Item-Key
        // -------------------------------
        /** \brief Load Parameter Item-Key Data.
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
        template<typename ItemType, typename ItemKey, typename ItemValue>
        static ItemType loadParamItemKey(
            const std::map<ItemKey, ItemValue>& item_map,
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member)
        {
            // Get parameter item key 
            auto result = getParamItemKey<ItemType>(item_map, param_xml, param_member);
            if (!result)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is missing or configured incorrectly");

                // Throw runtime exception
                throw std::runtime_error("Runtime exception! " + CLASS_PREFIX + __FUNCTION__ 
                    + ": Failed! Parameter [" + param_member + "] is missing or configured incorrectly");
            }

            // Return parameter value
            return result.value();
        } // Function end: loadParamItemKey()


        // Load Parameter Item-Value
        // -------------------------------
        /** \brief Load Parameter Item-Value to given Target variable.
        *
        * Gets parameter value from given parameter data, 
        * validates the parameter data against supplied item-map and loads values into target variable.
        * Function returns true for successful loading of parameter data.
        * If given parameter-member is not found in parameter data, configured incorrectly, 
        * or not part of given item-map, an error message is given and function return false.
        *
        * \param target         Reference to target variable to assign the parameter value to [typename ItemType]
        * \param item_map       Item-Map to act as lookup and validation for parameter-member [std::map<ItemKey, ItemValue>]
        * \param param_xml      Parameter data to be checked [const XmlRpc::XmlRpcValue&]
        * \param param_member   Parameter member to search for [const std::string&]
        * 
        * \return  Function return: Successful/Unsuccessful (true/false) [bool]
        */
        template<typename ItemType, typename ItemKey, typename ItemValue>
        static bool loadParamItemValue(
            ItemType& target,
            const std::map<ItemKey, ItemValue>& item_map,
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member)
        {
            // Get parameter item value 
            auto result = getParamItemValue<ItemType>(item_map, param_xml, param_member);
            if (!result)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is missing or configured incorrectly");

                // Function return
                return false;
            }

            // Assign parameter value to target variable
            target = result.value();

            // Function return
            return true;
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
        template<typename ItemType, typename ItemKey, typename ItemValue>
        static ItemType loadParamItemValue(
            const std::map<ItemKey, ItemValue>& item_map,
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member)
        {
            // Get parameter item value 
            auto result = getParamItemValue<ItemType>(item_map, param_xml, param_member);
            if (!result)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is missing or configured incorrectly");

                // Throw runtime exception
                throw std::runtime_error("Runtime exception! " + CLASS_PREFIX + __FUNCTION__ 
                    + ": Failed! Parameter [" + param_member + "] is missing or configured incorrectly");
            }

            // Return parameter value
            return result.value();
        } // Function end: loadParamItemValue()


        // Get Parameter Data
        // -------------------------------
        /** \brief Get Parameter Data.
        *
        * Searches for parameter-member in given parameter-data. 
        * Data-type of parameter-member is checked and converted to the respective fundamental type.
        * If parameter-member is found within parameter-data, and conversion of data-type is successful, 
        * the function returns with the converted parameter-data.
        * Failed parameter-member check or invalid data-type will result in error message and function returns false
        *
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * 
        * \return Function return: Successful: parameter data [typename ItemType] / Unsuccessful: false [bool]
        */
        template<typename ItemType>
        static boost::optional<ItemType> getParamData(
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member)
        {
            // Check for parameter-member in given parameter-data
            if(!checkMember(param_xml, param_member))
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is missing");

                // Function return
                return boost::none;
            } 

            // Convert parameter-member to respective item-type
            boost::optional<ItemType> result = convertParamType<ItemType>(param_xml[param_member]);
            if(!result)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is configured incorrectly");

                // Function return
                return boost::none;
            }

            // Function return
            return result;
        } // Function end: getParamData()


        // Get Parameter Item: Key
        // -------------------------------
        /** \brief Get Parameter Item Key.
        *
        * Parameter-member is validated against supplied item-map.
        * Searches for parameter-member in given parameter-data. 
        * Data-type of parameter-member is checked and converted to respective fundamental type.
        * Function requires a map to be supplied, this map validates and act as a lookup for passed parameter-member.
        * If parameter-member is found within parameter-data, conversion of data-type is successful, 
        * and parameter-member is found in given map, the function returns with the converted parameter-data.
        * Failed parameter-member check, invalid data-type or no match in map
        * will result in error message and function returns false
        *
        * \param item_map       Item-Map to act as lookup and validation for parameter-member [std::map<ItemKey, ItemValue>]
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * 
        * \return Function return: Successful: parameter data [typename ItemType] / Unsuccessful: false [bool]
        */
        template<typename ItemType, typename ItemKey, typename ItemValue>
        static boost::optional<ItemType> getParamItemKey(
            const std::map<ItemKey, ItemValue>& item_map,
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member)
        {
            // Get Parameter Data
            boost::optional<ItemType> result_param = getParamData<ItemKey>(param_xml, param_member);
            if(!result_param)
            {
                // Function return
                return boost::none;
            }

            // Search in given map using item-key
            // (This will also get the item-value, but this step is only to validate whether the item-key is valid)
            boost::optional<ItemValue> result_search = Toolbox::Map::searchMapByKey(item_map, result_param.value());
            if(!result_search)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is NOT a valid item-key in given item-map");

                // Function return
                return boost::none;
            }
            // Map search success! item-key is found in the container
            // Cast and return item-key
            return static_cast<ItemType>(result_param.value());
        } // Function end: getParamItemKey()


        // Get Parameter Item: Value
        // -------------------------------
        /** \brief Get Parameter Data Item Value.
        *
        * Parameter-member is validated against supplied item-map.
        * Searches for parameter-member in given parameter-data. 
        * Data-type of parameter-member is checked and converted to respective fundamental type.
        * Function requires a map to be supplied, this map validates and act as a lookup for passed parameter-member.
        * If parameter-member is found within parameter-data, conversion of data-type is successful, 
        * and parameter-member is found in given map the function returns with the converted parameter-data.
        * Failed parameter-member check, invalid data-type or no match in map
        * will result in error message and function returns false
        *
        * \param item_map       Item-Map to act as lookup and validation for parameter-member [std::map<ItemKey, ItemValue>]
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * 
        * \return Function return: Successful: parameter data [typename ItemType] / Unsuccessful: false [bool]
        */
        template<typename ItemType, typename ItemKey, typename ItemValue>
        static boost::optional<ItemType> getParamItemValue(
            const std::map<ItemKey, ItemValue>& item_map,
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member)
        {
            // Get Parameter Data
            boost::optional<ItemKey> result_param = getParamData<ItemKey>(param_xml, param_member);
            if(!result_param)
            {
                // Function return
                return boost::none;
            }

            // Search for item-value in given map using item-key
            boost::optional<ItemValue> result_search = Toolbox::Map::searchMapByKey(item_map, result_param.value());
            if(!result_search)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is NOT a valid item-key in given item-map");

                // Function return
                return boost::none;
            }
            // Map search success! item-key is found in the container
            // Cast and return item-value
            return static_cast<ItemType>(result_search.value());
        } // Function end: getParamItemValue()


    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

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
        

        // Convert Parameter Data
        // -------------------------------
        /** \brief Convert Parameter Data.
        *
        * Generic parameter convert/cast function.
        * Given parameter is explicitly casted to fundamental type, using template specilization. 
        * Parameter data-type is checked for valid type match with desired item-type.
        * Function returns casted item if successful convertion. 
        * Invalid data-type will result in error message and function return false
        *
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * 
        * \return Function return: Successful: item [typename ItemType] / Unsuccessful: false [bool]
        */
        template<typename ItemType>
        static boost::optional<ItemType> convertParamType(
            const XmlRpc::XmlRpcValue& param)
        {
            // Convert given parameter to respective item-type
            boost::optional<ItemType> result_param = XmlRpcValueConverter<ItemType>::convert(param);
            if(!result_param)
            {
                // Get human-readable type-names of given parameter and typename item-type
                std::string param_type_name = getParamTypeName(param.getType());
                std::string item_type_name = Convert::getTypeName(typeid(ItemType));

                // Report to terminal
                ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
                    << ": Failed! Parameter value (" << param << ")"
                    << " [XmlRpcValueType: " << param_type_name << "]"
                    << " does NOT convert to a [" << item_type_name << "]");

                // Function return
                return boost::none;
            }

            // Return converted parameter data as item-type
            return result_param;
        } // Function-End: castParamToItem()


    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:
    
        // Prefix message for class
        static const std::string CLASS_PREFIX;


};  // End Class: Parameters
} // End Namespace: Robotics Toolbox
#endif // PARAMETER_TOOL_H