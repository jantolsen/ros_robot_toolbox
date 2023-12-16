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

    // Boost
    #include <boost/optional.hpp>

    // Ros
    #include <ros/ros.h>

    // Ros Messages
    #include "sensor_msgs/JointState.h"

    // Robotics Toolbox
    #include "robot_toolbox/tools/common.h"
    #include "robot_toolbox/tools/convert.h"
    #include "robot_toolbox/tools/map.h"
    

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

        
        // Get Parameter Data: Bool 
        // -------------------------------
        /** \brief Search for member in given parameter-data and returns parameter-value if successful.
        * 
        * Function checks if the supplied parameter-data contains the parameter-member. 
        * Data-type of the parameter-member is checked and gets converted to respecitve value-type if successful.
        * Failed parameter-member check or invalid data-type will result in error message and function returns false
        * 
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * 
        * \return Function return: Successful: parameter value [bool] / Unsuccessful: false [bool]
        */
        static boost::optional<bool> getParamBool(
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member);


        // Get Parameter Data: Int 
        // -------------------------------
        /** \brief Search for member in given parameter-data and returns parameter-value if successful.
        * 
        * Function checks if the supplied parameter-data contains the parameter-member. 
        * Data-type of the parameter-member is checked and gets converted to respecitve value-type if successful.
        * Failed parameter-member check or invalid data-type will result in error message and function returns false
        * 
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * 
        * \return Function return: Successful: parameter value [int] / Unsuccessful: false [bool]
        */
        static boost::optional<int> getParamInt(
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member);


        // Get Parameter Data: Double 
        // -------------------------------
        /** \brief Search for member in given parameter-data and returns parameter-value if successful.
        * 
        * Function checks if the supplied parameter-data contains the parameter-member. 
        * Data-type of the parameter-member is checked and gets converted to respecitve value-type if successful.
        * Failed parameter-member check or invalid data-type will result in error message and function returns false
        * 
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * 
        * \return Function return: Successful: parameter value [double] / Unsuccessful: false [bool]
        */
        static boost::optional<double> getParamDouble(
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member);


        // Get Parameter Data: String 
        // -------------------------------
        /** \brief Search for member in given parameter-data and returns parameter-value if successful.
        * 
        * Function checks if the supplied parameter-data contains the parameter-member. 
        * Data-type of the parameter-member is checked and gets converted to respecitve value-type if successful.
        * Failed parameter-member check or invalid data-type will result in error message and function returns false
        * 
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * 
        * \return Function return: Successful: parameter value [std::string] / Unsuccessful: false [bool]
        */
        static boost::optional<std::string> getParamString(
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member);


        // Get Parameter Data: Type-Name 
        // -------------------------------
        /** \brief Search for member in given parameter-data and returns parameter-value if successful.
        * 
        * Function checks if the supplied parameter-data contains the parameter-member. 
        * Data-type of the parameter-member is checked and gets converted to respecitve value-type if successful.
        * Function requires a type-map to be supplied, to validate and act as a lookup for passed parameter-member.
        * Failed parameter-member check or invalid data-type will result in error message and function returns false
        * 
        * \param type_map       Type-Map to act as lookup and validation for parameter-member [std::map<std::string, TypeInfo>]
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * 
        * \return Function return: Successful: parameter type-name value [std::string] / Unsuccessful: false [bool]
        */
        template<typename TypeInfo>
        static boost::optional<std::string> getParamTypeName(
            const std::map<std::string, TypeInfo>& type_map,
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

            // Convert parameter-member to string-type
            boost::optional<std::string> result_cast = castParamToString(param_xml[param_member]);
            if(!result_cast)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is configured incorrectly");

                // Function return
                return boost::none;
            }

            // Assign parameter type-name as casted string-type
            std::string param_type_name = result_cast.value();
            
            // Search for parameter type-name in related type-map
            // (This will also get the type-info value, but this step is only to validate whether the type-name is valid)
            boost::optional<TypeInfo> result_search = searchTypeMap(type_map, param_type_name);
            if(!result_search)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is NOT a valid type-name");

                // Function return
                return boost::none;
            }

            // Map search success! type-name is found in the container
            // Convert to all upper-case and return type-name value
            return Toolbox::Convert::stringToUpperCase(param_type_name);
        } // Function end: getParamTypeName()



        // Get Parameter Data: Type-Info 
        // (std::map)
        // -------------------------------
        /** \brief Search for member in given parameter-data and returns parameter-value if successful.
        * 
        * Function checks if the supplied parameter-data contains the parameter-member. 
        * Data-type of the parameter-member is checked and gets converted to respecitve value-type if successful.
        * Function requires a type-map to be supplied, to validate and act as a lookup for passed parameter-member.
        * Failed parameter-member check or invalid data-type will result in error message and function returns false
        * 
        * \param type_map       Type-Map to act as lookup and validation for parameter-member [std::map<std::string, TypeInfo>]
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * 
        * \return Function return: Successful: parameter type-info value [TypeValue] / Unsuccessful: false [bool]
        */
        template<typename TypeValue>
        static boost::optional<TypeValue> getParamTypeInfo(
            const std::map<std::string, TypeValue>& type_map,
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

            // Convert parameter-member to string-type
            boost::optional<std::string> result_cast = castParamToString(param_xml[param_member]);
            if(!result_cast)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is configured incorrectly");

                // Function return
                return boost::none;
            }

            // Assign parameter type-name as casted string-type
            std::string param_type_name = result_cast.value();
            
            // Search for parameter type-name in related type-map and get related type-info value
            boost::optional<TypeValue> result_search = searchTypeMap(type_map, param_type_name);
            if(!result_search)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is NOT a valid type-name");

                // Function return
                return boost::none;
            }

            // Map search success! type-name is found in the container
            // Return related type-info value
            return result_search;
        } // Function end: getParamTypeInfo()


        // Get Parameter Data: Item-Key 
        // -------------------------------
        /** \brief Search for member in given parameter-data and returns parameter-value if successful.
        * 
        * Function checks if the supplied parameter-data contains the parameter-member. 
        * Data-type of the parameter-member is checked and gets converted to respecitve value-type if successful.
        * Function requires a type-map to be supplied, to validate and act as a lookup for passed parameter-member.
        * Failed parameter-member check or invalid data-type will result in error message and function returns false
        * 
        * \param item_map       Item-Map to act as lookup and validation for parameter-member [std::map<ItemKey, ItemValue>]
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * 
        * \return Function return: Successful: parameter item-key [ItemKey] / Unsuccessful: false [bool]
        */
        template<typename ItemKey, typename ItemValue>
        static boost::optional<ItemKey> getParamItemKey(
            const std::map<ItemKey, ItemValue>& item_map,
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
            boost::optional<ItemKey> result_cast = castParamToItemType<ItemKey>(param_xml[param_member]);
            if(!result_cast)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is configured incorrectly");

                // Function return
                return boost::none;
            }
            
            // Assign parameter item-key as casted item-type
            std::string param_item_key = result_cast.value();

            // Search for parameter item-key in related item-map
            // (This will also get the item-value, but this step is only to validate whether the item-key is valid)
            boost::optional<ItemValue> result_search = searchTypeMap(item_map, param_item_key);
            if(!result_search)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is NOT a valid item-key in given item-map");

                // Function return
                return boost::none;
            }
            // Map search success! item-key is found in the container
            // Return item-key
            return param_item_key;
        } // Function end: getParamTypeName()


        // Load Parameter Data: Bool 
        // -------------------------------
         /** \brief Load Parameter Data to given target member
         * 
         * This function loads a parameter from the given parameter data and assigns its value to the target variable.
         * If the parameter is missing or configured incorrectly, an error message is logged and the function returns false.
         * 
         * \param target        Reference to target variable to assign the parameter value to [typename TargetType]
         * \param param_xml     Parameter data to be checked [const XmlRpc::XmlRpcValue&]
         * \param param_member  Member item to search for within the parameter [const std::string&]
         * 
         * \return  Function return: Successful/Unsuccessful (true/false) [bool]
         */
        template<typename TargetType>
        static bool loadParamBool(
            TargetType& target,
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member)
        {
            // Get Bool parameter data
            auto result = getParamBool(param_xml, param_member);

            // Check if parameter-data is valid
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
        }


        // Load Parameter Data: Int 
        // -------------------------------
         /** \brief Load Parameter Data to given target member
         * 
         * This function loads a parameter from the given parameter data and assigns its value to the target variable.
         * If the parameter is missing or configured incorrectly, an error message is logged and the function returns false.
         * 
         * \param target        Reference to target variable to assign the parameter value to [typename TargetType]
         * \param param_xml     Parameter data to be checked [const XmlRpc::XmlRpcValue&]
         * \param param_member  Member item to search for within the parameter [const std::string&]
         * 
         * \return  Function return: Successful/Unsuccessful (true/false) [bool]
         */
        template<typename TargetType>
        static bool loadParamInt(
            TargetType& target,
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member)
        {
            // Get Int parameter data
            auto result = getParamInt(param_xml, param_member);

            // Check if parameter-data is valid
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
        }

        
        // Load Parameter Data: Double 
        // -------------------------------
         /** \brief Load Parameter Data to given target member
         * 
         * This function loads a parameter from the given parameter data and assigns its value to the target variable.
         * If the parameter is missing or configured incorrectly, an error message is logged and the function returns false.
         * 
         * \param target        Reference to target variable to assign the parameter value to [typename TargetType]
         * \param param_xml     Parameter data to be checked [const XmlRpc::XmlRpcValue&]
         * \param param_member  Member item to search for within the parameter [const std::string&]
         * 
         * \return  Function return: Successful/Unsuccessful (true/false) [bool]
         */
        template<typename TargetType>
        static bool loadParamDouble(
            TargetType& target,
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member)
        {
            // Get Double parameter data
            auto result = getParamDouble(param_xml, param_member);

            // Check if parameter-data is valid
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
        }


        // Load Parameter Data: String 
        // -------------------------------
         /** \brief Load Parameter Data to given target member
         * 
         * This function loads a parameter from the given parameter data and assigns its value to the target variable.
         * If the parameter is missing or configured incorrectly, an error message is logged and the function returns false.
         * 
         * \param target        Reference to target variable to assign the parameter value to [typename TargetType]
         * \param param_xml     Parameter data to be checked [const XmlRpc::XmlRpcValue&]
         * \param param_member  Member item to search for within the parameter [const std::string&]
         * 
         * \return  Function return: Successful/Unsuccessful (true/false) [bool]
         */
        template<typename TargetType>
        static bool loadParamString(
            TargetType& target,
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member)
        {
            // Get String parameter data
            auto result = getParamString(param_xml, param_member);

            // Check if parameter-data is valid
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
        } // Function end: loadParamString()

        
        // Load Parameter Data: Type-Name 
        // -------------------------------
         /** \brief Load Parameter Data to given target member
         * 
         * This function loads a parameter from the given parameter data and assigns its value to the target variable.
         * Function requires a type-map to be supplied, to validate and act as a lookup for passed parameter-member.
         * If the parameter is missing or configured incorrectly, an error message is logged and the function returns false.
         * 
         * \param target        Reference to target variable to assign the parameter value to [typename TargetType]
         * \param param_xml     Parameter data to be checked [const XmlRpc::XmlRpcValue&]
         * \param param_member  Member item to search for within the parameter [const std::string&]
         * \param type_map      Type-Map to act as lookup and validation for parameter-member [std::map<std::string, TypeInfo>]
         * 
         * \return  Function return: Successful/Unsuccessful (true/false) [bool]
         */
        template<typename TargetType, typename TypeInfo>
        static bool loadParamTypeName(
            TargetType& target,
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member,
            const std::map<std::string, TypeInfo>& type_map)
        {
            // Get Type-Name parameter data
            auto result = getParamTypeName(type_map, param_xml, param_member);

            // Check if parameter-data is valid
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
        } // Function end: loadParamTypeName()


        // Load Parameter Data: Type-Info 
        // -------------------------------
         /** \brief Load Parameter Data to given target member
         * 
         * This function loads a parameter from the given parameter data and assigns its value to the target variable.
         * Function requires a type-map to be supplied, to validate and act as a lookup for passed parameter-member.
         * If the parameter is missing or configured incorrectly, an error message is logged and the function returns false.
         * 
         * \param target        Reference to target variable to assign the parameter value to [typename TargetType]
         * \param param_xml     Parameter data to be checked [const XmlRpc::XmlRpcValue&]
         * \param param_member  Member item to search for within the parameter [const std::string&]
         * \param type_map      Type-Map to act as lookup and validation for parameter-member [std::map<std::string, TypeInfo>]
         * 
         * \return  Function return: Successful/Unsuccessful (true/false) [bool]
         */
        template<typename TargetType, typename TypeInfo>
        static bool loadParamTypeInfo(
            TargetType& target,
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member,
            const std::map<std::string, TypeInfo>& type_map)
        {
            // Get Type-Name parameter data
            auto result = getParamTypeInfo(type_map, param_xml, param_member);

            // Check if parameter-data is valid
            if (!result)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is missing or configured incorrectly");

                // Function return
                return false;
            }

            // Convert parameter-value to int-type
            boost::optional<int> result_cast = castParamToInt(result.value());
            if(!result_cast)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is configured incorrectly");

                // Function return
                return false;
            }

            // Assign target variable
            target = result_cast.value();

            // Function return
            return true;
        } // Function end: loadParamTypeInfo()


        // Load Parameter Data: Item-Key 
        // -------------------------------
         /** \brief Load Parameter Data to given target member
         * 
         * This function loads a parameter from the given parameter data and assigns its value to the target variable.
         * Function requires a item-map to be supplied, to validate and act as a lookup for passed parameter-member.
         * If the parameter is missing or configured incorrectly, an error message is logged and the function returns false.
         * 
         * \param target        Reference to target variable to assign the parameter value to [typename ItemKey]
         * \param param_xml     Parameter data to be checked [const XmlRpc::XmlRpcValue&]
         * \param param_member  Member item to search for within the parameter [const std::string&]
         * \param item_map      Item-Map to act as lookup and validation for parameter-member [std::map<ItemKey, ItemValue>]
         * 
         * \return  Function return: Successful/Unsuccessful (true/false) [bool]
         */
        template<typename ItemKey, typename ItemValue>
        static bool loadParamItemKey(
            ItemKey& target,
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member,
            const std::map<ItemKey, ItemValue>& item_map)
        {
            // Get Type-Name parameter data
            auto result = getParamItemKey(item_map, param_xml, param_member);

            // Check if parameter-data is valid
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


        // Convert Parameter Data to Bool-Value 
        // -------------------------------
        /** \brief Convert parameter data to bool-value
        * 
        * Parameter is checked for data-type and returns bool-value if successful.
        * Invalid data-type will result in error message and function return false
        * 
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * 
        * \return Function return: Successful: parameter value [bool] / Unsuccessful: false [bool]
        */
        static boost::optional<bool> castParamToBool(
            const XmlRpc::XmlRpcValue& param);

        
        // Convert Parameter Data to Int-Value 
        // -------------------------------
        /** \brief Convert parameter data to int-value
        * 
        * Parameter is checked for data-type and returns int-value if successful.
        * Invalid data-type will result in error message and function return false
        * 
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * 
        * \return Function return: Successful: parameter value [int] / Unsuccessful: false [bool]
        */
        static boost::optional<int> castParamToInt(
            const XmlRpc::XmlRpcValue& param);


        // Convert Parameter Data to Double-Value 
        // -------------------------------
        /** \brief Convert parameter data to double-value
        * 
        * Parameter is checked for data-type and returns double-value if successful.
        * Invalid data-type will result in error message and function return false
        * 
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * 
        * \return Function return: Successful: parameter value [double] / Unsuccessful: false [bool]
        */
        static boost::optional<double> castParamToDouble(
            const XmlRpc::XmlRpcValue& param);


        // Convert Parameter Data to String-Value 
        // -------------------------------
        /** \brief Convert parameter data to string-value
        * 
        * Parameter is checked for data-type and returns string-value if successful.
        * Invalid data-type will result in error message and function return false
        * 
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * 
        * \return Function return: Successful: parameter value [std::string] / Unsuccessful: false [bool]
        */
        static boost::optional<std::string> castParamToString(
            const XmlRpc::XmlRpcValue& param);


        // Convert Parameter Data to Item-Type-Value 
        // -------------------------------
        /** \brief Generic cast function. Convert parameter data to item-value
        * 
        * Parameter is checked for data-type and is cast to respective type
        * and returns item-value if successful.
        * Invalid data-type will result in error message and function return false
        * 
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * 
        * \return Function return: Successful: parameter value [ParamType] / Unsuccessful: false [bool]
        */
        template<typename ItemType>
        static boost::optional<ItemType> castParamToItemType(
            const XmlRpc::XmlRpcValue& param)
        {
            return XmlRpcValueConverter<ItemType>::convert(param);

            // // Local variable(s)
            // // (copy input-parameter to remove const to allow type-casting)
            // XmlRpc::XmlRpcValue param_ = param;

            // // Determine data-type to cast to respective type  
            // switch (param_.getType())
            // {
            //     // Boolean
            //     case XmlRpc::XmlRpcValue::TypeBoolean:
            //         // Call respective cast-function
            //         return castParamToBool(param);

            //     // Int
            //     case XmlRpc::XmlRpcValue::TypeInt:
            //         // Call respective cast-function
            //         return castParamToInt(param);

            //     // Double
            //     case XmlRpc::XmlRpcValue::TypeDouble:
            //         // Call respective cast-function
            //         return castParamToDouble(param);

            //     // String
            //     case XmlRpc::XmlRpcValue::TypeString:
            //         // Call respective cast-function
            //         auto result = castParamToString(param);
            //         // (Convert resulting string to all upper-case)
            //         return Toolbox::Convert::stringToUpperCase(result.value());

            //     // Default/Invalid
            //     default:
            //         // Report to terminal
            //         ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__
            //             << ": Failed! Parameter [" << param_ << "]"
            //             << " (XmlRpcType: " << getDataTypeName(param.getType()) << ")"
            //             << " does NOT convert to a fundamental-type");

            //         // Function return
            //         return boost::none;
            // }
        } // Function-End: castParamToItem()


        // Search Type-Map:
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given type-name in supplied type-map. 
        * 
        * If given type-name is found within the map, function returns the related type-info value of the container-pair.
        * If no search-item is found within the map, function returns false 
        * Map search will ignore capitalization of letters in key-string.
        * 
        * \param type_map   Type-Map to search thorugh [boost::bimap<std::string, typename TypeInfo>]
        * \param type_name  Type-Name to search for (key) [std::string]
        * 
        * \return Function return: Successful: Type-Info value [typename TypeInfo] / Unsuccessful: false [bool]
        */
        template<typename TypeInfo>
        static boost::optional<TypeInfo> searchTypeMap(
            const std::map<std::string, TypeInfo>& type_map,
            const std::string& type_name)
        {
            // Call map search function
            boost::optional<TypeInfo> result = Toolbox::Map::searchMapByKey(type_map, type_name);
            if(!result)
            {
                // Map search failed! Type-Info key (left-element) is NOT found in the container
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Given type-name [" << type_name <<"] was NOT found in given type-map");

                // Function return
                return boost::none;
            } 
        
            // Function return
            return result;
        } // Function-End: searchTypeMapByName()




    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

        // Get Name of Parameter Data-Type 
        // -------------------------------
        /** \brief Get and converts Parameter Data-Type to a readable type-name
        * 
        * \param type       Data-type to compare parameter against [XmlRpc::XmlRpcValue::Type]
        * 
        * \return Name of parameter-type represented [std::string]
        */
        static std::string getDataTypeName(
            const XmlRpc::XmlRpcValue::Type& type);


    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:
    
        // Prefix message for class
        static const std::string CLASS_PREFIX;


        // XmlRpcValue Converter
        // (Specialization Primary/Default)
        // -------------------------------
        // (Template Specialization)
        template<typename ParamType>
        struct XmlRpcValueConverter
        {
            // Convert XmlRpcValue Parameter
            // -------------------------------
            /** \brief Convert XmlRpcValue to fundemental type
            *
            * Parameter is checked for data-type and explicitly cast to a fundamental type.
            * Invalid data-type will result in error message and function return false
            * 
            * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
            * 
            * \return Function return: Successful: parameter value [ParamType] / Unsuccessful: false [bool]
            */
            static boost::optional<ParamType> convert(
                const XmlRpc::XmlRpcValue& param) 
            {
                // Unsupported type!
                // Report to terminal
                ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
                    << ": Failed! Parameter [" << param << "]"
                    << " (XmlRpcType: " << getDataTypeName(param.getType()) << ")"
                    << " does NOT convert to a fundamental-type");

                // Function return
                return boost::none;
            }
        };


};  // End Class: Parameters

} // End Namespace: Robotics Toolbox
#endif // PARAMETER_TOOL_H