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
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * \param member     Member item to search for within parameter [std::string]
        * \return Function result: Supplied parameter contains specified member (true/false)
        */
        static bool checkMember(
            const XmlRpc::XmlRpcValue& param, 
            const std::string& member);


        // Check and Compare Parameter Type
        // -------------------------------
        /** \brief Check if supplied parameter equals specified parameter data-type.
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * \param type       Data-type to compare parameter against [XmlRpc::XmlRpcValue::Type]
        * \return Function result: Supplied parameter matches type of specified data-type (true/false)
        */
        static bool checkDataType(
            const XmlRpc::XmlRpcValue& param, 
            const XmlRpc::XmlRpcValue::Type& type);


        // Check and Compare Parameter Size
        // -------------------------------
        /** \brief Check if supplied parameter equals specified size.
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * \param size       Size to compare parameter against [int]
        * \return Function result: Supplied paramter matches specified size (true/false)
        */
        static bool checkSize(
            const XmlRpc::XmlRpcValue& param, 
            const int& size);


        // Check Parameter
        // -------------------------------
        // (Function Overloading)
        /** \brief Check that supplied parameter contains member and equals data-type.
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * \param member     Member item to search for within parameter [std::string]
        * \param type       Data-type to compare parameter against [XmlRpc::XmlRpcValue::Type]
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
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * \param member     Member item to search for within parameter [std::string]
        * \param type       Data-type to compare parameter against [XmlRpc::XmlRpcValue::Type]
        * \param size       Size to compare parameter against [int]
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
        * Function checks if the supplied parameter-data contains the parameter-member. 
        * Data-type of the parameter-member is checked and gets converted to respecitve value-type if successful.
        * Failed parameter-member check or invalid data-type will result in error message and function returns false
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * \return Function return: Successful: parameter value [bool] / Unsuccessful: false [bool]
        */
        static boost::optional<bool> getParamBool(
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member);


        // Get Parameter Data: Int 
        // -------------------------------
        /** \brief Search for member in given parameter-data and returns parameter-value if successful.
        * Function checks if the supplied parameter-data contains the parameter-member. 
        * Data-type of the parameter-member is checked and gets converted to respecitve value-type if successful.
        * Failed parameter-member check or invalid data-type will result in error message and function returns false
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * \return Function return: Successful: parameter value [int] / Unsuccessful: false [bool]
        */
        static boost::optional<int> getParamInt(
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member);


        // Get Parameter Data: Double 
        // -------------------------------
        /** \brief Search for member in given parameter-data and returns parameter-value if successful.
        * Function checks if the supplied parameter-data contains the parameter-member. 
        * Data-type of the parameter-member is checked and gets converted to respecitve value-type if successful.
        * Failed parameter-member check or invalid data-type will result in error message and function returns false
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * \return Function return: Successful: parameter value [double] / Unsuccessful: false [bool]
        */
        static boost::optional<double> getParamDouble(
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member);


        // Get Parameter Data: String 
        // -------------------------------
        /** \brief Search for member in given parameter-data and returns parameter-value if successful.
        * Function checks if the supplied parameter-data contains the parameter-member. 
        * Data-type of the parameter-member is checked and gets converted to respecitve value-type if successful.
        * Failed parameter-member check or invalid data-type will result in error message and function returns false
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * \return Function return: Successful: parameter value [std::string] / Unsuccessful: false [bool]
        */
        static boost::optional<std::string> getParamString(
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member);


        // Get Parameter Data: Type-Name 
        // -------------------------------
        /** \brief Search for member in given parameter-data and returns parameter-value if successful.
        * Function checks if the supplied parameter-data contains the parameter-member. 
        * Data-type of the parameter-member is checked and gets converted to respecitve value-type if successful.
        * Function requires a type-map to be supplied, to validate and act as a lookup for passed parameter-member.
        * Failed parameter-member check or invalid data-type will result in error message and function returns false
        * \param type_map       Type-Map to act as lookup and validation for parameter-member [std::map<std::string, TypeInfo>]
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * \return Function return: Successful: parameter type-name value [std::string] / Unsuccessful: false [bool]
        */
        template<typename TypeValue>
        static boost::optional<std::string> getParamTypeName(
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member,
            const boost::bimap<std::string, TypeValue>& type_map)
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

            // Acquired parameter-member as string-type
            std::string param_string = *result_cast;
        
            // Create a local map with case-insensitive comparator
            // (ignores capitalization of letters in given key [std::string])
            boost::bimap<std::string, TypeValue, Toolbox::Map::CaseInsensitiveComparator> type_map_ci;
            
            // Copy contents of given type-map into local map (case-insensitive)
            type_map_ci.insert(type_map.begin(), type_map.end());

            // Search for parameter-member in type-map
            boost::optional<std::string> result_search = searchTypeMapByName(type_map_ci, param_string);
            if(!result_search)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is NOT a valid type-name");

                // Function return
                return boost::none;
            }

            // Function return
            return param_string;
        } // Function end: getParamTypeName()


        // Get Parameter Data: Type-Info 
        // -------------------------------
        /** \brief Search for member in given parameter-data and returns parameter-value if successful.
        * Function checks if the supplied parameter-data contains the parameter-member. 
        * Data-type of the parameter-member is checked and gets converted to respecitve value-type if successful.
        * Function requires a type-map to be supplied, to validate and act as a lookup for passed parameter-member.
        * Failed parameter-member check or invalid data-type will result in error message and function returns false
        * \param type_map       Type-Map to act as lookup and validation for parameter-member [std::map<std::string, TypeInfo>]
        * \param param_xml      Parameter-data to be checked [XmlRpc::XmlRpcValue]
        * \param param_member   Member item to search for within parameter [std::string]
        * \return Function return: Successful: parameter type-info value [TypeValue] / Unsuccessful: false [bool]
        */
        template<typename TypeValue>
        static boost::optional<TypeValue> getParamTypeInfo(
            const XmlRpc::XmlRpcValue& param_xml,
            const std::string& param_member,
            const boost::bimap<std::string, TypeValue>& type_map)
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

            // Aqcuired parameter-member as string-type
            std::string param_string = *result_cast;
        
            // Create a local map with case-insensitive comparator
            // (ignores capitalization of letters in given key [std::string])
            boost::bimap<std::string, TypeValue, Toolbox::Map::CaseInsensitiveComparator> type_map_ci;
            
            // Copy contents of given type-map into local map (case-insensitive)
            type_map_ci.insert(type_map.begin(), type_map.end());

            // Search for parameter-member in type-map
            boost::optional<TypeValue> result_map = searchTypeMapByName(type_map_ci, param_string);
            if(!result_map)
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << param_member <<"] is NOT a valid type-name");

                // Function return
                return boost::none;
            }

            // Function return
            return result_map;
        } // Function end: getParamTypeInfo()


        // Convert Parameter Data to Bool-Value 
        // -------------------------------
        /** \brief Convert parameter data to bool-value
        * Parameter is checked for data-type and returns bool-value if successful.
        * Invalid data-type will result in error message and function return false
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * \return Function return: Successful: parameter value [bool] / Unsuccessful: false [bool]
        */
        static boost::optional<bool> castParamToBool(
            const XmlRpc::XmlRpcValue& param);

        
        // Convert Parameter Data to Int-Value 
        // -------------------------------
        /** \brief Convert parameter data to int-value
        * Parameter is checked for data-type and returns int-value if successful.
        * Invalid data-type will result in error message and function return false
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * \return Function return: Successful: parameter value [int] / Unsuccessful: false [bool]
        */
        static boost::optional<int> castParamToInt(
            const XmlRpc::XmlRpcValue& param);


        // Convert Parameter Data to Double-Value 
        // -------------------------------
        /** \brief Convert parameter data to double-value
        * Parameter is checked for data-type and returns double-value if successful.
        * Invalid data-type will result in error message and function return false
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * \return Function return: Successful: parameter value [double] / Unsuccessful: false [bool]
        */
        static boost::optional<double> castParamToDouble(
            const XmlRpc::XmlRpcValue& param);


        // Convert Parameter Data to String-Value 
        // -------------------------------
        /** \brief Convert parameter data to string-value
        * Parameter is checked for data-type and returns string-value if successful.
        * Invalid data-type will result in error message and function return false
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * \return Function return: Successful: parameter value [std::string] / Unsuccessful: false [bool]
        */
        static boost::optional<std::string> castParamToString(
            const XmlRpc::XmlRpcValue& param);


        // // Search Type-Map: Find by Type-Name
        // // -------------------------------
        // /** \brief Search for given type-name in supplied type-map. 
        // * If given type-name is found within the map, function returns the related type-name of the container-pair.
        // * If no search-item is found within the map, function returns false 
        // * \param type_map   Type-Map to search thorugh [std::map<std::string, typename TypeInfo>]
        // * \param type_name  Type-Name to search for (key) [std::string]
        // * \return Function return: Successful: type-name value [typename TypeInfo] / Unsuccessful: false [bool]
        // */
        // template<typename TypeInfo>
        // static boost::optional<TypeInfo> searchTypeMapByName(
        //     const std::map<std::string, TypeInfo>& type_map,
        //     const std::string& type_name)
        // {
        //     // Search for key type-name in map
        //     auto search = type_map.left.find(type_name);

        //     // Check if searched key is found in the container
        //     if(search == type_map.left.end())
        //     {
        //         // Map search failed! Type-Info key is NOT found in the container
        //         ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
        //             << ": Failed! Given type-name [" << type_name <<"] was NOT found in given type-map");
                
        //         // Function return
        //         return boost::none;
        //     }

        //     // Map search success! Type-Name key is found in the container
        //     // Return related Type-Info
        //     return search->second;
        // } // Function end: searchTypeMapByName()


        // // Search Type-Map: Find by Type-Name
        // // -------------------------------
        // /** \brief Search for given type-name in supplied type-map. 
        // * If given type-name is found within the map, function returns the related type-name of the container-pair.
        // * If no search-item is found within the map, function returns false 
        // * Supplied map contains operator 
        // * (typically CaseInsensitiveComparator used to ignore capitalization of letters in key-string)
        // * \param type_map   Type-Map to search thorugh [std::map<std::string, typename TypeInfo, Operator>]
        // * \param type_name  Type-Name to search for (key) [std::string]
        // * \return Function return: Successful: type-name value [typename TypeInfo] / Unsuccessful: false [bool]
        // */
        // template<typename TypeInfo>
        // static boost::optional<TypeInfo> searchTypeMapByName(
        //     const std::map<std::string, TypeInfo, Toolbox::Map::CaseInsensitiveComparator>& type_map,
        //     const std::string& type_name)
        // {
        //     // Search for key type-name in map
        //     auto search = type_map.left.find(type_name);

        //     // Check if searched key is found in the container
        //     if(search == type_map.left.end())
        //     {
        //         // Map search failed! Type-Info key is NOT found in the container
        //         ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
        //             << ": Failed! Given type-name [" << type_name <<"] was NOT found in given type-map");
                
        //         // Function return
        //         return boost::none;
        //     }

        //     // Map search success! Type-Name key is found in the container
        //     // Return related Type-Info
        //     return search->second;
        // } // Function end: searchTypeMapByName()


        // Search Type-Map: Find by Type-Name
        // -------------------------------
        /** \brief Search for given type-name in supplied type-map. 
        * If given type-name is found within the map, function returns the related type-name of the container-pair.
        * If no search-item is found within the map, function returns false 
        * \param type_map   Type-Map to search thorugh [boost::bimap<std::string, typename TypeInfo>]
        * \param type_name  Type-Name to search for (key) [std::string]
        * \return Function return: Successful: type-name value [typename TypeInfo] / Unsuccessful: false [bool]
        */
        template<typename TypeInfo>
        static boost::optional<TypeInfo> searchTypeMapByName(
            const boost::bimap<std::string, TypeInfo>& type_map,
            const std::string& type_name)
        {
            // Search for key type-name (left-element) in map
            auto search = type_map.left.find(type_name);

            // Check if searched key is found in the container
            if(search == type_map.left.end())
            {
                // Map search failed! Type-Info key (left-element) is NOT found in the container
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Given type-name [" << type_name <<"] was NOT found in given type-map");
                
                // Function return
                return boost::none;
            }

            // Map search success! Type-Name key (left-element) is found in the container
            // Return related Type-Info (right-element)
            return search->second;
        } // Function end: searchTypeMapByName()


        // Search Type-Map: Find by Type-Name
        // -------------------------------
        /** \brief Search for given type-name in supplied type-map. 
        * If given type-name is found within the map, function returns the related type-name of the container-pair.
        * If no search-item is found within the map, function returns false 
        * Supplied map contains operator 
        * (typically CaseInsensitiveComparator used to ignore capitalization of letters in key-string)
        * \param type_map   Type-Map to search thorugh [boost::bimap<std::string, typename TypeInfo, Operator>]
        * \param type_name  Type-Name to search for (key) [std::string]
        * \return Function return: Successful: type-name value [typename TypeInfo] / Unsuccessful: false [bool]
        */
        template<typename TypeInfo>
        static boost::optional<TypeInfo> searchTypeMapByName(
            const boost::bimap<std::string, TypeInfo, Toolbox::Map::CaseInsensitiveComparator>& type_map,
            const std::string& type_name)
        {
            // Search for key type-name (left-element) in map
            auto search = type_map.left.find(type_name);

            // Check if searched key is found in the container
            if(search == type_map.left.end())
            {
                // Map search failed! Type-Info key (left-element) is NOT found in the container
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Given type-name [" << type_name <<"] was NOT found in given type-map");
                
                // Function return
                return boost::none;
            }

            // Map search success! Type-Name key (left-element) is found in the container
            // Return related Type-Info (right-element)
            return search->second;
        } // Function end: searchTypeMapByName()


        // Search Type-Map: Find by Type-Info 
        // -------------------------------
        /** \brief Search for given type-info in supplied type-map. 
        * If given type-info is found within map, function returns the related type-name of the container-pair.
        * If no search-item is found within the map, function returns false 
        * \param type_map   Type-Map to search thorugh [boost::bimap<std::string, typename TypeInfo>]
        * \param type_name  Type-Info to search for (key) [typename TypeInfo]
        * \return Function return: Successful: type-name value [std::string] / Unsuccessful: false [bool]
        */
        template<typename TypeInfo>
        static boost::optional<std::string> searchTypeMapByType(
            const boost::bimap<std::string, TypeInfo>& type_map,
            const TypeInfo& type_info)
        {
            // Search for key type-info (right-element) in map
            auto search = type_map.right.find(type_info);

            // Check if searched key is found in the container
            if(search == type_map.right.end())
            {
                // Map search failed! Type-Info key (right-element) is NOT found in the container
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Given type-info [" << type_info <<"] was NOT found in given type-map");
                
                // Function return
                return boost::none;
            }

            // Map search success! Type-Info key (right-element) is found in the container
            // Return related Type-Name (left-element)
            return search->second;
        } // Function end: searchTypeMapByType()


    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

        // Get Name of Parameter Data-Type 
        // -------------------------------
        /** \brief Get and converts Parameter Data-Type to a readable type-name
        * \param type       Data-type to compare parameter against [XmlRpc::XmlRpcValue::Type]
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

};  // End Class: Parameters
} // End Namespace: Robotics Toolbox
#endif // PARAMETER_TOOL_H