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

    // Ros
    #include <ros/ros.h>

    // Ros Messages
    #include "sensor_msgs/JointState.h"

    // Robotics Toolbox
    #include "robot_toolbox/tools/common.h"
    

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
        static bool checkType(
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


        // Search Type-Map 
        // -------------------------------
        // (Function Overloading)
        /** \brief Search and find the relating type-name/type-id of on the given type-name/type-id
        * by searching through the supplied type-map
        * \param search_item    Item to to search for (Type-Name or Type-ID) [typename SearchType]
        * \param type_map       Type-Map to search thorugh [std::map<typename Key, typename Value>]
        * \param result_item    Resulting item (Type-Name or Type-ID) [typename ResultType]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename SearchType, typename ResultType, typename MapKey, typename MapValue>
        static bool searchTypeMap(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& type_map,
            ResultType& result_item)
        {
            // Compare search-item type vs map-key type
            if constexpr (std::is_same<SearchType, MapKey>::value)
            {
                // Call MapGetValue()
                // (using search-item as key to search for related value)
                return Common::mapGetValue(search_item, type_map, result_item);
            }
            // Compare search-item type vs map-value type
            else if constexpr (std::is_same<SearchType, MapValue>::value)
            {
                // Call MapGetKey()
                // (using search-item as value to search for related key)
                return Common::mapGetKey(search_item, type_map, result_item);
            }

            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << " Failed! Data-Type of Search-Item: [" << search_item << "]" 
                << " does NOT match the data-type of either the Type-Map's Key nor Value");

            // Function return
            return false;
        } // Function-End: searchTypeMap()


        // // Search Type-Map 
        // // (Find Type-Name by using Type-ID)
        // // -------------------------------
        // // (Function Overloading)
        // /** \brief Search for type-id in supplied type-map and to find respective type-name
        // * \param type_id    Type-ID Parameter [int]
        // * \param type_map   Type-Map to search through [std::map<std::string, enum>]
        // * \param type_name  Type-Name Paramter [std::string]
        // * \param err_print  Print potential error-message to terminal (disabled at default) [bool]
        // * \return Function result: Successful/unsuccessful (true/false)
        // */
        // template<typename Enum, typename Operator>
        // static bool searchTypeMap(
        //     const int& type_id, 
        //     const std::map<std::string, Enum, Operator>& type_map,
        //     std::string& type_name)
        // {
        //     // Iterate through supplied map
        //     for(auto const& it : type_map)
        //     {
        //         // Compare iterator-value against supplied type-id
        //         if(static_cast<int>(it.second) == type_id)
        //         {
        //             // Set Type-Name equal to map-key
        //             type_name = it.first;

        //             // Function return
        //             return true;
        //         } 
        //     }
        //     // Report to terminal
        //     ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
        //         << " Failed! Type-ID: [" << type_id << "] was NOT found in given Type-Map");

        //     // Function return
        //     return false;
        // } // Function-End: searchTypeMap()


        // // Search Type-Map 
        // // (Find Type-ID by using Type-Name)
        // // -------------------------------
        // /** \brief Search for type-name in supplied type-map to find respective type-id
        // * \param type_name  Type-Name Paramter [std::string]
        // * \param type_map   Type-Map to search through [std::map<std::string, enum>]
        // * \param type_id    Type-ID Parameter [int]
        // * \param err_print  Print potential error-message to terminal (disabled at default) [bool]
        // * \return Function result: Successful/unsuccessful (true/false)
        // */
        // template<typename Enum, typename Operator>
        // static bool searchTypeMap(
        //     const std::string& type_name, 
        //     const std::map<std::string, Enum, Operator>& type_map,
        //     int& type_id)
        // {
        //     // Search through type-map using type-name as key
        //     auto search = type_map.find(type_name);

        //     // Check if searched element is found in the container
        //     if(search != type_map.end())
        //     {
        //         // Set Type-ID equal to map-value
        //         type_id = search->second;

        //         // Function return
        //         return true;
        //     }
        //     // No element was found in the container
        //     // (iterator has reached the end of the container)

        //     // Report error to terminal
        //     ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
        //         <<  ": Failed! Type-Name: [" << type_name << "] was NOT found in given Type-Map");

        //     // Function return
        //     return false;
        // } // Function-End: searchTypeMap()


    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

        // Get Name of Parameter-Type 
        // -------------------------------
        /** \brief Converts Parameter-Type to a readable type-name
        * \param type       Data-type to compare parameter against [XmlRpc::XmlRpcValue::Type]
        * \return Name of parameter-type represented [std::string]
        */
        static std::string getParamTypeName(
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