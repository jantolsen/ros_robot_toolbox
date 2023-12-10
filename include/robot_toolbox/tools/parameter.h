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


        // Search Type-Map 
        // -------------------------------
        // (Function Overloading)
        /** \brief Search and find the relating type-name/type-id of on the given type-name/type-id
        * by searching through the supplied type-map
        * \param search_item    Item to search for (Type-Name or Type-ID) [typename SearchType]
        * \param type_map       Type-Map to search thorugh [std::map<typename MapKey, typename MapValue>]
        * \param result_item    Resulting item (Type-Name or Type-ID) [typename ResultType]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename SearchType, typename ResultType, typename MapKey, typename MapValue>
        static bool searchTypeMap(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& type_map,
            ResultType& result_item)
        {
            // Call Common::mapSearch()
            if(!Common::mapSearch(search_item, type_map, result_item))
            {
                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Given Search-item: [" << search_item << "] was NOT found in Type-Map");

                // Function return
                return false;
            }

            // Function return
            return true;
        } // Function-End: searchTypeMap()


        // Check Type Item
        // -------------------------------
        /** \brief Search and find type-item parameter from parameter-server
        * This involves checking the parameter-server for the given parameter-member
        * and searching for the type-item in the supplied type-map.
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * \param member     Member item to search for within parameter [std::string]
        * \param type_map   Type-Map to search thorugh [std::map<typename MapKey, typename MapValue>]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename MapKey, typename MapValue>
        static bool checkTypeItem(
            const XmlRpc::XmlRpcValue& param, 
            const std::string& member,
            const std::map<MapKey, MapValue>& type_map)
        {
            // Local variable(s)
            int param_member_int;
            std::string param_member_str;

            // Check for parameter on parameter-server
            if(!checkMember(param, member))
            {
                // Parameter get/validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    <<  ": Failed! Parameter [" << member << "] is missing");

                // Function return
                return false;
            }

            // Check for parameter type-item in type-map
            // Get Parameter data-type
            switch (param.getType())
            {
                // Integer:
                // (Valid Data-type for type-map search)
                case XmlRpc::XmlRpcValue::TypeInt:
                    // Cast parameter-member to int
                    param_member_int = static_cast<int>(param[member]);

                    // Search for type-item in type-map
                    if(!Common::mapCheckItem(param_member_int, type_map))
                    {
                        // Parameter map-search failed
                        ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                            <<  ": Failed! Parameter [" << member << "] is NOT a part of valid types in related type-map");

                        // Function return
                        return false;
                    }

                    // Function return
                    return true;

                // String:
                // (Valid Data-type for type-map search)
                case XmlRpc::XmlRpcValue::TypeString:
                    // Cast parameter-member to string
                    param_member_str = static_cast<std::string>(param[member]);

                    // Search for type-item in type-map
                    if(!Common::mapCheckItem(param_member_str, type_map))
                    {
                        // Parameter map-search failed
                        ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                            <<  ": Failed! Parameter [" << member << "] is NOT a part of valid types in related type-map");

                        // Function return
                        return false;
                    }

                    // Function return
                    return true;

                // Default:
                // (Invalid data-type for type-map search)
                default:
                    // Report to terminal
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        << ": Failed! Parameter data-type [" << getParamTypeName(param.getType()) << "] does NOT fit for type-map search");

                    // Function return
                    return false;
            }
        } // Function-End: checkTypeItem()


        // Get Type Item
        // -------------------------------
        // (Function Overloading)
        /** \brief Search and find type-item parameter from parameter-server
        * This involves checking the parameter-server for the given parameter-member
        * and searching for the type-item in the supplied type-map.
        * Function returns corresponding type-item [int] and type-item-name [std::string]
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * \param member     Member item to search for within parameter [std::string]
        * \param type_map   Type-Map to search thorugh [std::map<typename MapKey, typename MapValue>]
        * \param result_item        Resulting type-item [int]
        * \param result_item_name   Resulting type-item-name [std::string]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename MapKey, typename MapValue>
        static bool getTypeItem(
            const XmlRpc::XmlRpcValue& param, 
            const std::string& member,
            const std::map<MapKey, MapValue>& type_map,
            int& result_item,
            std::string& result_item_name)
        {
            // Local variable(s)
            int param_member_int;
            std::string param_member_str;
            int search_result_int;
            std::string search_result_str;
            
            // Check type-item parameter
            if(!checkTypeItem(param, member, type_map))
            {
                // Function return
                return false;
            }

            // Get Parameter data-type and search for type-item in type-map
            switch (param.getType())
            {
                // Integer:
                // (Valid Data-type for type-map search)
                case XmlRpc::XmlRpcValue::TypeInt:
                    // Cast parameter-member to int
                    param_member_int = static_cast<int>(param[member]);

                    // Search for type-item in type-map
                    if(searchTypeMap(param_member_int, type_map, search_result_str))
                    {
                        // Type-item is found in type-map!
                        // Update results
                        result_item = param_member_int;
                        result_item_name = search_result_str;
    
                        // Function return
                        return true;
                    }

                    // Parameter map-search failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Failed! Parameter [" << member << "] is NOT a part of valid types in related type-map");

                    // Function return
                    return false;

                // String:
                // (Valid Data-type for type-map search)
                case XmlRpc::XmlRpcValue::TypeString:
                    // Cast parameter-member to string
                    param_member_str = static_cast<std::string>(param[member]);

                    // Search for type-item in type-map
                    if(searchTypeMap(param_member_str, type_map, search_result_int))
                    {
                        // Type-item is found in type-map!
                        // Update results
                        result_item = search_result_int;
                        result_item_name = param_member_str;

                        // Function return
                        return true;
                    }

                    // Parameter map-search failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Failed! Parameter [" << member << "] is NOT a part of valid types in related type-map");

                    // Function return
                    return false;

                // Default:
                // (Invalid data-type for type-map search)
                default:
                    // Report to terminal
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        << ": Failed! Parameter data-type [" << getParamTypeName(param.getType()) << "] does NOT fit for type-map search");

                    // Function return
                    return false;
            }
        } // Function-End: getTypeItem()


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