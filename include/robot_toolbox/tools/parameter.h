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


        // Search Type-Identifier Map 
        // -------------------------------
        /** \brief Search for the given type-identifier (type-name/type-id) in the supplied type-identifer-map,
        * to find the related type-identifier pair (type-name/type-id)
        * \param search_item    Identifier item to search for (Type-Name or Type-ID) [typename SearchType]
        * \param type_map       Type-Map to search thorugh [std::map<typename MapKey, typename MapValue>]
        * \param result_item    Resulting item (Type-Name or Type-ID) [typename ResultType]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename SearchType, typename ResultType, typename MapKey, typename MapValue>
        static bool searchTypeIdentifierMap(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& type_map,
            ResultType& result_item)
        {
            // Call Common::mapSearch()
            if(!Common::mapSearch(search_item, type_map, result_item))
            {
                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Given Search-Item: [" << search_item << "]"
                    << " was NOT found in Type-Identifier-Map");

                // Function return
                return false;
            }

            // Function return
            return true;
        } // Function-End: searchTypeIdentifierMap()


        // Check Type-Identifier
        // -------------------------------
        /** \brief Check if the given type-identifier (type-name/type-id) parameter 
        * is present on the parameter-server. Also checks if the given type-identifier 
        * (type-name/type-id) exists in the supplied type-identifer-map
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * \param member     Member item to search for within parameter [std::string]
        * \param type_map   Type-Map to search thorugh [std::map<typename MapKey, typename MapValue>]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename MapKey, typename MapValue>
        static bool checkTypeIdentifier(
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
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter [" << member << "] is missing");

                // Function return
                return false;
            }

            // Get parameter data-type and search for type-identifier in type-identifier-map
            switch (param[member].getType())
            {
                // Integer:
                // (Valid Data-type for type-identifier-map search)
                case XmlRpc::XmlRpcValue::TypeInt:
                    // Cast parameter-member to int
                    param_member_int = static_cast<int>(param[member]);

                    // Search for type-identifier in type-identifier-map
                    if(!Common::mapContains(param_member_int, type_map))
                    {
                        // Type-identifier map-search failed
                        ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                            << ": Failed! Parameter [" << member << "]" 
                            << " is NOT a part of valid types in related Type-Identifier-Map");

                        // Function return
                        return false;
                    }

                    // Function return
                    return true;

                // String:
                // (Valid Data-type for type-identifier-map search)
                case XmlRpc::XmlRpcValue::TypeString:
                    // Cast parameter-member to string
                    param_member_str = static_cast<std::string>(param[member]);

                    // Search for type-identifier in type-identifier-map
                    if(!Common::mapContains(param_member_str, type_map))
                    {
                        // Type-identifier map-search failed
                        ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                            << ": Failed! Parameter [" << member << "]"
                            << " is NOT a part of valid types in related Type-Identifier-Map");

                        // Function return
                        return false;
                    }

                    // Function return
                    return true;

                // Default:
                // (Invalid data-type for type-identifier-map search)
                default:
                    // Report to terminal
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        << ": Failed! Parameter data-type [" << getDataTypeName(param.getType()) << "]"
                        << " does NOT fit for Type-Identifier-Map search");

                    // Function return
                    return false;
            }
        } // Function-End: checkIdentifier()


        // Get Type-Identifier
        // -------------------------------
        /** \brief Search for the type-identifier (type-name/type-id) parameter on the parameter-server
        * Also search for the given type-identifier (type-name/type-id) in the supplied type-identifer-map
        * to find the related type-identifier pair (type-name/type-id)
        * Function returns corresponding type-item [int] and type-item-name [std::string]
        * \param param      Parameter to be checked [XmlRpc::XmlRpcValue]
        * \param member     Member item to search for within parameter [std::string]
        * \param type_map   Type-Map to search thorugh [std::map<typename MapKey, typename MapValue>]
        * \param result_item        Resulting type-item [int]
        * \param result_item_name   Resulting type-item-name [std::string]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename MapKey, typename MapValue>
        static bool getTypeIdentifier(
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
            
            // Check Type-Identifier
            if(!checkTypeIdentifier(param, member, type_map))
            {
                // Function return
                return false;
            }

            // Get parameter data-type and search for type-identifier in type-identifier-map
            switch (param[member].getType())
            {
                // Integer:
                // (Valid Data-type for type-identifier-map search)
                case XmlRpc::XmlRpcValue::TypeInt:
                    // Cast parameter-member to int
                    param_member_int = static_cast<int>(param[member]);

                    // Search for type-identifier in type-identifier-map
                    if(searchTypeIdentifierMap(param_member_int, type_map, search_result_str))
                    {
                        // Type-identifier is found in type-identifier-map!
                        // Update results
                        result_item = param_member_int;
                        result_item_name = search_result_str;
    
                        // Function return
                        return true;
                    }

                    // Type-identifier map-search failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        << ": Failed! Parameter [" << member << "]"
                        << " is NOT a part of valid types in related Type-Identifier-Map");

                    // Function return
                    return false;

                // String:
                // (Valid Data-type for type-identifier-map search)
                case XmlRpc::XmlRpcValue::TypeString:
                    // Cast parameter-member to string
                    param_member_str = static_cast<std::string>(param[member]);

                    // Search for type-identifier in type-identifier-map
                    if(searchTypeIdentifierMap(param_member_str, type_map, search_result_int))
                    {
                        // Type-identifier is found in type-identifier-map!
                        // Update results
                        result_item = search_result_int;
                        result_item_name = param_member_str;
                        
                        // Function return
                        return true;
                    }

                    // Type-identifier-map search failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        << ": Failed! Parameter [" << member << "]"
                        << " is NOT a part of valid types in related Type-Identifier-Map");

                    // Function return
                    return false;

                // Default:
                // (Invalid data-type for type-identifier-map search)
                default:
                    // Report to terminal
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        << ": Failed! Parameter data-type [" << getDataTypeName(param.getType()) << "]"
                        << " does NOT fit for Type-Identifier-Map search");

                    // Function return
                    return false;
            }
        } // Function-End: getTypeIdentifier()


    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

        // Get Name of Parameter Data-Type 
        // -------------------------------
        /** \brief Converts Parameter Data-Type to a readable type-name
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