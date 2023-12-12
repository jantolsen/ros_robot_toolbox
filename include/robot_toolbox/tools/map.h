// Robotics Toolbox - Map Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Map Tools contains helper and utility functions 
//      useful for Robotics applications
//
// Version:
//  0.1 - Initial Version
//        [11.12.2023]  -   Jan T. Olsen
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
#ifndef MAP_TOOL_H       
#define MAP_TOOL_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <vector>
    #include <map>

    // Boost
    #include <boost/bimap.hpp>

    // Ros
    #include <ros/ros.h>

    // Toolbox
    #include "robot_toolbox/tools/common.h"
    #include "robot_toolbox/tools/convert.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{

// Map Tool Class
// -------------------------------
class Map
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:
        
        // Case-Insensitive Comparator
        // (useful for maps with [std::string] as keys)
        struct CaseInsensitiveComparator
        {
            // Operator to ignore lower- and upper case differences
            bool operator()(const std::string& a, const std::string& b) const noexcept
            {
                // Compare strings
                int result = strcasecmp(a.c_str(), b.c_str());

                // Function return
                return (result < 0);
            }
        };


        // Search Map (std::map)
        // -------------------------------
        // (Template specialization: Valid SearchType)
        /** \brief Search for given search-item in supplied map. 
        * If search-item is found, function returns the related the item in container-pair.
        * For scenerarios where the search-item is not found, function return unsuccessful (false)
        * \param map                Map to search thorugh [std::map<typename MapKey, typename MapValue>]
        * \param search_item        Item to search for [typename SearchType]
        * \param case_insensitive   Ignore capitalization of letters in string-key (enabled as default) [bool]
        * \return Function result: Successful/unsuccessful (resulting item/false) [boost::optional<typename ResultType>]
        */
        template<typename SearchType, typename ResultType, typename MapKey, typename MapValue>
        typename std::enable_if<std::is_same<SearchType, MapKey>::value, boost::optional<ResultType>>::type
        static searchMap(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& map,
            const bool& case_insensitive=true)
        {
            // Check for case-insensitive flag
            if(case_insensitive)
            {
                // Create a new map instance with case-insensitive comparator
                std::map<MapKey, MapValue, CaseInsensitiveComparator> map_case_insensitive;

                // Copy given map elements to case-insensitive map
                map_case_insensitive.insert(map.begin(), map.end());

                // Search for key in map
                auto search = map_case_insensitive.find(search_item);

                // Check if searched key is found in the container
                if(search != map_case_insensitive.end())
                {
                    // Related item found for given key in map
                    ResultType result_item = search->second;

                    // Function return
                    return result_item;
                }
            }
            // Non case-insensitive flag
            else
            {
                // Search for key in map
                auto search = map.find(search_item);

                // Check if searched key is found in the container
                if(search != map.end())
                {
                    // Related item found for given key in map
                    ResultType result_item = search->second;

                    // Function return
                    return result_item;
                }
            }

            // No value was found in the container
            // (iterator has reached the end of the container)

            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given Search-Item: [" << search_item << "] was NOT found in given map");

            // Function return
            return boost::none;
        } // Function-End: searchMap()


        // Search Map (std::map)
        // -------------------------------
        // (Template specialization: Invalid SearchType)
        /** \brief Search for given search-item in supplied map. 
        * If search-item is found, function returns the related the item in container-pair.
        * For scenerarios where the search-item is not found, function return unsuccessful (false)
        * \param map            Map to search thorugh [std::map<typename MapKey, typename MapValue>]
        * \param search_item    Item to search for [typename SearchType]
        * \return Function result: Successful/unsuccessful (resulting item/false) [boost::optional<typename ResultType>]
        */
        template<typename SearchType, typename ResultType, typename MapKey, typename MapValue>
        typename std::enable_if<(not std::is_same<SearchType, MapKey>::value), boost::optional<ResultType>>::type
        static searchMap(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& map,
            const bool& case_insensitive=true)
        {
            // Search-Item type doesn't match Map-Key type

            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Data-Type of Search-Item: [" << typeid(search_item).name() << "]"
                << " does NOT match the data-type of the Map's Key-type [" << typeid(MapKey).name() << "]");

            // Function return
            return boost::none;
        } // Function-End: searchMap()


    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

        // Search std-map
        template<typename SearchType, typename MapKey, typename MapValue>
        typename std::enable_if<std::is_same<SearchType, MapKey>::value, bool>::type
        static searchMap(
            const SearchType& search_item,
            const std::map<MapKey, MapValue>& map)
        {
            // Search for key in map
            auto search = map.find(search_item);

            // Check if searched key is found in the container
            if(search != map.end())
            {
                // Key is found in the container
                return true;
            }

            // Key is NOT found in the container
            return false;
        }

        // Search std-map
        template<typename SearchType, typename MapKey, typename MapValue>
        typename std::enable_if<(not std::is_same<SearchType, MapKey>::value), bool>::type
        static searchMap(
            const SearchType& search_item,
            const std::map<MapKey, MapValue>& map)
        {
            // Search-item type does not equal map-Key type
            return false;
        }


        // Search bi-map
        template<typename SearchType, typename MapKey, typename MapValue>
        typename std::enable_if<std::is_same<SearchType, MapKey>::value, bool>::type
        static searchMap(
            const SearchType& search_item,
            const boost::bimap<MapKey, MapValue>& map)
        {
            // Search for left-key in map
            auto search = map.left.find(search_item);

            // Check if searched key is found in the container
            if(search != map.left.end())
            {
                // Key is found in the container
                return true;
            }

            // Key is NOT found in the container
            return false;
        }

        // Search bi-map
        template<typename SearchType, typename MapKey, typename MapValue>
        typename std::enable_if<std::is_same<SearchType, MapValue>::value, bool>::type
        static searchMap(
            const SearchType& search_item,
            const boost::bimap<MapKey, MapValue>& map)
        {
            // Search for right-key in map
            auto search = map.right.find(search_item);

            // Check if searched key is found in the container
            if(search != map.right.end())
            {
                // Key is found in the container
                return true;
            }

            // Key is NOT found in the container
            return false;
        }

        // Search bi-map
        template<typename SearchType, typename MapKey, typename MapValue>
        typename std::enable_if<(not std::is_same<SearchType, MapKey>::value) and (not std::is_same<SearchType, MapValue>::value), bool>::type
        static searchMap(
            const SearchType& search_item,
            const boost::bimap<MapKey, MapValue>& map)
        {
            // Search-item type does not equal map-key type not map-value type
            return false;
        }


    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:

        // Prefix message for class
        static const std::string CLASS_PREFIX;


}; // End Class: Common
} // End Namespace: Robotics Toolbox
#endif // MAP_TOOL_H 