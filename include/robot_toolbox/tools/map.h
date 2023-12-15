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
    #include <boost/optional.hpp>

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
        
        // Search Map by Key:
        // (std::map)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given key in supplied map. 
        * If given key is found within the map, function returns the related value of the container-pair.
        * If no key is found within the map, function returns false.
        * 
        * Map search will ignore capitalization of letters in key-string.
        * \param map    Map to search thorugh [std::map<typename Key, typename Value>]
        * \param key    Key to search for [typename Key]
        * 
        * \return Function return: Successful: value [typename Value] / Unsuccessful: false [bool]
        */
        template<typename Key, typename Value>
        static boost::optional<Value> searchMapByKey(
            const std::map<Key, Value>& map,
            const Key& key)
        {
            // Call overloading function
            return searchStdMapByKey(map, key);
        } // Function-End: searchMapByKey()


        // Search Map by Key:
        // (boost::bimap)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given key in supplied map. 
        * If given key is found within the map, function returns the related value of the container-pair.
        * If no key is found within the map, function returns false.
        * 
        * Map search will ignore capitalization of letters in key-string.
        * \param map    Map to search thorugh [boost::bimap<typename Key, typename Value>]
        * \param key    Key to search for [typename Key]
        * 
        * \return Function return: Successful: value [typename Value] / Unsuccessful: false [bool]
        */
        template<typename Key, typename Value>
        static boost::optional<Value> searchMapByKey(
            const boost::bimap<Key, Value>& map,
            const Key& key)
        {
            // Call overloading function
            return searchBiMapByKey(map, key);
        } // Function-End: searchMapByKey()


        // Search Map by Value:
        // -------------------------------
        // (std::map)
        // (Function Overloading)
        /** \brief Search for given value in supplied map. 
        * 
        * If given value is found within the map, function returns the related key of the container-pair.
        * If no value is found within the map, function returns false. 
        * Map search will ignore capitalization of letters in key-string.
        * 
        * \param map    Map to search thorugh [std::map<typename Key, typename Value>]
        * \param value  Value to search for [typename Value]
        * 
        * \return Function return: Successful: value [typename Key] / Unsuccessful: false [bool]
        */
        template<typename Key, typename Value>
        static boost::optional<Key> searchMapByValue(
            const std::map<Key, Value>& map,
            const Value& value)
        {
            // Call overloading function
            return searchStdMapByValue(map, value);
        } // Function-End: searchMapByValue()


        // Search Map by Value:
        // -------------------------------
        // (boost::bimap)
        // (Function Overloading)
        /** \brief Search for given value in supplied map.
        *  
        * If given value is found within the map, function returns the related key of the container-pair.
        * If no value is found within the map, function returns false.
        * Map search will ignore capitalization of letters in key-string.
        * 
        * \param map    Map to search thorugh [boost::bimap<typename Key, typename Value>]
        * \param value  Value to search for [typename Value]
        * 
        * \return Function return: Successful: value [typename Key] / Unsuccessful: false [bool]
        */
        template<typename Key, typename Value>
        static boost::optional<Key> searchMapByValue(
            const boost::bimap<Key, Value>& map,
            const Value& value)
        {
            // Call overloading function
            return searchBiMapByValue(map, value);
        } // Function-End: searchMapByValue()


    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:
        
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


        // Search Std-Map: by Key:
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given key in supplied map. 
        * 
        * If given key is found within the map, function returns the related value of the container-pair.
        * If no key is found within the map, function returns false 
        * 
        * \param map    Map to search thorugh [std::map<typename Key, typename Value>]
        * \param key    Key to search for [typename Key]
        * 
        * \return Function return: Successful: value [typename Value] / Unsuccessful: false [bool]
        */
        template<typename Key, typename Value>
        static boost::optional<Value> searchStdMapByKey(
            const std::map<Key, Value>& map,
            const Key& key)
        {
            // Search for key given in map
            auto search = map.find(key);

            // Check if searched key is found in the container
            if(search == map.end())
            {
                // Map search failed! Key is NOT found in the container
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Search key [" << key <<"] was NOT found in given map");
                
                // Function return
                return boost::none;
            }

            // Map search success! Key is found in the container
            // Return related Value
            return search->second;
        } // Function-End: searchStdMapByKey()


        // Search Std-Map by Key: 
        // (string-key and CaseInsensitiveComparator)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given key in supplied map.
        * 
        * Map search will ignore capitalization of letters in key-string.
        * If given key is found within the map, function returns the related value of the container-pair.
        * If no key is found within the map, function returns false.
        * 
        * \param map    Map to search thorugh [std::map<std::string, typename Value>]
        * \param key    Key to search for [std::string]
        * 
        * \return Function return: Successful: value [typename Value] / Unsuccessful: false [bool]
        */
        template<typename Value>
        static boost::optional<Value> searchStdMapByKey(
            const std::map<std::string, Value>& map,
            const std::string& key)
        {
            // Create a new map instance with case-insensitive comparator
            std::map<std::string, Value, CaseInsensitiveComparator> map_ci;

            // Copy given map elements to new map
            map_ci.insert(map.begin(), map.end());

            // Search for key given in map
            auto search = map_ci.find(key);

            // Check if searched key is found in the container
            if(search == map_ci.end())
            {
                // Map search failed! Key is NOT found in the container
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Search key [" << key <<"] was NOT found in given map");
                
                // Function return
                return boost::none;
            }

            // Map search success! Key is found in the container
            // Return related Value
            return search->second;
        } // Function-End: searchStdMapByKey()


        // Search Boost-BiMap by Key:
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given key in supplied map. 
        * If given key is found within the map, function returns the related value of the container-pair.
        * If no key is found within the map, function returns false 
        * 
        * \param map    Map to search thorugh [boost::bimap<typename Key, typename Value>]
        * \param key    Key to search for [typename Key]
        * 
        * \return Function return: Successful: value [typename Value] / Unsuccessful: false [bool]
        */
        template<typename Key, typename Value>
        static boost::optional<Value> searchBiMapByKey(
            const boost::bimap<Key, Value>& map,
            const Key& key)
        {
            // Search for key (left-element) given in map
            auto search = map.left.find(key);

            // Check if searched key (left-element) is found in the container
            if(search == map.left.end())
            {
                // Map search failed! Key is NOT found in the container
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Search key [" << key <<"] was NOT found in given map");
                
                // Function return
                return boost::none;
            }

            // Map search success! Key (left-element) is found in the container
            // Return related Value (right-element)
            return search->second;
        } // Function-End: searchBiMapByKey()


        // Search Boost-BiMap by Key: 
        // (string-key)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given key in supplied map. 
        * Map search will ignore capitalization of letters in key-string.
        * If given key is found within the map, function returns the related value of the container-pair.
        * If no key is found within the map, function returns false.
        * 
        * \param map    Map to search thorugh [boost::bimap<std::string, typename Value>]
        * \param key    Key to search for [std::string]
        * 
        * \return Function return: Successful: value [typename Value] / Unsuccessful: false [bool]
        */
        template<typename Value>
        static boost::optional<Value> searchBiMapByKey(
            const boost::bimap<std::string, Value>& map,
            const std::string& key)
        {
            // Create a new map instance with case-insensitive comparator
            boost::bimap<
                boost::bimaps::set_of<std::string, CaseInsensitiveComparator>, 
                boost::bimaps::set_of<Value>
            > map_ci;

            // Copy given map elements to new map
            for (const auto& pair : map)
            {
                map_ci.insert({pair.left, pair.right});
            }

            // Search for key (left-element) given in map
            auto search = map_ci.left.find(key);

            // Check if searched key (left-element) is found in the container
            if(search == map_ci.left.end())
            {
                // Map search failed! Key is NOT found in the container
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Search key [" << key <<"] was NOT found in given map");
                
                // Function return
                return boost::none;
            }

            // Map search success! Key (left-element) is found in the container
            // Return related Value (right-element)
            return search->second;
        } // Function-End: searchBiMapByKey()


        // Search Std-Map by Value:
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given value in supplied map. 
        * If given value is found within the map, function returns the related key of the container-pair.
        * If no value is found within the map, function returns false 
        * 
        * \param map    Map to search thorugh [std::map<typename Key, typename Value>]
        * \param value  Value to search for [typename Value]
        * 
        * \return Function return: Successful: value [typename Key] / Unsuccessful: false [bool]
        */
        template<typename Key, typename Value>
        static boost::optional<Key> searchStdMapByValue(
            const std::map<Key, Value>& map,
            const Value& value)
        {
            // Iterate through given map
            for(auto const& it : map)
            {
                // Compare iterator-value against supplied value
                if(it.second == value)
                {
                    // Map search success! Value is found in the container
                    // Return related Key
                    return it.first;
                }
                // Continue iteration
            }

            // Map search failed! Value is NOT found in the container
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Search value [" << value <<"] was NOT found in given map");
                
            // Function return
            return boost::none;
        } // Function-End: searchStdMapByValue()


        // Search Std-Map by Value: 
        // (string-value)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given value in supplied map. 
        * Map search will ignore capitalization of letters in value-string.
        * If given value is found within the map, function returns the related key of the container-pair.
        * If no value is found within the map, function returns false 
        * 
        * \param map    Map to search thorugh [std::map<std::string, typename Value>]
        * \param key    Value to search for [std::string]
        * 
        * \return Function return: Successful: value [typename Value] / Unsuccessful: false [bool]
        */
        template<typename Key>
        static boost::optional<Key> searchStdMapByValue(
            const std::map<Key, std::string>& map,
            const std::string& value)
        {
            // Iterate through supplied map
            for(auto const& it : map)
            {
                // Compare iterator-value against supplied value
                // (Case-Insensitivity)
                if(strcasecmp(it.second.c_str(), value.c_str()) == 0)
                {
                    // Map search success! Value is found in the container
                    // Return related Key
                    return it.first;
                }
                // Continue iteration
            }

            // Map search failed! Value is NOT found in the container
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Search value [" << value <<"] was NOT found in given map");
                
            // Function return
            return boost::none;
        } // Function-End: searchStdMapByValue()


        // Search Boost-Bi-Map by Value:
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given value in supplied map. 
        * If given value is found within the map, function returns the related key of the container-pair.
        * If no value is found within the map, function returns false 
        * 
        * \param map    Map to search thorugh [boost::bimap<typename Key, typename Value>]
        * \param value  Value to search for [typename Value]
        * 
        * \return Function return: Successful: value [typename Key] / Unsuccessful: false [bool]
        */
        template<typename Key, typename Value>
        static boost::optional<Key> searchBiMapByValue(
            const boost::bimap<Key, Value>& map,
            const Value& value)
        {
            // Search for value (right-element) given in map
            auto search = map.right.find(value);

            // Check if searched value (right-element) is found in the container
            if(search == map.right.end())
            {
                // Map search failed! Value is NOT found in the container
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Search value [" << value <<"] was NOT found in given map");
                
                // Function return
                return boost::none;
            }

            // Map search success! Value (right-element) is found in the container
            // Return related Key (left-element)
            return search->second;
        } // Function-End: searchBiMapByValue()


        // Search Boost-Bi-Map by Value: 
        // (string-value)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given value in supplied map. 
        * Map search will ignore capitalization of letters in value-string.
        * If given value is found within the map, function returns the related key of the container-pair.
        * If no value is found within the map, function returns false 
        * 
        * \param map    Map to search thorugh [std::map<typename Key, std::string>]
        * \param key    Value to search for [std::string]
        * 
        * \return Function return: Successful: value [typename Key] / Unsuccessful: false [bool]
        */
        template<typename Key>
        static boost::optional<Key> searchBiMapByValue(
            const boost::bimap<Key, std::string>& map,
            const std::string& value)
        {
            // Create a new map instance with case-insensitive comparator
            boost::bimap<
                boost::bimaps::set_of<Key>, 
                boost::bimaps::set_of<std::string, CaseInsensitiveComparator>
            > map_ci;

            // Copy given map elements to new map
            for (const auto& pair : map)
            {
                map_ci.insert({pair.left, pair.right});
            }

            // Search for value (right-element) given in map
            auto search = map_ci.right.find(value);

            // Check if searched value (right-element) is found in the container
            if(search == map_ci.right.end())
            {
                // Map search failed! Value is NOT found in the container
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Search value [" << value <<"] was NOT found in given map");
                
                // Function return
                return boost::none;
            }

            // Map search success! Value (right-element) is found in the container
            // Return related Key (left-element)
            return search->second;
        } // Function-End: searchMapByValue()


    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:

        // Prefix message for class
        static const std::string CLASS_PREFIX;


}; // End Class: Common
} // End Namespace: Robotics Toolbox
#endif // MAP_TOOL_H 