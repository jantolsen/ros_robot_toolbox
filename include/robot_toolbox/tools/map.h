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
        
        // Search Map by Key
        // (Case: std:map)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given key in supplied map. 
        *
        * If given key is found within the map, function returns the related value of the container-pair.
        * If no key is found within the map, function returns false.
        * Map search will ignore capitalization of letters in search-string.
        *
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
            // Search map by key
            boost::optional<Value> result = searchStdMapByKey(map, key);
            if(!result)
            {
                // Map search failed! Key is NOT found in the map
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Search key [" << key <<"] was NOT found in given map");

                // Function return
                return boost::none;
            } 
        
            // Function return
            return result;
        } // Function-End: searchMapByKey()


        // Search Map by Key
        // (Case: boost:bimap)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given key in supplied map. 
        *
        * If given key is found within the map, function returns the related value of the container-pair.
        * If no key is found within the map, function returns false.
        * Map search will ignore capitalization of letters in search-string.
        * 
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
            // Search map by key
            boost::optional<Value> result = searchBiMapByKey(map, key);
            if(!result)
            {
                // Map search failed! Key is NOT found in the map
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Search key [" << key <<"] was NOT found in given map");

                // Function return
                return boost::none;
            } 
        
            // Function return
            return result;
        } // Function-End: searchMapByKey()


        // Search Map by Value
        // (Case: std:map)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given value in supplied map. 
        * 
        * If given value is found within the map, function returns the related key of the container-pair.
        * If no value is found within the map, function returns false. 
        * Map search will ignore capitalization of letters in search-string.
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
            // Search map by value
            boost::optional<Key> result = searchStdMapByValue(map, value);
            if(!result)
            {
                // Map search failed! Key is NOT found in the map
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Search value [" << value <<"] was NOT found in given map");

                // Function return
                return boost::none;
            } 
            // Function return
            return result;
        } // Function-End: searchMapByValue()


        // Search Map by Value
        // (Case: boost:bimap)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given value in supplied map.
        *  
        * If given value is found within the map, function returns the related key of the container-pair.
        * If no value is found within the map, function returns false.
        * Map search will ignore capitalization of letters in search-string.
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
            // Search map by value
            boost::optional<Key> result = searchBiMapByValue(map, value);
            if(!result)
            {
                // Map search failed! Key is NOT found in the map
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Search value [" << value <<"] was NOT found in given map");

                // Function return
                return boost::none;
            } 
            // Function return
            return result;
        } // Function-End: searchMapByValue()


    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

        // Search Map by Key (std:map)
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
            auto it = map.find(key);

            // Check for searched key in map
            if(it == map.end())
            {
                // Map search failed! Key is NOT found in the map
                // Return false
                return boost::none;
            }

            // Map search success! Key is found in the map
            // Return related value
            return boost::make_optional(it->second);
        } // Function-End: searchStdMapByKey()


        // Search Map by Key (std:map)
        // (string-key and case-ignore)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given key in supplied map.
        * 
        * If given key is found within the map, function returns the related value of the container-pair.
        * If no key is found within the map, function returns false.
        * Map search will ignore capitalization of letters in search-string.
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
            // Iterate through given map
            for (const auto& entry : map) 
            {
                // Compare iterator-value against supplied key
                // (Case-Insensitive Comparison) 
                if (compareStringsCaseInsensitive(entry.first, key)) 
                {
                    // Map search success! Key is found in the map
                    // Return related value
                    return boost::make_optional(entry.second);
                }
                // Continue iteration
            }
            // Map search failed! Key is NOT found in the map
            // Return false
            return boost::none;
        } // Function-End: searchStdMapByKey()


        // Search Map by Key (boost:bimap)
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
            auto it = map.left.find(key);

            // Check for searched key (left-element) in map
            if(it == map.left.end())
            {
                // Map search failed! Key (left-element) is NOT found in the map
                // Return false
                return boost::none;
            }
            // Map search success! Key (left-element) is found in the map
            // Return related Value (right-element)
            return boost::make_optional(it->second);
        } // Function-End: searchBiMapByKey()


        // Search Map by Key (boost:bimap) 
        // (string-key and case-ignore)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given key in supplied map. 
        *
        * If given key is found within the map, function returns the related value of the container-pair.
        * If no key is found within the map, function returns false.
        * Map search will ignore capitalization of letters in search-string.
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
            // Iterate through given map (left-element)
            for (const auto& entry : map.left) 
            {
                // Compare iterator-value against supplied key
                // (Case-Insensitive Comparison) 
                if (compareStringsCaseInsensitive(entry.first, key)) 
                {
                    // Map search success! Key (left-element) is found in the container
                    // Return related Value (right-element)
                    return boost::make_optional(entry.second);
                }
                // Continue iteration
            }
            // Map search failed! Key is NOT found in the map
            // Return false
            return boost::none;
        } // Function-End: searchBiMapByKey()


        // Search Map by Value (std:map)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given value in supplied map. 
        *
        * If given value is found within the map, function returns the related key of the container-pair.
        * If no value is found within the map, function returns false 
        * 
        * \param map    Map to search thorugh [std::map<typename Key, typename Value>]
        * \param value  Value to search for [typename Value]
        * 
        * \return Function return: Successful: key [typename Key] / Unsuccessful: false [bool]
        */
        template<typename Key, typename Value>
        static boost::optional<Key> searchStdMapByValue(
            const std::map<Key, Value>& map,
            const Value& value)
        {
            // Iterate through given map
            for (const auto& entry : map) 
            {
                // Compare iterator-value against supplied value
                if (entry.second == value) 
                {
                    // Map search success! Value is found in the map
                    // Return related Key
                    return boost::make_optional(entry.first);
                }
                // Continue iteration
            }
            // Map search failed! Value is NOT found in the map
            // Return false
            return boost::none;
        } // Function-End: searchStdMapByValue()


        // Search Map by Value (std:map) 
        // (string-value and case-ignore)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given value in supplied map. 
        *
        * If given value is found within the map, function returns the related key of the container-pair.
        * If no value is found within the map, function returns false 
        * Map search will ignore capitalization of letters in search-string.
        * 
        * \param map    Map to search thorugh [std::map<std::string, typename Value>]
        * \param key    Value to search for [std::string]
        * 
        * \return Function return: Successful: key [std::string] / Unsuccessful: false [bool]
        */
        template<typename Key>
        static boost::optional<Key> searchStdMapByValue(
            const std::map<Key, std::string>& map,
            const std::string& value)
        {
            // Iterate through given map
            for (const auto& entry : map) 
            {
                // Compare iterator-value against supplied value
                // (Case-Insensitive Comparison) 
                if (compareStringsCaseInsensitive(entry.second, value)) 
                {
                    // Map search success! Value is found in the map
                    // Return related key
                    return boost::make_optional(entry.first);
                }
                // Continue iteration
            }
            // Map search failed! Value is NOT found in the map
            // Return false
            return boost::none;
        } // Function-End: searchStdMapByValue()


        // Search Map by Value (boost:bimap)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given value in supplied map.
        * 
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

            // Check for searched value (right-element) in map
            if(search == map.right.end())
            {
                // Map search success! Value (right-element) is found in the map
                // Return related Key (left-element)
                return boost::none;
            }

            // Map search success! Value (right-element) is found in the map
            // Return related Key (left-element)
            return search->second;
        } // Function-End: searchBiMapByValue()


        // Search Map by Value (boost:bimap)
        // (string-value and case-ignore)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search for given value in supplied map.
        *
        * If given value is found within the map, function returns the related key of the container-pair.
        * If no value is found within the map, function returns false 
        * Map search will ignore capitalization of letters in search-string.
        * 
        * \param map    Map to search thorugh [std::map<typename Key, std::string>]
        * \param key    Value to search for [std::string]
        * 
        * \return Function return: Successful: key [typename Key] / Unsuccessful: false [bool]
        */
        template<typename Key>
        static boost::optional<Key> searchBiMapByValue(
            const boost::bimap<Key, std::string>& map,
            const std::string& value)
        {
            // Iterate through given map (right-element)
            for (const auto& entry : map.right) 
            {
                // Compare iterator-value against supplied value
                // (Case-Insensitive Comparison) 
                if (compareStringsCaseInsensitive(entry.first, value)) 
                {
                    // Map search success! Value (right-element) is found in the map
                    // Return related Key (left-element)
                    return boost::make_optional(entry.second);
                }
                // Continue iteration
            }
            // Map search failed! Value is NOT found in the map
            // Return false
            return boost::none;
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