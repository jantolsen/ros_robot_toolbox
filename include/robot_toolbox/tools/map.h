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

        // Map: Reverse Map
        // -------------------------------
        // (Function Overloading)
        /** \brief Reverse a map (swap key and value)
        * \param map        Map to reverse [std::map<typename Key, typename Value>]
        * \return Returns reversed map [std::map<typename Value, typename Key>]
        */
        template<typename Key, typename Value>
        static std::map<Value, Key> mapReverse(
            const std::map<Key, Value>& map)
        {
            // Create reverse map
            std::map<Value, Key> reverse_map;

            // Iterate through supplied map
            for(auto const& it : map)
            {
                // Insert reverse key-value pair
                reverse_map.insert(std::make_pair(it.second, it.first));
            }

            // Function return
            return reverse_map;
        } // Function-End: mapReverse()


        // Map: Reverse Map
        // -------------------------------
        // (Function Overloading)
        /** \brief Reverse a map (swap key and value)
        * Map contains struct-operator for CaseInsensitiveComparator 
        * (used for ignore capitalization of letters in string)
        * \param map        Map to reverse [std::map<typename Key, typename Value, typename Operator>]
        * \return Returns reversed map [std::map<typename Value, typename Key, typename Operator>]
        */
        template<typename Key, typename Value, typename Operator>
        static std::map<Value, Key, Operator> mapReverse(
            const std::map<Key, Value, Operator>& map)
        {
            // Create reverse map
            std::map<Value, Key, Operator> reverse_map;

            // Iterate through supplied map
            for(auto const& it : map)
            {
                // Insert reverse key-value pair
                reverse_map.insert(std::make_pair(it.second, it.first));
            }

            // Function return
            return reverse_map;
        } // Function-End: mapReverse()


        template<typename SearchType, typename ResultType, typename MapKey, typename MapValue>
        typename std::enable_if<std::is_same<SearchType, MapKey>::value, bool>::type
        static mapFind(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& map,
            ResultType& result_item)
        {
            // Search map using search-item as map-key to search for related value
            return mapFindValue(search_item, map, result_item);
        } // Function-End: mapSearch()

        template<typename SearchType, typename ResultType, typename MapKey, typename MapValue>
        typename std::enable_if<std::is_same<SearchType, MapValue>::value, bool>::type
        static mapFind(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& map,
            ResultType& result_item)
        {
            // Search map using search-item as map-value to search for related key
            return mapFindKey(search_item, map, result_item);
        } // Function-End: mapSearch()


        template<typename SearchType, typename ResultType, typename MapKey, typename MapValue>
        typename std::enable_if<(not std::is_same<SearchType, MapKey>::value) 
            and (not std::is_same<SearchType, MapValue>::value), bool>::type
        static mapFind(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& map,
            ResultType& result_item)
        {
            
            return false;
        } // Function-End: mapSearch()


        // Map: Search for Item
        // -------------------------------
        // (Function Overloading)
        /** \brief Search through supplied map to find the related element-item for the given search-item.
        * Function checks data-type of search-item aginst map's key-type and value-type
        * to determine which item (key or value) to use for the map search.
        * \param search_item    Item to search for [typename SearchType]
        * \param map            Map to search thorugh [std::map<typename MapKey, typename MapValue>]
        * \param result_item    Resulting item [typename ResultType]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename SearchType, typename ResultType, typename MapKey, typename MapValue>
        static bool mapSearch(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& map,
            ResultType& result_item)
        {
            // Map Search
            if(mapFind<SearchType>(search_item, map, result_item))
            {
                // Function return
                return true;
            }

            // // Compare search-item type vs map-key type
            // if constexpr (std::is_same<SearchType, MapKey>::value)
            // {
            //     // Search map using search-item as map-key to search for related value
            //     return mapFindValue(search_item, map, result_item);
            // }
            // // Compare search-item type vs map-value type
            // else if constexpr (std::is_same<SearchType, MapValue>::value)
            // {
            //     // Search map using search-item as map-value to search for related key
            //     return mapFindKey(search_item, map, result_item);
            // }

            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Data-Type of Search-Item: [" << typeid(search_item).name() << "]"
                << " does NOT match the data-type of either the Map's Key-type [" << typeid(MapKey).name() << "]"
                << " nor Value-type [" << typeid(MapValue).name() << "]");

            // Function return
            return false;
        } // Function-End: mapSearch()


        // // Map: Search for Item 
        // // -------------------------------
        // // (Function Overloading)
        // /** \brief Search through supplied map to find the related element-item for the given search-item.
        // * Function checks data-type of search-item aginst map's key-type and value-type
        // * to determine which item (key or value) to use for the map search.
        // * Supplied map contains operator (typically CaseInsensitiveComparator 
        // * used to ignore capitalization of letters in key-string)
        // * \param search_item    Item to search for [typename SearchType]
        // * \param map            Map to search thorugh [std::map<typename MapKey, typename MapValue, typename MapOperator>]
        // * \param result_item    Resulting item [typename ResultType]
        // * \return Function result: Successful/unsuccessful (true/false)
        // */
        // template<typename SearchType, typename ResultType, typename MapKey, typename MapValue, typename MapOperator>
        // static bool mapSearch(
        //     const SearchType& search_item, 
        //     const std::map<MapKey, MapValue, MapOperator>& map,
        //     ResultType& result_item)
        // {
        //     // Compare search-item type vs map-key type
        //     if constexpr (std::is_same<SearchType, MapKey>::value)
        //     {
        //         // Search map using search-item as map-key to search for related value
        //         return mapFindValue(search_item, map, result_item);
        //     }
        //     // Compare search-item type vs map-value type
        //     else if constexpr (std::is_same<SearchType, MapValue>::value)
        //     {
        //         // Search map using search-item as map-value to search for related key
        //         return mapFindKey(search_item, map, result_item);
        //     }

        //     // Report to terminal
        //     ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
        //         << ": Failed! Data-Type of Search-Item: [" << typeid(search_item).name() << "]"
        //         << " does NOT match the data-type of either the Map's Key-type [" << typeid(MapKey).name() << "]"
        //         << " nor Value-type [" << typeid(MapValue).name() << "]");

        //     // Function return
        //     return false;
        // } // Function-End: mapSearch()

        template<typename SearchType, typename MapKey, typename MapValue>
        typename std::enable_if<std::is_same<SearchType, MapKey>::value, bool>::type
        static mapCheck(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& map)
        {
            // Search map using search-item as map-key
            return mapContainsKey(search_item, map);
        } // Function-End: mapSearch()

        template<typename SearchType, typename MapKey, typename MapValue>
        typename std::enable_if<std::is_same<SearchType, MapValue>::value, bool>::type
        static mapCheck(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& map)
        {
            // Search map using search-item as map-value
            return mapContainsValue(search_item, map);
        } // Function-End: mapSearch()

        template<typename SearchType, typename MapKey, typename MapValue>
        typename std::enable_if<(not std::is_same<SearchType, MapKey>::value) 
            and (not std::is_same<SearchType, MapValue>::value), bool>::type
        static mapCheck(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& map)
        {
            // Search map using search-item as map-value
            return false;
        } // Function-End: mapSearch()


        // Map: Contains Item 
        // (Search and check if given value exists in map)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search and check if given search-item exists in supplied map.
        * Function checks data-type of search-item aginst map's key-type and value-type
        * to determine which item (key or value) to use for the map search.
        * \param search_item    Item to search for [typename SearchType]
        * \param map            Map to search thorugh [std::map<typename MapKey, typename MapValue>]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename SearchType, typename MapKey, typename MapValue>
        static bool mapContains(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& map)
        {
            // Map Search
            if(mapCheck<SearchType>(search_item, map))
            {
                // Function return
                return true;
            }

            // // Compare search-item type vs map-key type
            // if constexpr (std::is_same<SearchType, MapKey>::value)
            // {
            //     // Search for item as key
            //     if(!mapContainsKey(search_item, map))
            //     {
            //         // Item was found in the map
            //         // Report to terminal
            //         ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
            //             << ": Failed! Given item: [" << search_item << "] was NOT found in given map");
                        
            //         // Function return
            //         return true;
            //     }

            //     // Function return
            //     return true;
            // }
            // // Compare search-item type vs map-value type
            // else if constexpr (std::is_same<SearchType, MapValue>::value)
            // {
            //     // Search for item as value
            //     if(!mapContainsValue(search_item, map))
            //     {
            //         // Item was found in the map
            //         // Report to terminal
            //         ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
            //             << ": Failed! Given item: [" << search_item << "] was NOT found in given map");
                        
            //         // Function return
            //         return true;
            //     } 

            //     // Function return
            //     return true;
            // }

            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Data-Type of Search-Item: [" << typeid(search_item).name() << "]"
                << " does NOT match the data-type of either the Map's Key-type [" << typeid(MapKey).name() << "]"
                << " nor Value-type [" << typeid(MapValue).name() << "]");

            // Function return
            return false;
        } // Function-End: mapCheckItem()
        

    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

        // Map: Find Value
        // (Search and find Value in Map using Key) 
        // -------------------------------
        // (Function Overloading)
        /** \brief Search and find value in supplied map by searching for given key
        * \param key        Key to search for [typename Key]
        * \param map        Map to search thorugh [std::map<typename Key, typename Value>]
        * \param value      Value at given key in map [typename Value]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename Key, typename Value>
        static bool mapFindValue(
            const Key& key, 
            const std::map<Key, Value>& map,
            Value& value)
        {
            // Search for key in map
            auto search = map.find(key);

            // Check if searched key is found in the container
            if(search != map.end())
            {   
                // Value found for given key in map
                value = search->second;

                // Function return
                return true;
            }
            // No value was found in the container
            // (iterator has reached the end of the container)

            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given Key: [" << key << "] was NOT found in given map");

            // Function return
            return false;
        } // Function-End: mapFindValue()


        // Map: Find Value
        // (Search and find Value in Map using Key [std::string]) 
        // -------------------------------
        // (Function Overloading)
        /** \brief Search and find value in supplied map by searching for given key [std::string]
        * \param key        Key to search for [std::string]
        * \param map        Map to search thorugh [std::map<std::string, typename Value>]
        * \param value      Value at given key in map [typename Value]
        * \param case_insensitive   Ignore capitalization of letters in given key [std::string]
        *                           (enabled as default) [bool]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename Value>
        static bool mapFindValue(
            const std::string& key, 
            const std::map<std::string, Value>& map,
            Value& value,
            const bool& case_insensitive=true)
        {
            // Check for case-insensitive key flag
            if(case_insensitive)
            {
                // Create a new map instance with case-insensitive comparator
                std::map<std::string, Value, CaseInsensitiveComparator> map_new;

                // Copy given map elements to new map
                map_new.insert(map.begin(), map.end());

                // Call overloading function
                // (map with CaseInsensitiveComparator)
                return mapFindValue(key, map_new, value);
            }
            // Non case-insensitive key
            else
            {
                // Search for key in map
                auto search = map.find(key);

                // Check if searched key is found in the container
                if(search != map.end())
                {   
                    // Value found for given key in map
                    value = search->second;

                    // Function return
                    return true;
                }
                // No value was found in the container
                // (iterator has reached the end of the container)

                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Given Key: [" << key << "] was NOT found in given map");

                // Function return
                return false;
            }
        } // Function-End: mapFindValue()


        // Map: Find Value
        // (Search and find Value in Map using Key) 
        // -------------------------------
        // (Function Overloading)
        /** \brief Search and find value in supplied map by searching for given key
        * Supplied map contains operator (typically CaseInsensitiveComparator 
        * used to ignore capitalization of letters in key-string)
        * \param key        Key to search for [typename Key]
        * \param map        Map to search thorugh [std::map<typename Key, typename Value, typename Operator>]
        * \param value      Value at given key in map [typename Value]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename Key, typename Value, typename Operator>
        static bool mapFindValue(
            const Key& key, 
            const std::map<Key, Value, Operator>& map,
            Value& value)
        {
            // Search for key in map
            auto search = map.find(key);

            // Check if searched key is found in the container
            if(search != map.end())
            {   
                // Value found for given key in map
                value = search->second;

                // Function return
                return true;
            }
            // No value was found in the container
            // (iterator has reached the end of the container)

            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given Key: [" << key << "] was NOT found in given map");

            // Function return
            return false;
        } // Function-End: mapFindValue()


        template<typename Value, typename Key>
        typename std::enable_if<std::is_same<Value, std::string>::value, bool>::type
        static testFindKey(
            const Value& value, 
            const std::map<Key, Value>& map,
            Key& key)
        {
            // Iterate through supplied map
            for(auto const& it : map)
            {
                // Compare iterator-value against supplied value
                // (Case-Insensitivity)
                if(strcasecmp(it.second.c_str(), value.c_str()) == 0)
                {
                    // Set Key equal to map-key at given value
                    key = it.first;

                    // Function return
                    return true;
                }
                // Continue iteration
            }
        } // Function-End: mapSearch()

        template<typename Value, typename Key>
        typename std::enable_if<(not std::is_same<Value, std::string>::value), bool>::type
        static testFindKey(
            const Value& value, 
            const std::map<Key, Value>& map,
            Key& key)
        {
            // Iterate through supplied map
            for(auto const& it : map)
            {
                // Compare iterator-value against supplied value
                if(it.second == value)
                {
                    // Set Key equal to map-key at given value
                    key = it.first;

                    // Function return
                    return true;
                }
                // Continue iteration
            }
        } // Function-End: mapSearch()
        
        // Map: Find Key
        // (Search and find Key in Map using Value)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search and find key in supplied map by searching for given value
        * \param value      Value to search for [typename Value]
        * \param map        Map to search thorugh [std::map<typename Key, typename Value>]
        * \param key        Key at given value in map [typename Key]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename Key, typename Value>
        static bool mapFindKey(
            const Value& value, 
            const std::map<Key, Value>& map,
            Key& key)
        {
            if(testFindKey<Value>(value, map, key))
            {
                // Function return
                return true;
            }

            // // Check if value is string-type
            // if constexpr (std::is_same<Value, std::string>::value)
            // {   
            //     // Iterate through supplied map
            //     for(auto const& it : map)
            //     {
            //         // Compare iterator-value against supplied value
            //         // (Case-Insensitivity)
            //         if(strcasecmp(it.second.c_str(), value.c_str()) == 0)
            //         {
            //             // Set Key equal to map-key at given value
            //             key = it.first;

            //             // Function return
            //             return true;
            //         }
            //         // Continue iteration
            //     }
            // }
            // // Non string-type
            // else
            // {
            //     // Iterate through supplied map
            //     for(auto const& it : map)
            //     {
            //         // Compare iterator-value against supplied value
            //         if(it.second == value)
            //         {
            //             // Set Key equal to map-key at given value
            //             key = it.first;

            //             // Function return
            //             return true;
            //         }
            //         // Continue iteration
            //     }
            // }

            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Value: [" << value << "] was NOT found in given map");

            // Function return
            return false;
        } // Function-End: mapFindKey()


        // // Map: Find Key
        // // (Search and find Key in Map using Value)
        // // -------------------------------
        // // (Function Overloading)
        // /** \brief Search and find key in supplied map by searching for given value
        // * Supplied map contains operator (typically CaseInsensitiveComparator 
        // * used to ignore capitalization of letters in key-string)
        // * \param value      Value to search for [typename Value]
        // * \param map        Map to search thorugh [std::map<typename Key, typename Value, typename Operator>]
        // * \param key        Key at given value in map [typename Key]
        // * \return Function result: Successful/unsuccessful (true/false)
        // */
        // template<typename Key, typename Value, typename Operator>
        // static bool mapFindKey(
        //     const Value& value, 
        //     const std::map<Key, Value, Operator>& map,
        //     Key& key)
        // {
        //     // Check if value is string-type
        //     if constexpr (std::is_same<Value, std::string>::value)
        //     {   
        //         // Iterate through supplied map
        //         for(auto const& it : map)
        //         {
        //             // Compare iterator-value against supplied value
        //             // (Case-Insensitivity)
        //             if(strcasecmp(it.second.c_str(), value.c_str()) == 0)
        //             {
        //                 // Set Key equal to map-key at given value
        //                 key = it.first;

        //                 // Function return
        //                 return true;
        //             }
        //             // Continue iteration
        //         }
        //     }
        //     // Non string-type
        //     else
        //     {
        //         // Iterate through supplied map
        //         for(auto const& it : map)
        //         {
        //             // Compare iterator-value against supplied value
        //             if(it.second == value)
        //             {
        //                 // Set Key equal to map-key at given value
        //                 key = it.first;

        //                 // Function return
        //                 return true;
        //             }
        //             // Continue iteration
        //         }
        //     }

        //     // Report to terminal
        //     ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
        //         << ": Failed! Value: [" << value << "] was NOT found in given map");

        //     // Function return
        //     return false;
        // } // Function-End: mapFindKey()
        

        // Map: Contains Key 
        // (Search and check if given Key exists in map)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search and check if given key exists in supplied map
        * \param search_item    Item to search for [typename SearchType]
        * \param map            Map to search thorugh [std::map<typename MapKey, typename MapValue>]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename SearchType, typename MapKey, typename MapValue>
        static bool mapContainsKey(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& map)
        {
            // Search for key in map
            auto search = map.find(search_item);

            // Check if searched key is found in the container
            if(search != map.end())
            {   
                // Key is found within map
                return true;
            }
            // Search-item was NOT found in the container
            // (iterator has reached the end of the container)

            // Function return
            return false;
        } // Function-End: mapContainsKey()


        template<typename SearchType, typename MapKey, typename MapValue>
        typename std::enable_if<std::is_same<SearchType, std::string>::value, bool>::type
        static testFindValue(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& map)
        {
            // Iterate through supplied map
            for(auto const& it : map)
            {
                // Compare iterator-value against supplied value
                // (Case-Insensitivity)
                if(strcasecmp(it.second.c_str(), search_item.c_str()) == 0)
                {
                    // Value is found within map
                    return true;
                }
                // Continue iteration
            }
        } // Function-End: mapSearch()

        template<typename SearchType, typename MapKey, typename MapValue>
        typename std::enable_if<(not std::is_same<SearchType, std::string>::value), bool>::type
        static testFindValue(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& map)
        {
            // Iterate through supplied map
            for(auto const& it : map)
            {
                // Compare iterator-value against supplied value
                if(it.second == search_item)
                {
                    // Value is found within map
                    return true;
                }
                // Continue iteration
            }
        } // Function-End: mapSearch()


        // Map: Contains Value 
        // (Search and check if given value exists in map)
        // -------------------------------
        // (Function Overloading)
        /** \brief Search and check if given value exists in supplied map
        * \param search_item    Item to search for [typename SearchType]
        * \param map            Map to search thorugh [std::map<typename MapKey, typename MapValue>]
        * \return Function result: Successful/unsuccessful (true/false)
        */
        template<typename SearchType, typename MapKey, typename MapValue>
        static bool mapContainsValue(
            const SearchType& search_item, 
            const std::map<MapKey, MapValue>& map)
        {
            if(testFindValue<SearchType>(search_item, map))
            {
                // Function return
                return true;
            }
            // // Check if search-item is string-type
            // if constexpr (std::is_same<SearchType, std::string>::value)
            // {   
            //     // Iterate through supplied map
            //     for(auto const& it : map)
            //     {
            //         // Compare iterator-value against supplied value
            //         // (Case-Insensitivity)
            //         if(strcasecmp(it.second.c_str(), search_item.c_str()) == 0)
            //         {
            //             // Value is found within map
            //             return true;
            //         }
            //         // Continue iteration
            //     }
            // }
            // // Non string-type
            // else
            // {
            //     // Iterate through supplied map
            //     for(auto const& it : map)
            //     {
            //         // Compare iterator-value against supplied value
            //         if(it.second == search_item)
            //         {
            //             // Value is found within map
            //             return true;
            //         }
            //         // Continue iteration
            //     }
            // }
            // Search-item was NOT found in the container
            // Function return
            return false;
        } // Function-End: mapContainsValue()


    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:

        // Prefix message for class
        static const std::string CLASS_PREFIX;


}; // End Class: Common
} // End Namespace: Robotics Toolbox
#endif // MAP_TOOL_H 