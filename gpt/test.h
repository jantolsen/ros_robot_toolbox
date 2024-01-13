template<typename Key, typename Value>
static boost::optional<Value> searchMap(
    const boost::bimap<Key, Value>& map,
    Key& key)
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
        return it->second;
    } // Function-End: searchBiMapByKey()


template<typename Key, typename Value>
static boost::optional<Value> searchMap(
    const boost::bimap<Key, Value>& map,
    Value& key)
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