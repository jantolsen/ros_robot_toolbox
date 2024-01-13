// Robotics Toolbox - Parameter Loader
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Parameter Loader contains utility functions for loading
//      and validation of parameter-data from parameter-container
//      obtained from the parameter-server. 
//
// Version:
//  0.2 -   Split Parameter Tools implementation
//          into multiple files utilizing template-classes 
//          and inline implementation.
//          [12.01.2024]  -   Jan T. Olsen
//  0.1 -   Initial Version
//          [17.12.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include guard:
// -------------------------------
#ifndef PARAM_LOADER_INL
#define PARAM_LOADER_INL

// Include Header-files:
// -------------------------------
    // Standard
    #include <iostream>
    #include <string>
    #include <vector>

    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include <robot_toolbox/toolbox.h>

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{   
    // Parameter Loader Class - Member(s)
    // -------------------------------

    // Load Parameter Data
    // (Template: Primary/Default)
    // -------------------------------
    template<typename ParamData>
    inline boost::optional<ParamData> ParameterLoader::loadParamData(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_name)
    {
        // Check for parameter-member in given parameter-data
        if(!ParameterTool::checkMember(param_xml, param_name))
        {
            // Parameter search failed
            return boost::none;
        } 

        // Convert parameter-member to respective data-type
        boost::optional<ParamData> result_data = ParameterConverter::convertParamType<ParamData>(param_xml[param_name]);
        if(!result_data)
        {
            // Parameter convertion failed
            ROS_ERROR_STREAM("Toolbox::ParameterLoader::" << __FUNCTION__ 
                << ": Failed! Parameter [" << param_name << "] is configured incorrectly");

            // Function return
            return boost::none;
        }

        // Function return
        return result_data;
    } // Function-End: loadParamData()


    // Load Parameter Data
    // (Template: Primary/Default)
    // -------------------------------
    template<typename ParamData>
    inline boost::optional<ParamData> ParameterLoader::loadParamData(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_name,
        const std::vector<ParamData>& validation_set)
    {
        // Call overloading parameter loading function
        boost::optional<ParamData> result_data = loadParamData<ParamData>(param_xml, param_name);
        if(!result_data)
        {
            // Function return
            return boost::none;
        }

        // Validate parameter-data against given validation-set
        if(!isValueInSet(result_data.value(), validation_set))
        {
            // Parameter validation failed
            ROS_ERROR_STREAM("Toolbox::ParameterLoader::" << __FUNCTION__ 
                << ": Failed! Parameter [" << param_name <<"] with value"
                << " [" << result_data.value() << "] is NOT a valid entry");

            // Function return
            return boost::none;
        }

        // Function return
        return result_data;
    } // Function-End: loadParamData()


    // Load Parameter Data
    // (Template: Primary/Default)
    // -------------------------------
    template<typename ParamData, typename MapKey, typename MapValue>
    inline boost::optional<ParamData> ParameterLoader::loadParamData(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_name,
        const std::map<MapKey, MapValue>& item_map)
    {
        // Call overloading parameter loading function
        boost::optional<MapKey> result_data = loadParamData<MapKey>(param_xml, param_name);
        if(!result_data)
        {
            // Function return
            return boost::none;
        }

        // Using paramter-data as key, search for paired value in given item-map
        boost::optional<MapValue> result_search = Toolbox::Map::searchMapByKey(item_map, result_data.value());
        if(!result_search)
        {
            // Parameter validation failed
            ROS_ERROR_STREAM("Toolbox::ParameterLoader::" << __FUNCTION__ 
                << ": Failed! Parameter [" << param_name <<"] with value"
                << " [" << result_data.value() << "] is NOT a valid valid key in given item-map");

            // Function return
            return boost::none;
        }
        // Map search success! paramter-data is found in the container
        // Cast and return related item-value
        return static_cast<ParamData>(result_search.value());
    } // Function-End: loadParamData()


} // End of namespace "Toolbox"
#endif // PARAM_LOADER_INL 