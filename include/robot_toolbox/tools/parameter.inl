// Robotics Toolbox - Parameter Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Parameter Tools contains utility functions for
//      loading, convertion and validation of 
//      parameter-data from parameter-container
//      obtained from the parameter-server. 
//
// Version:
//  0.2 -   Overhaul of Parameter Tools implementation.
//          Introduced inline implementation.
//          [12.01.2024]  -   Jan T. Olsen
//  0.1 -   Initial Version
//          [17.12.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include guard:
// -------------------------------
#ifndef PARAMETER_TOOL_INL
#define PARAMETER_TOOL_INL

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

    // Get Parameter Data
    // (Template: Primary/Default)
    // -------------------------------
    // (Function Overloading)
    template<typename ParamData>
    inline ParamData Parameter::getParamData(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_name)
    {
        // Load parameter data
        auto result_data = Parameter::loadParamData<ParamData>(param_xml, param_name);
        if (!result_data)
        {
            // Parameter loading failed!
            // Throw runtime exception
            throw std::runtime_error("Runtime exception! " + Parameter::CLASS_PREFIX + __FUNCTION__ 
                + ": Failed! Parameter [" + param_name + "] is missing or configured incorrectly");
        }

        // Parameter loading success!
        // Return parameter value
        return result_data.value();
    } // Function-End: getParamData()


    // Get Parameter Data
    // (Template: Primary/Default)
    // -------------------------------
    // (Function Overloading)
    template<typename ParamData>
    inline ParamData Parameter::getParamData(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_name,
        const ParamData& data_default)
    {
        // Load parameter data
        auto result_data = Parameter::loadParamData<ParamData>(param_xml, param_name, data_default);
        if (!result_data)
        {
            // Parameter loading failed!
            // Throw runtime exception
            throw std::runtime_error("Runtime exception! " + Parameter::CLASS_PREFIX + __FUNCTION__ 
                + ": Failed! Parameter [" + param_name + "] is missing or configured incorrectly");
        }

        // Parameter loading success!
        // Return parameter value
        return result_data.value();
    } // Function-End: getParamData()


    // Get Parameter Data
    // (Template: Primary/Default)
    // -------------------------------
    // (Function Overloading)
    template<typename ParamData>
    inline ParamData Parameter::getParamData(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_name,
        const std::vector<ParamData>& data_set)
    {
        // Load parameter data
        auto result_data = Parameter::loadParamData<ParamData>(param_xml, param_name, data_set);
        if (!result_data)
        {
            // Parameter loading failed!
            // Throw runtime exception
            throw std::runtime_error("Runtime exception! " + Parameter::CLASS_PREFIX + __FUNCTION__ 
                + ": Failed! Parameter [" + param_name + "] is missing or configured incorrectly");
        }

        // Parameter loading success!
        // Return parameter value
        return result_data.value();
    } // Function-End: getParamData()


    // Get Parameter Data
    // (Template: Primary/Default)
    // -------------------------------
    // (Function Overloading)
    template<typename ParamData, typename MapKey, typename MapValue>
    inline ParamData Parameter::getParamData(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_name,
        const std::map<MapKey, MapValue>& data_map)
    {
        // Load parameter data
        auto result_data = Parameter::loadParamData<ParamData>(param_xml, param_name, data_map);
        if (!result_data)
        {
            // Parameter loading failed!
            // Throw runtime exception
            throw std::runtime_error("Runtime exception! " + Parameter::CLASS_PREFIX + __FUNCTION__ 
                + ": Failed! Parameter [" + param_name + "] is missing or configured incorrectly");
        }

        // Parameter loading success!
        // Return parameter value
        return result_data.value();
    } // Function-End: getParamData()


    // Load Parameter Data
    // (Template: Primary/Default)
    // -------------------------------
    // (Function Overloading)
    template<typename ParamData>
    inline boost::optional<ParamData> Parameter::loadParamData(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_name)
    {
        // Check parameter-data for specified parameter-member
        if(!param_xml.hasMember(param_name))
        {
            // Parameter search failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter [" << param_name << "] was NOT found");

            // Function return
            return boost::none;
        }

        // Convert parameter-member to respective data-type
        boost::optional<ParamData> result_data = Parameter::convertParamType<ParamData>(param_xml[param_name]);
        if(!result_data)
        {
            // Parameter convertion failed
            ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__ 
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
    // (Function Overloading)
    template<typename ParamData>
    inline boost::optional<ParamData> Parameter::loadParamData(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_name,
        const ParamData& data_default)
    {
        // Check parameter-data for specified parameter-member
        if(!param_xml.hasMember(param_name))
        {
            // Report to terminal
            ROS_WARN_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Warning! Parameter [" << param_name << "] was NOT found."
                << " Using default-data");

            // Parameter search failed
            return data_default;
        }

        // Convert parameter-member to respective data-type
        boost::optional<ParamData> result_data = Parameter::convertParamType<ParamData>(param_xml[param_name]);
        if(!result_data)
        {
            // Parameter convertion failed
            ROS_WARN_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__ 
                << ": Warning! Parameter [" << param_name << "] is configured incorrectly."
                << " Using default-data");

            // Function return
            return data_default;
        }

        // Function return
        return result_data;
    } // Function-End: loadParamData()


    // Load Parameter Data
    // (Template: Primary/Default)
    // -------------------------------
    // (Function Overloading)
    template<typename ParamData>
    inline boost::optional<ParamData> Parameter::loadParamData(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_name,
        const std::vector<ParamData>& data_set)
    {
        // Call overloading parameter loading function
        boost::optional<ParamData> result_data = loadParamData<ParamData>(param_xml, param_name);
        if(!result_data)
        {
            // Function return
            return boost::none;
        }

        // Validate parameter-data against given validation-set
        if(!isValueInSet(result_data.value(), data_set))
        {
            // Parameter validation failed
            ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__ 
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
    // (Function Overloading)
    template<typename ParamData, typename MapKey, typename MapValue>
    inline boost::optional<ParamData> Parameter::loadParamData(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_name,
        const std::map<MapKey, MapValue>& data_map)
    {
        // Call overloading parameter loading function
        boost::optional<MapKey> result_data = loadParamData<MapKey>(param_xml, param_name);
        if(!result_data)
        {
            // Function return
            return boost::none;
        }

        // Using paramter-data as key, search for paired value in given item-map
        boost::optional<MapValue> result_search = Toolbox::Map::searchMapByKey(data_map, result_data.value());
        if(!result_search)
        {
            // Parameter validation failed
            ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter [" << param_name <<"] with value"
                << " [" << result_data.value() << "] is NOT a valid valid key in given item-map");

            // Function return
            return boost::none;
        }
        // Map search success! paramter-data is found in the container
        // Cast and return related item-value
        return static_cast<ParamData>(result_search.value());
    } // Function-End: loadParamData()


    // Convert XmlRpcValue Parameter
    // (Template: Primary/Default)
    // -------------------------------
    template<typename ParamType>
    inline boost::optional<ParamType> Parameter::convertParamType(
        const XmlRpc::XmlRpcValue& param)
    {
        // Report to terminal
        ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
            << ": Failed! Parameter (" << param << ")"
            << " [XmlRpcValueType: " << Parameter::getParamTypeName(param) << "]"
            << " is unsupporeted for conversion");

        // Unsupported type!
        return boost::none;
    } // Function-End: convertParamType()

    
    // Convert XmlRpcValue Parameter
    // -------------------------------
    // (Template Specialization: Bool)
    template<>
    inline boost::optional<bool> Parameter::convertParamType(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        // (copy input-parameter to remove const to allow type-casting)
        XmlRpc::XmlRpcValue param_ = param;

        // Check parameter against specified type
        if(param_.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
        {
            // Report to terminal
            ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << Parameter::getParamTypeName(param) << "]"
                << " does NOT convert to [boolean]");

            // Parameter type mismatch, return false
            return boost::none;
        }
        // Function return
        return static_cast<bool>(param_);
    } // Function-End: convertParamType<bool>()
    

    // Convert XmlRpcValue Parameter
    // -------------------------------
    // (Template Specialization: Int)
    template<>
    inline boost::optional<int> Parameter::convertParamType(
        const XmlRpc::XmlRpcValue& param) 
    {
        // Local variable(s)
        // (copy input-parameter to remove const to allow type-casting)
        XmlRpc::XmlRpcValue param_ = param;

        // Check parameter against specified type
        if(param_.getType() != XmlRpc::XmlRpcValue::TypeInt)
        {
            // Report to terminal
            ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << Parameter::getParamTypeName(param) << "]"
                << " does NOT convert to [int]");

            // Parameter type mismatch, return false
            return boost::none;
        }

        // Function return
        return static_cast<int>(param_);
    } // Function-End: convertParamType<int>()


    // Convert XmlRpcValue Parameter
    // -------------------------------
    // (Template Specialization: Double)
    template<>
    inline boost::optional<double> Parameter::convertParamType(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        // (copy input-parameter to remove const to allow type-casting)
        XmlRpc::XmlRpcValue param_ = param;

        // Check parameter against specified type
        if(param_.getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
            // Report to terminal
            ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << Parameter::getParamTypeName(param) << "]"
                << " does NOT convert to [double]");

            // Parameter type mismatch, return false
            return boost::none;
        }
        // Function return
        return static_cast<double>(param_);
    } // Function-End: convertParamType<bool>()


    // Convert XmlRpcValue Parameter
    // -------------------------------
    // (Template Specialization: String)
    template<>
    inline boost::optional<std::string> Parameter::convertParamType(
        const XmlRpc::XmlRpcValue& param) 
    {
        // Local variable(s)
        // (copy input-parameter to remove const to allow type-casting)
        XmlRpc::XmlRpcValue param_ = param;

        // Check parameter against specified type
        if(param_.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            // Report to terminal
            ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << Parameter::getParamTypeName(param) << "]"
                << " does NOT convert to [string]");

            // Parameter type mismatch, return false
            return boost::none;
        }

        // Function return
        return static_cast<std::string>(param_);
    } // Function-End: convertParamType<std::string>()


    // Convert XmlRpcValue Parameter
    // -------------------------------
    // (Template Specialization: std::vector<bool>)
    template<>
    inline boost::optional<std::vector<bool>> Parameter::convertParamType(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        XmlRpc::XmlRpcValue param_ = param;
        std::vector<bool> result;

        // Check parameter against specified type
        if (param_.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            // Report to terminal
            ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << Parameter::getParamTypeName(param) << "]"
                << " is NOT an array");

            // Parameter type mismatch, return false
            return boost::none;
        }
        
        // Iterate through the array
        for (int i = 0; i < param_.size(); ++i)
        {
            // Check if each element is an boolean
            if (param_[i].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
            {
                result.push_back(static_cast<bool>(param_[i]));
            }
            else
            {
                // Report to terminal
                ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
                    << ": Failed! Parameter-Element (" << param_[i] << ")" 
                    << " [XmlRpcValueType: " << Parameter::getParamTypeName(param_[i]) << "]"
                    << " within Paramter (" << param << ")"
                    << " does NOT convert to [bool]");

                // Element type mismatch, return false
                return boost::none;
            }
        }

        // Function return
        return result;
    } // Function-End: convertParamType<std::vector<bool>>()


    // Convert XmlRpcValue Parameter
    // -------------------------------
    // (Template Specialization: std::vector<int>)
    template<>
    inline boost::optional<std::vector<int>> Parameter::convertParamType(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        XmlRpc::XmlRpcValue param_ = param;
        std::vector<int> result;

        // Check parameter against specified type
        if (param_.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            // Report to terminal
            ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << Parameter::getParamTypeName(param) << "]"
                << " is NOT an array");

            // Parameter type mismatch, return false
            return boost::none;
        }

        // Iterate through the array
        for (int i = 0; i < param_.size(); ++i)
        {
            // Check if each element is an integer
            if (param_[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
            {
                result.push_back(static_cast<int>(param_[i]));
            }
            else
            {
                // Report to terminal
                ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
                    << ": Failed! Parameter-Element (" << param_[i] << ")" 
                    << " [XmlRpcValueType: " << Parameter::getParamTypeName(param_[i]) << "]"
                    << " within Paramter (" << param << ")"
                    << " does NOT convert to [int]");

                // Element type mismatch, return false
                return boost::none;
            }
        }

        // Function return
        return result;
    } // Function-End: convertParamType<std::vector<int>>()


    // Convert XmlRpcValue Parameter
    // -------------------------------
    // (Template Specialization: std::vector<double>)
    template<>
    inline boost::optional<std::vector<double>> Parameter::convertParamType(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        XmlRpc::XmlRpcValue param_ = param;
        std::vector<double> result;

        // Check parameter against specified type
        if (param_.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            // Report to terminal
            ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << Parameter::getParamTypeName(param) << "]"
                << " is NOT an array");

            // Parameter type mismatch, return false
            return boost::none;
        }

        // Iterate through the array
        for (int i = 0; i < param_.size(); ++i)
        {
            // Check if each element is a double
            if (param_[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            {
                result.push_back(static_cast<double>(param_[i]));
            }
            else
            {
                // Report to terminal
                ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
                    << ": Failed! Parameter-Element (" << param_[i] << ")" 
                    << " [XmlRpcValueType: " << Parameter::getParamTypeName(param_[i]) << "]"
                    << " within Paramter (" << param << ")"
                    << " does NOT convert to [double]");

                // Element type mismatch, return false
                return boost::none;
            }
        }

        // Function return
        return result;
    } // Function-End: convertParamType<std::vector<double>>()


    // Convert XmlRpcValue Parameter
    // -------------------------------
    // (Template Specialization: std::vector<std::string>)
    template<>
    inline boost::optional<std::vector<std::string>> Parameter::convertParamType(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        XmlRpc::XmlRpcValue param_ = param;
        std::vector<std::string> result;

        // Check parameter against specified type
        if (param_.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            // Report to terminal
            ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << Parameter::getParamTypeName(param) << "]"
                << " does is not an array");

            // Parameter type mismatch, return false
            return boost::none;
        }

        // Iterate through the array
        for (int i = 0; i < param_.size(); ++i)
        {
            // Check if each element is a string
            if (param_[i].getType() == XmlRpc::XmlRpcValue::TypeString)
            {
                result.push_back(static_cast<std::string>(param_[i]));
            }
            else
            {
                // Report to terminal
                ROS_ERROR_STREAM(Parameter::CLASS_PREFIX << __FUNCTION__
                    << ": Failed! Parameter-Element (" << param_[i] << ")" 
                    << " [XmlRpcValueType: " << Parameter::getParamTypeName(param_[i]) << "]"
                    << " within Paramter (" << param << ")"
                    << " does NOT convert to [string]");

                // Element type mismatch, return false
                return boost::none;
            }
        }
        // Function return
        return result;
    } // Function-End: convertParamType<std::vector<std::string>>()

} // End of namespace "Toolbox"
#endif // PARAMETER_TOOL_INL 