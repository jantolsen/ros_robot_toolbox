// Robotics Toolbox - Parameter Converter
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Parameter Converter contains utility functions for conversion 
//      of XmlRpc::XmlRpcValue to fundamental types. 
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
#ifndef PARAM_CONVERTER_INL
#define PARAM_CONVERTER_INL

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
    // Parameter Converter Class - Member(s)
    // -------------------------------

    // Convert XmlRpcValue Parameter
    // (Template: Primary/Default)
    // -------------------------------
    template<typename ParamType>
    inline boost::optional<ParamType> ParameterConverter::convertParamType(
        const XmlRpc::XmlRpcValue& param)
    {
        // Report to terminal
        ROS_ERROR_STREAM("Toolbox::ParameterConverter::" << __FUNCTION__
            << ": Failed! Parameter (" << param << ")"
            << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param) << "]"
            << " is unsupporeted for conversion");

        // Unsupported type!
        return boost::none;
    } // Function-End: convertParamType()

    
    // Convert XmlRpcValue Parameter
    // -------------------------------
    // (Template Specialization: Bool)
    template<>
    inline boost::optional<bool> ParameterConverter::convertParamType(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        // (copy input-parameter to remove const to allow type-casting)
        XmlRpc::XmlRpcValue param_ = param;

        // Check parameter against specified type
        if(param_.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
        {
            // Report to terminal
            ROS_ERROR_STREAM("Toolbox::ParameterConverter::" << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param) << "]"
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
    inline boost::optional<int> ParameterConverter::convertParamType(
        const XmlRpc::XmlRpcValue& param) 
    {
        // Local variable(s)
        // (copy input-parameter to remove const to allow type-casting)
        XmlRpc::XmlRpcValue param_ = param;

        // Check parameter against specified type
        if(param_.getType() != XmlRpc::XmlRpcValue::TypeInt)
        {
            // Report to terminal
            ROS_ERROR_STREAM("Toolbox::ParameterConverter::" << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param) << "]"
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
    inline boost::optional<double> ParameterConverter::convertParamType(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        // (copy input-parameter to remove const to allow type-casting)
        XmlRpc::XmlRpcValue param_ = param;

        // Check parameter against specified type
        if(param_.getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
            // Report to terminal
            ROS_ERROR_STREAM("Toolbox::ParameterConverter::" << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param) << "]"
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
    inline boost::optional<std::string> ParameterConverter::convertParamType(
        const XmlRpc::XmlRpcValue& param) 
    {
        // Local variable(s)
        // (copy input-parameter to remove const to allow type-casting)
        XmlRpc::XmlRpcValue param_ = param;

        // Check parameter against specified type
        if(param_.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            // Report to terminal
            ROS_ERROR_STREAM("Toolbox::ParameterConverter::" << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param) << "]"
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
    inline boost::optional<std::vector<bool>> ParameterConverter::convertParamType(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        XmlRpc::XmlRpcValue param_ = param;
        std::vector<bool> result;

        // Check parameter against specified type
        if (param_.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            // Report to terminal
            ROS_ERROR_STREAM("Toolbox::ParameterConverter::" << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param) << "]"
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
                ROS_ERROR_STREAM("Toolbox::ParameterConverter::" << __FUNCTION__
                    << ": Failed! Parameter-Element (" << param_[i] << ")" 
                    << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param_[i]) << "]"
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
    inline boost::optional<std::vector<int>> ParameterConverter::convertParamType(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        XmlRpc::XmlRpcValue param_ = param;
        std::vector<int> result;

        // Check parameter against specified type
        if (param_.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            // Report to terminal
            ROS_ERROR_STREAM("Toolbox::ParameterConverter::" << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param) << "]"
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
                ROS_ERROR_STREAM("Toolbox::ParameterConverter::" << __FUNCTION__
                    << ": Failed! Parameter-Element (" << param_[i] << ")" 
                    << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param_[i]) << "]"
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
    inline boost::optional<std::vector<double>> ParameterConverter::convertParamType(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        XmlRpc::XmlRpcValue param_ = param;
        std::vector<double> result;

        // Check parameter against specified type
        if (param_.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            // Report to terminal
            ROS_ERROR_STREAM("Toolbox::ParameterConverter::" << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param) << "]"
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
                ROS_ERROR_STREAM("Toolbox::ParameterConverter::" << __FUNCTION__
                    << ": Failed! Parameter-Element (" << param_[i] << ")" 
                    << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param_[i]) << "]"
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
    inline boost::optional<std::vector<std::string>> ParameterConverter::convertParamType(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        XmlRpc::XmlRpcValue param_ = param;
        std::vector<std::string> result;

        // Check parameter against specified type
        if (param_.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            // Report to terminal
            ROS_ERROR_STREAM("Toolbox::ParameterConverter::" << __FUNCTION__
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param) << "]"
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
                ROS_ERROR_STREAM("Toolbox::ParameterConverter::" << __FUNCTION__
                    << ": Failed! Parameter-Element (" << param_[i] << ")" 
                    << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param_[i]) << "]"
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
#endif // PARAM_CONVERTER_INL 