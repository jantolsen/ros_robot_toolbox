// Robotics Toolbox - Parameter Converter
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Parameter Converter contains utility functions for conversion 
//      of XmlRpc::XmlRpcValue to fundamental types. 
//      Functionality is implemented as template-class (utilizing struct)
//      and template specialization(s) function(s).
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
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
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
    // // Parameter Converter Class
    // // (Template: Primary/Default)
    // // -------------------------------
    //     // Constants
    //     // (Template Specialization: Primary/Default)
    //     // -------------------------------
    //     template<typename ParamType>
    //     const std::string ParameterConverter<ParamType>::CLASS_PREFIX = "Toolbox::ParameterConverter::";
    

    // Parameter Converter Class
    // (Template Specialization: Bool)
    // -------------------------------
    template<>
    struct ParameterConverter<bool>
    {
        // Convert XmlRpcValue Parameter
        // -------------------------------
        // (Template Specialization: Bool)
        static boost::optional<bool> convert(
            const XmlRpc::XmlRpcValue& param)
        {
            // Local variable(s)
            // (copy input-parameter to remove const to allow type-casting)
            XmlRpc::XmlRpcValue param_ = param;

            // Check parameter against specified type
            if(param_.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
            {
                // Parameter type mismatch, return false
                return boost::none;
            }
            // Function return
            return static_cast<bool>(param_);
        } // Function-End: convert<bool>()
    }; // Class-End: ParameterConverter<bool>()


    // Parameter Converter Class
    // -------------------------------
    // (Template Specialization: Int)
    template<>
    struct ParameterConverter<int>
    {
        // Convert XmlRpcValue Parameter
        // -------------------------------
        // (Template Specialization: Int)
        static boost::optional<int> convert(
            const XmlRpc::XmlRpcValue& param) 
        {
            // Local variable(s)
            // (copy input-parameter to remove const to allow type-casting)
            XmlRpc::XmlRpcValue param_ = param;

            // Check parameter against specified type
            if(param_.getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                // Parameter type mismatch, return false
                return boost::none;
            }

            // Function return
            return static_cast<int>(param_);
        } // Function-End: convert<int>()
    }; // Class-End: ParameterConverter<int>()


    // Parameter Converter Class
    // -------------------------------
    // (Template Specialization: Double)
    template<>
    struct ParameterConverter<double>
    {
        // Convert XmlRpcValue Parameter
        // -------------------------------
        // (Template Specialization: Double)
        static boost::optional<double> convert(
            const XmlRpc::XmlRpcValue& param)
        {
            // Local variable(s)
            // (copy input-parameter to remove const to allow type-casting)
            XmlRpc::XmlRpcValue param_ = param;

            // Check parameter against specified type
            if(param_.getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                // Parameter type mismatch, return false
                return boost::none;
            }
            // Function return
            return static_cast<double>(param_);
        } // Function-End: convert<bool>()
    }; // Class-End: ParameterConverter<bool>()


    // Parameter Converter Class
    // -------------------------------
    // (Template Specialization: String)
    template<>
    struct ParameterConverter<std::string>
    {
        // Convert XmlRpcValue Parameter
        // -------------------------------
        // (Template Specialization: String)
        static boost::optional<std::string> convert(
            const XmlRpc::XmlRpcValue& param) 
        {
            // Local variable(s)
            // (copy input-parameter to remove const to allow type-casting)
            XmlRpc::XmlRpcValue param_ = param;

            // Check parameter against specified type
            if(param_.getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                // Parameter type mismatch, return false
                return boost::none;
            }

            // Function return
            return static_cast<std::string>(param_);
        } // Function-End: convert<std::string>()
    }; // Class-End: ParameterConverter<std::string>()


    // Parameter Converter Class
    // -------------------------------
    // (Template Specialization: std::vector<bool>)
    template<>
    struct ParameterConverter<std::vector<bool>>
    {
        // Convert XmlRpcValue Parameter
        // -------------------------------
        // (Template Specialization: std::vector<bool>)
        static boost::optional<std::vector<bool>> convert(
            const XmlRpc::XmlRpcValue& param)
        {
            // Local variable(s)
            XmlRpc::XmlRpcValue param_ = param;
            std::vector<bool> result;

            // Check parameter against specified type
            if (param_.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
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
                    // Element type mismatch, return false
                    return boost::none;
                }
            }

            // Function return
            return result;
        } // Function-End: convert<std::vector<bool>>()
    }; // Class-End: ParameterConverter<std::vector<bool>()


    // Parameter Converter Class
    // -------------------------------
    // (Template Specialization: std::vector<int>)
    template<>
    struct ParameterConverter<std::vector<int>>
    {
        // Convert XmlRpcValue Parameter
        // -------------------------------
        // (Template Specialization: std::vector<int>)
        static boost::optional<std::vector<int>> convert(
            const XmlRpc::XmlRpcValue& param)
        {
            // Local variable(s)
            XmlRpc::XmlRpcValue param_ = param;
            std::vector<int> result;

            // Check parameter against specified type
            if (param_.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
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
                    // Element type mismatch, return false
                    return boost::none;
                }
            }

            // Function return
            return result;
        } // Function-End: convert<std::vector<int>>()
    }; // Class-End: ParameterConverter<std::vector<int>()


    // Parameter Converter Class
    // -------------------------------
    // (Template Specialization: std::vector<double>)
    template<>
    struct ParameterConverter<std::vector<double>>
    {
        // Convert XmlRpcValue Parameter
        // -------------------------------
        // (Template Specialization: std::vector<double>)
        static boost::optional<std::vector<double>> convert(
            const XmlRpc::XmlRpcValue& param)
        {
            // Local variable(s)
            XmlRpc::XmlRpcValue param_ = param;
            std::vector<double> result;

            // Check parameter against specified type
            if (param_.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
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
                    // Element type mismatch, return false
                    return boost::none;
                }
            }

            // Function return
            return result;
        } // Function-End: convert<std::vector<double>>()
    }; // Class-End: ParameterConverter<std::vector<double>>()


    // Parameter Converter Class
    // -------------------------------
    // (Template Specialization: std::vector<string>)
    template<>
    struct ParameterConverter<std::vector<std::string>>
    {
        // Convert XmlRpcValue Parameter
        // -------------------------------
        // (Template Specialization: std::vector<std::string>)
        static boost::optional<std::vector<std::string>> convert(
            const XmlRpc::XmlRpcValue& param)
        {
            // Local variable(s)
            XmlRpc::XmlRpcValue param_ = param;
            std::vector<std::string> result;

            // Check parameter against specified type
            if (param_.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
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
                    // Element type mismatch, return false
                    return boost::none;
                }
            }
            // Function return
            return result;
        } // Function-End: convert<std::vector<std::string>>()
    }; // Class-End: ParameterConverter<std::vector<std::string>>()


} // End of namespace "Toolbox"
#endif // PARAM_CONVERTER_INL 