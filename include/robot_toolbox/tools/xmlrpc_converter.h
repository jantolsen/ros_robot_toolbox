// Robotics Toolbox - XmlRpc Converter
// -------------------------------
// Description:
//      XmlRpc Value Convert template file 
//      Contains template specialization(s) for converting
//      XmlRpcValue to a fundemental type
//
// Version:
//  0.1 - Initial Version
//        [17.12.2023]  -   Jan T. Olsen
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
#ifndef XMLRPC_CONVERTER_H       
#define XMLRPC_CONVERTER_H

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
    // XmlRpcValue Converter
    // (Specialization Primary/Default)
    // -------------------------------
    // (Template Specialization)
    template<typename ParamType>
    struct XmlRpcValueConverter
    {
        // Convert XmlRpcValue Parameter
        // -------------------------------
        /** \brief Convert XmlRpcValue to fundemental type.
        *
        * Parameter is checked for data-type and explicitly cast to a fundamental type.
        * Invalid data-type will result in error message and function return false
        * 
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * 
        * \return Function return: Successful: parameter value [ParamType] / Unsuccessful: false [bool]
        */
        static boost::optional<ParamType> convert(
            const XmlRpc::XmlRpcValue& param) 
        {
            // Unsupported type!
            return boost::none;
        } // Function-End: convert()
    }; // Struct-End: XmlRpcValueConverter()


    // XmlRpcValue Converter
    // (Specialization for Bool)
    // -------------------------------
    // (Template Specialization)
    template<>
    struct XmlRpcValueConverter<bool>
    {
        // Convert XmlRpcValue Parameter
        // -------------------------------
        /** \brief Convert XmlRpcValue to fundemental type.
        *
        * Parameter is checked for data-type and explicitly cast to a fundamental type.
        * Invalid data-type will result in error message and function return false
        * 
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * 
        * \return Function return: Successful: parameter value [ParamType] / Unsuccessful: false [bool]
        */
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
    }; // Struct-End: XmlRpcValueConverter<bool>()


    // XmlRpcValue Converter
    // (Specialization for Int)
    // -------------------------------
    // (Template Specialization)
    template<>
    struct XmlRpcValueConverter<int>
    {
        // Convert XmlRpcValue Parameter
        // -------------------------------
        /** \brief Convert XmlRpcValue to fundemental type.
        *
        * Parameter is checked for data-type and explicitly cast to a fundamental type.
        * Invalid data-type will result in error message and function return false
        * 
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * 
        * \return Function return: Successful: parameter value [ParamType] / Unsuccessful: false [bool]
        */
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
    }; // Struct-End: XmlRpcValueConverter<int>()


    // XmlRpcValue Converter
    // (Specialization for Double)
    // -------------------------------
    // (Template Specialization)
    template<>
    struct XmlRpcValueConverter<double>
    {
        // Convert XmlRpcValue Parameter
        // -------------------------------
        /** \brief Convert XmlRpcValue to fundemental type.
        *
        * Parameter is checked for data-type and explicitly cast to a fundamental type.
        * Invalid data-type will result in error message and function return false
        * 
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * 
        * \return Function return: Successful: parameter value [ParamType] / Unsuccessful: false [bool]
        */
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
        } // Function-End: convert<double>()
    }; // Struct-End: XmlRpcValueConverter<double>()


    // XmlRpcValue Converter
    // (Specialization for String)
    // -------------------------------
    // (Template Specialization)
    template<>
    struct XmlRpcValueConverter<std::string>
    {
        // Convert XmlRpcValue Parameter
        // -------------------------------
        /** \brief Convert XmlRpcValue to fundemental type.
        *
        * Parameter is checked for data-type and explicitly cast to a fundamental type.
        * Invalid data-type will result in error message and function return false
        * 
        * \param param  Parameter to be converted [XmlRpc::XmlRpcValue]
        * 
        * \return Function return: Successful: parameter value [ParamType] / Unsuccessful: false [bool]
        */
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
    }; // Struct-End: XmlRpcValueConverter<std::string>()

} // End Namespace: Robotics Toolbox
#endif // XMLRPC_CONVERTER_H 