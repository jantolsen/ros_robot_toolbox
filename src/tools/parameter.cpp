// Robotics Toolbox - Parameter Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Parameter Tools contains helper and utility functions 
//      related to loading, reading and validating parameter-data
//      parameter-data obtained from parameter-server.
//
// Version:
//  0.2 -   Overual of Parameter Tools implementation.
//          Introduced inline implementation.
//          [12.01.2024]  -   Jan T. Olsen
//  0.1 -   Initial Version
//          [19.11.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robot_toolbox/tools/parameter.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
    // Parameter Tool Class - Members:
    // -------------------------------

    // Constants
    // -------------------------------
    const std::string Parameter::CLASS_PREFIX = "Toolbox::Parameter::";


    // Check Parameter Member
    // -------------------------------
    bool Parameter::checkMember(
        const XmlRpc::XmlRpcValue& param, 
        const std::string& member)
    {
        // Check parameter for specified member
        if(!param.hasMember(member))
        {
            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given Parameter member [" << member << "] was NOT found");

            // Function return
            return false;
        }
        
        // Function return
        return true;
    } // Function-End: checkMember()


    // Check and Compare Parameter Type
    // -------------------------------
    bool Parameter::checkDataType(
        const XmlRpc::XmlRpcValue& param, 
        const XmlRpc::XmlRpcValue::Type& type)
    {
        // Check parameter against specified type
        if(param.getType() != type)
        {
            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given Parameter data-type [" << getParamTypeName(param.getType()) << "]"
                << " does NOT match comparing type [" << getParamTypeName(type) << "]");

            // Function return
            return false;
        }

        // Function return
        return true;
    } // Function-End: checkType()


    // Check and Compare Parameter Size
    // -------------------------------
    bool Parameter::checkSize(
        const XmlRpc::XmlRpcValue& param, 
        const int& size)
    {
        // Determine that the supplied parameter has a size
        // by checking the data-type
        switch (param.getType())
        {
            // Valid: Data-type with a size
            case XmlRpc::XmlRpcValue::TypeString:
            case XmlRpc::XmlRpcValue::TypeBase64:
            case XmlRpc::XmlRpcValue::TypeArray:
            case XmlRpc::XmlRpcValue::TypeStruct:
                break;
            // Invalid: Data-type does not have size
            default:
                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Given Parameter data-type does NOT have a size "
                    << " [" << getParamTypeName(param.getType()) << "]");

                // Function return
                return false;
        }

        // Check supplied parameter's size against specified size
        if(param.size() != size)
        {
            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given Parameter size [" << param.size() << "]" 
                << " does NOT match comparing size [" << size << "]");

            // Function return
            return false;
        }

        // Function return
        return true;
        
    } // Function-End: checkSize()


    // Get Name of Parameter Data-Type 
    // -------------------------------
    std::string Parameter::getParamTypeName(
        const XmlRpc::XmlRpcValue::Type& param_type)
    {
        // Determine data-type and represent it as string  
        switch (param_type)
        {
            // Boolean
            case XmlRpc::XmlRpcValue::TypeBoolean:
                // Function return
                return "Type-Boolean";
            // Integer
            case XmlRpc::XmlRpcValue::TypeInt:
                // Function return
                return "Type-Int";
            // Double
            case XmlRpc::XmlRpcValue::TypeDouble:
                // Function return
                return "Type-Double";
            // String
            case XmlRpc::XmlRpcValue::TypeString:
                // Function return
                return "Type-String";
            // Date-Time
            case XmlRpc::XmlRpcValue::TypeDateTime:
                // Function return
                return "Type-DateTime";
            // Base64/Bytes
            case XmlRpc::XmlRpcValue::TypeBase64:
                // Function return
                return "Type-Base64/Bytes";
            // Array
            case XmlRpc::XmlRpcValue::TypeArray:
                // Function return
                return "Type-Array";
            // Struct
            case XmlRpc::XmlRpcValue::TypeStruct:
                // Function return
                return "Type-Struct";
            // Invalid
            case XmlRpc::XmlRpcValue::TypeInvalid:
            default:
                // Function return
                return "Type-Invalid";
        }
    } // Function-End: getParamTypeName()


    // Get Name of Parameter Data-Type 
    // -------------------------------
    std::string Parameter::getParamTypeName(
        const XmlRpc::XmlRpcValue& param)
    {
        // Call overloading function
        return getParamTypeName(param.getType());
    } // Function-End: getParamTypeName()

} // End Namespace: Robotics Toolbox