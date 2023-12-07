// Robotics Toolbox - Parameter Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Parameter Tools contains helper and utility functions 
//      related to reading and validate parameters
//
// Version:
//  0.1 - Initial Version
//        [19.11.2023]  -   Jan T. Olsen
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
        const std::string& member,
        const bool& err_print)
    {
        // Check parameter for specified member
        if(!param.hasMember(member))
        {
            // Report to terminal
            if(err_print)
            {
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << " Failed! Parameter member [" << member << "] was not found");
            }

            // Function return
            return false;
        }
        
        // Function return
        return true;
    } // Function-End: checkMember()


    // Check and Compare Parameter Type
    // -------------------------------
    bool Parameter::checkType(
        const XmlRpc::XmlRpcValue& param, 
        const XmlRpc::XmlRpcValue::Type& type,
        const bool& err_print)
    {
        // Check parameter against specified type
        if(param.getType() != type)
        {
            // Report to terminal
            if(err_print)
            {
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << " Failed! Parameter types does not match: "
                    << " Input-parameter XmlRpc-Type: [" << getParamTypeName(param.getType()) << "]" 
                    << " vs comparing XmlRpc-Type: [" << getParamTypeName(type) << "]");
            }

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
        const int& size,
        const bool& err_print)
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
                if(err_print)
                {
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        << " Failed! Parameter data-type does not have a size: "
                        << " Input-parameter XmlRpc-Type: [" << getParamTypeName(param.getType()) << "]");
                }

                // Function return
                return false;
        }

        // Check supplied parameter's size against specified size
        if(param.size() != size)
        {
            // Report to terminal
            if(err_print)
            {
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << " Failed! Parameter size does not match: "
                    << " Input-parameter size: [" << param.size() << "]"
                    << " vs comparing size: [" << size << "]");
            }

            // Function return
            return false;
        }

        // Function return
        return true;
    } // Function-End: checkSize()


    // Check Parameter
    // -------------------------------
    // (Function Overloading)
    bool Parameter::checkParameter(
        const XmlRpc::XmlRpcValue& param, 
        const std::string& member, 
        const XmlRpc::XmlRpcValue::Type& type,
        const bool& err_print)
    {
        // Check parameter for specified member
        if(!checkMember(param, member, err_print)) return false;

        // Check parameter-member against specified type
        if(!checkType(param[member], type, err_print)) return false;

        // Function return
        return true;
    } // Function-End: checkParameter()


    // Check Parameter
    // -------------------------------
    // (Function Overloading)
    bool Parameter::checkParameter(
        const XmlRpc::XmlRpcValue& param, 
        const std::string& member, 
        const XmlRpc::XmlRpcValue::Type& type, 
        const int& size,
        const bool& err_print)
    {
        // Check parameter for specified member
        if(!checkMember(param, member, err_print)) return false;

        // Check parameter-member against specified type
        if(!checkType(param[member], type, err_print)) return false;

        // Check parameter-member against specified size
        if(!checkSize(param, size, err_print)) return false;

        // Function return
        return true;
    } // Function-End: checkParameter()


    // Get Name of Parameter-Type 
    // -------------------------------
    std::string Parameter::getParamTypeName(
        const XmlRpc::XmlRpcValue::Type& type)
    {
        // Determine data-type represent it by string  
        switch (type)
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
    } // Function-End: getTypeName()
    
} // End Namespace: Robotics Toolbox