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
                << ": Failed! Given Parameter data-type [" << getDataTypeName(param.getType()) << "]"
                << " does NOT match comparing type [" << getDataTypeName(type) << "]");

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
                    << " [" << getDataTypeName(param.getType()) << "]");

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


    // Check Parameter
    // -------------------------------
    // (Function Overloading)
    bool Parameter::checkParameter(
        const XmlRpc::XmlRpcValue& param, 
        const std::string& member, 
        const XmlRpc::XmlRpcValue::Type& type)
    {
        // Check parameter for specified member
        if(!checkMember(param, member)) return false;

        // Check parameter-member against specified type
        if(!checkDataType(param[member], type)) return false;

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
        const int& size)
    {
        // Check parameter for specified member
        if(!checkMember(param, member)) return false;

        // Check parameter-member against specified type
        if(!checkDataType(param[member], type)) return false;

        // Check parameter-member against specified size
        if(!checkSize(param, size)) return false;

        // Function return
        return true;
    } // Function-End: checkParameter()


    // Get Parameter Data: Bool 
    // -------------------------------
    boost::optional<bool> Parameter::getParamBool(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_member)
    {
        // Check for parameter-member in given parameter-data
        if(!checkMember(param_xml, param_member))
        {
            // Parameter validation failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter [" << param_member <<"] is missing");

            // Function return
            return boost::none;
        } 

        // Convert parameter-member to bool-type
        boost::optional<bool> result = castParamToBool(param_xml[param_member]);
        if(!result)
        {
            // Parameter validation failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter [" << param_member <<"] is configured incorrectly");

            // Function return
            return boost::none;
        } 
       
        // Function return
        return result;
    } // Function-End: getParamBool()


    // Get Parameter Data: Int 
    // -------------------------------
    boost::optional<int> Parameter::getParamInt(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_member)
    {
        // Check for parameter-member in given parameter-data
        if(!checkMember(param_xml, param_member))
        {
            // Parameter validation failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter [" << param_member <<"] is missing");

            // Function return
            return boost::none;
        } 

        // Convert parameter-member to int-type
        boost::optional<int> result = castParamToInt(param_xml[param_member]);
        if(!result)
        {
            // Parameter validation failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter [" << param_member <<"] is configured incorrectly");

            // Function return
            return boost::none;
        } 
       
        // Function return
        return result;
    } // Function-End: getParamInt()


    // Get Parameter Data: Double 
    // -------------------------------
    boost::optional<double> Parameter::getParamDouble(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_member)
    {
        // Check for parameter-member in given parameter-data
        if(!checkMember(param_xml, param_member))
        {
            // Parameter validation failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter [" << param_member <<"] is missing");

            // Function return
            return boost::none;
        } 

        // Convert parameter-member to string-type
        boost::optional<double> result = castParamToDouble(param_xml[param_member]);
        if(!result)
        {
            // Parameter validation failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter [" << param_member <<"] is configured incorrectly");

            // Function return
            return boost::none;
        } 
       
        // Function return
        return result;
    } // Function-End: getParamDouble()


    // Get Parameter Data: String 
    // -------------------------------
    boost::optional<std::string> Parameter::getParamString(
        const XmlRpc::XmlRpcValue& param_xml,
        const std::string& param_member)
    {
        // Check for parameter-member in given parameter-data
        if(!checkMember(param_xml, param_member))
        {
            // Parameter validation failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter [" << param_member <<"] is missing");

            // Function return
            return boost::none;
        } 

        // Convert parameter-member to string-type
        boost::optional<std::string> result = castParamToString(param_xml[param_member]);
        if(!result)
        {
            // Parameter validation failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter [" << param_member <<"] is configured incorrectly");

            // Function return
            return boost::none;
        } 
       
        // Function return
        return result;
    } // Function-End: getParamString()


    // Convert Parameter Data to Bool-Value 
    // -------------------------------
    boost::optional<bool> Parameter::castParamToBool(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        // (copy input-parameter to remove const to allow type-casting)
        XmlRpc::XmlRpcValue param_ = param;

        // Check parameter against specified type
        if(param_.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
        {
            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__
                << ": Failed! Parameter [" << param_ << "]"
                << " (XmlRpcType: " << getDataTypeName(param.getType()) << ")"
                << " does NOT convert to a bool-type");

            // Function return
            return boost::none;
        }

        // Function return
        return static_cast<bool>(param_);
    } // Function-End: castParamToBool()

    
    // Convert Parameter Data to Int-Value 
    // -------------------------------
    boost::optional<int> Parameter::castParamToInt(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        // (copy input-parameter to remove const to allow type-casting)
        XmlRpc::XmlRpcValue param_ = param;

        // Check parameter against specified type
        if(param_.getType() != XmlRpc::XmlRpcValue::TypeInt)
        {
            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__
                << ": Failed! Parameter [" << param_ << "]"
                << " (XmlRpcType: " << getDataTypeName(param.getType()) << ")"
                << " does NOT convert to an int-type");

            // Function return
            return boost::none;
        }

        // Function return
        return static_cast<int>(param_);
    } // Function-End: castParamToInt()


     // Convert Parameter Data to Double-Value 
    // -------------------------------
    boost::optional<double> Parameter::castParamToDouble(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        // (copy parameter to remove const to allow type-casting)
        XmlRpc::XmlRpcValue param_ = param;

        // Check parameter against specified type
        if(param_.getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__
                << ": Failed! Parameter [" << param_ << "]"
                << " (XmlRpcType: " << getDataTypeName(param.getType()) << ")"
                << " does NOT convert to a double-type");

            // Function return
            return boost::none;
        }

        // Function return
        return static_cast<double>(param_);
    } // Function-End: castParamToDouble()


    // Convert Parameter Data to String-Value
    // -------------------------------
    boost::optional<std::string> Parameter::castParamToString(
        const XmlRpc::XmlRpcValue& param)
    {
        // Local variable(s)
        // (copy input-parameter to remove const to allow type-casting)
        XmlRpc::XmlRpcValue param_ = param;

        // Check parameter against specified type
        if(param_.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__
                << ": Failed! Parameter [" << param_ << "]"
                << " (XmlRpcType: " << getDataTypeName(param.getType()) << ")"
                << " does NOT convert to a string-type");

            // Function return
            return boost::none;
        }

        // Function return
        return static_cast<std::string>(param_);
    } // Function-End: castParamToString()

    
    // Get Name of Parameter Data-Type
    // -------------------------------
    std::string Parameter::getDataTypeName(
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