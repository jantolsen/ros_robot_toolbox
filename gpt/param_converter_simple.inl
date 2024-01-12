// Include guard:
// -------------------------------
#ifndef PARAM_CONVERTER_INL
#define PARAM_CONVERTER_INL

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{   
    // Parameter Converter Class - Member(s)
    // -------------------------------

    // Constants
    // -------------------------------
    const std::string ParameterConverter::CLASS_PREFIX = "Toolbox::ParameterConverter::";
    

    // Convert XmlRpcValue Parameter
    // (Template: Primary/Default)
    // -------------------------------
    template<typename ParamType>
    boost::optional<ParamType> ParameterConverter::convertParamXmlValue(
        const XmlRpc::XmlRpcValue& param)
    {
        // Report to terminal
        ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__
            << ": Failed! Parameter (" << param << ")"
            << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param) << "]"
            << " is unsupporeted for conversion");

        // Unsupported type!
        return boost::none;
    } // Function-End: convert()

    
    // Convert XmlRpcValue Parameter
    // -------------------------------
    // (Template Specialization: Bool)
    template<>
    boost::optional<bool> ParameterConverter::convertParamXmlValue(
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
                << ": Failed! Parameter (" << param << ")"
                << " [XmlRpcValueType: " << ParameterTool::getParamTypeName(param) << "]"
                << " does NOT convert to [boolean]");

            // Parameter type mismatch, return false
            return boost::none;
        }
        // Function return
        return static_cast<bool>(param_);
    } // Function-End: convert<bool>()
}