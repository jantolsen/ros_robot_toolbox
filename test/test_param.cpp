// Test Toolbox Node - Parameter
// -------------------------------
// Description:
//      Test Toolbox Node
//
// Version:
//  0.1 - Initial Version
//        [13.01.2024]  -   Jan T. Olsen
//
// -------------------------------


// Include Header-files:
// -------------------------------
    // Standard
    #include <memory>
    #include <iostream>
    #include <string>

    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include "robot_toolbox/toolbox.h"


// Test: Enum(s)
// -------------------------------
    // Test Type
    enum TestType
    {
        A,
        B,
        C,
        D,
        E,
        F
    };

// Test: Map(s)
// -------------------------------
    // Test Type Map
    static std::map<std::string, TestType> const test_map =
    {
        {"BLUE",    TestType::A},
        {"RED",     TestType::B},
        {"YELLOW",  TestType::C},
        {"GREEN",   TestType::D},
    };

// Test: Function(s)
// -------------------------------

    // Load Parameter file
    // -------------------------------
    XmlRpc::XmlRpcValue loadParameter(
        const std::string& param_name)
    {
        // Define local variable(s)
        XmlRpc::XmlRpcValue param_xml;
        
        // Check parameter server for Target parameters
        if(!ros::param::get(param_name, param_xml))
        {
            // Failed to get parameter
            ROS_ERROR_STREAM(__FUNCTION__ 
                <<  ": Failed! Target Parameter [" << param_name << "] is NOT found");
        }

        // Function return
        return param_xml; 
    }

    // Initialize Validation Set 
    // -------------------------------
    std::vector<std::string> initValidationSet()
    {
        // Initialize validation-set vector
        std::vector<std::string> validation_set = {"RED", "BLUE", "GREEN"};
        
        // Function return
        return validation_set;
    }

// Test Toolbox Node - Parameter 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a ROS Node with a node name
    ros::init(argc, argv, "test_toolbox_parameter");   

    // Starting ROS Nodehandle(s)
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Test(s)
    // -------------------------------
        // Validation set
        std::vector<std::string> validation_set = initValidationSet();
        
        // Load parameters
        XmlRpc::XmlRpcValue param_xml = loadParameter("test_params");


    // Convert
    // -------------------------------
        // Convert parameter(s)
        // auto param_color = Toolbox::ParameterConverter::convertParamType<std::string>(param_xml["color"]);


    // Loading
    // -------------------------------
        // Load parameter(s)
        auto param_bool = Toolbox::ParameterLoader::loadParamData<bool>(param_xml, "value_bool");
        auto param_int = Toolbox::ParameterLoader::loadParamData<int>(param_xml, "value_int");
        auto param_double = Toolbox::ParameterLoader::loadParamData<double>(param_xml, "value_double");
        auto param_string = Toolbox::ParameterLoader::loadParamData<std::string>(param_xml, "value_string");
        auto param_color = Toolbox::ParameterLoader::loadParamData<std::string>(param_xml, "color", validation_set);
        auto param_vec = Toolbox::ParameterLoader::loadParamData<std::vector<int>>(param_xml, "value_vec");
        auto param_map = Toolbox::ParameterLoader::loadParamData<int>(param_xml, "map_key" , test_map);
        


    // Print Result
    // -------------------------------
        ROS_INFO_STREAM(" ");
        ROS_INFO_STREAM("Test Parameters:");
        ROS_INFO_STREAM("--------------------");
        ROS_INFO_STREAM("Bool: "    << param_bool.value());
        ROS_INFO_STREAM("Int: "     << param_int.value());
        ROS_INFO_STREAM("Double: "  << param_double.value());
        ROS_INFO_STREAM("String: "  << param_string.value());
        ROS_INFO_STREAM("Color: "   << param_color.value());

        ROS_INFO_STREAM("Vector: ");
        for (int i = 0; i < param_vec.value().size(); i++)
        {
            ROS_INFO_STREAM("   Item [" << i << "]: " << param_vec.value()[i]);
        }

        ROS_INFO_STREAM("Color-Map: "   << param_map.value());
        


    // // Main Loop
    // // -------------------------------
    // ros::Rate rate(10.0);
    // while (ros::ok()) 
    // {
    //     // Logic

    //     ros::spinOnce(); // Handle ROS callbacks
    // }
    
    // // Spin to keep the node alive
    // ros::spin();
    
    // Shutdown
    // -------------------------------
    // ROS-Loop waiting for shutdown
    ros::waitForShutdown();

    // Function return
    return 0;
}

