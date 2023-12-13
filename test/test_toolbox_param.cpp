// Test Toolbox Node 
// -------------------------------
// Description:
//      Test Toolbox Node
//
// Version:
//  0.1 - Initial Version
//        [06.01.2023]  -   Jan T. Olsen
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

    // Boost
    #include <boost/optional.hpp>

namespace test
{
    enum KinematicSolverType
    {
        A,
        B,
        C,
        D,
        E,
        F
    };

    // Kinematic Solver Type Map
    // (Matches the solver-types defined in InfoKinematics.msg)
    static std::map<std::string, KinematicSolverType> const kinematicSolverTypeMap =
    {
        {"KDL", KinematicSolverType::A},
        {"OPW", KinematicSolverType::B},
        {"TRACIK", KinematicSolverType::C},
        {"LMA", KinematicSolverType::D},
        {"CACHED_KDL", KinematicSolverType::E},
        {"CACHED_TRACIK", KinematicSolverType::F}
    };

    static std::map<std::string, int> const testMap_keyString =
    {
        {"KDL",1},
        {"OPW",2},
        {"TRACIK",3},
        {"LMA",4},
        {"CACHED_KDL",5},
        {"CACHED_TRACIK",6}
    };

    // Test: Test Param func
    // -------------------------------
    void testParamFunc(XmlRpc::XmlRpcValue& param)
    {
        ROS_INFO(" ");
        ROS_INFO("Param: Test: Bool");
        ROS_INFO("--------------------");
        
            ROS_INFO(" ");
            ROS_INFO("Test: 1");
            ROS_INFO("--------------------");

            // bool test1 = Toolbox::Parameter::getParam<bool>(param, "param_bool1");
            if(auto res = Toolbox::Parameter::getParamBool(param, "param_bool1"))
            {
                int test = *res;
                ROS_INFO_STREAM("Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("Test: 2");
            ROS_INFO("--------------------");

            // bool test1 = Toolbox::Parameter::getParam<bool>(param, "param_bool1");
            if(auto res = Toolbox::Parameter::getParamBool(param, "param_bool22"))
            {
                int test = *res;
                ROS_INFO_STREAM("Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("Results: Failed!");
            }

        ROS_INFO(" ");
        ROS_INFO(" ");
        ROS_INFO("Param: Test: Int");
        ROS_INFO("--------------------");
        
            ROS_INFO(" ");
            ROS_INFO("Test: 1");
            ROS_INFO("--------------------");

            if(auto res = Toolbox::Parameter::getParamInt(param, "param_int1"))
            {
                int test = *res;
                ROS_INFO_STREAM("Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("Test: 2");
            ROS_INFO("--------------------");

            if(auto res = Toolbox::Parameter::getParamInt(param, "param_int2"))
            {
                int test = *res;
                ROS_INFO_STREAM("Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("Results: Failed!");
            }

    } // Function end: testMapGetValues()
    
    XmlRpc::XmlRpcValue testLoadParams()
    {
        XmlRpc::XmlRpcValue params;

        // Assign some parameters
        params["param_bool1"] = false;
        params["param_bool2"] = true;
        params["param_int1"] = 456;
        params["param_int2"] = "0";
        params["param_double1"] = 1.23;
        params["param_double2"] = -0.99;
        params["param_string1"] = "type";
        params["param_string2"] = "TYPE";

        // Return parameters
        return params;
    }
   

} // End: Namespace test

// Test Toolbox Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a ROS Node with a node name
    ros::init(argc, argv, "test_toolbox_map");   

    // Starting ROS Nodehandle(s)
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Test(s)
    // -------------------------------
        XmlRpc::XmlRpcValue params = test::testLoadParams();
        test::testParamFunc(params);

        // int test = static_cast<int>(params["param_string1"]);


    
    // Shutdown
    // -------------------------------
    // ROS-Loop waiting for shutdown
    // ros::waitForShutdown();

    // Function return
    return 0;
}

