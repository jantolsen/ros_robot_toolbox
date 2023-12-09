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
    static std::map<std::string, KinematicSolverType, Toolbox::MapCaseInsensitiveComparator> const kinematicSolverTypeMap =
    {
        {"KDL", KinematicSolverType::A},
        {"OPW", KinematicSolverType::B},
        {"TRACIK", KinematicSolverType::C},
        {"LMA", KinematicSolverType::D},
        {"CACHED_KDL", KinematicSolverType::E},
        {"CACHED_TRACIK", KinematicSolverType::F}
    };


    static std::map<int, std::string> const testMap_keyInt =
    {
        {1, "KDL"},
        {2, "OPW"},
        {3, "TRACIK"},
        {4, "LMA"},
        {5, "CACHED_KDL"},
        {6, "CACHED_TRACIK"}
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

    static std::map<KinematicSolverType, std::string> const testMap_keyEnum =
    {
        {KinematicSolverType::A, "KDL"},
        {KinematicSolverType::B, "OPW"},
        {KinematicSolverType::C, "TRACIK"},
        {KinematicSolverType::D, "LMA"},
        {KinematicSolverType::E, "CACHED_KDL"},
        {KinematicSolverType::F, "CACHED_TRACIK"}
    };

    // Test: Map Get Values
    // (Get Values in Maps using Keys)
    // -------------------------------
    void testMapGetValues()
    {
        ROS_INFO("Map: Search value using key:");
        ROS_INFO("--------------------");
        
        ROS_INFO(" ");
        ROS_INFO("Test-Map Key-Int:");
        ROS_INFO("--------------------");

            ROS_INFO("Test: 1");
            int key_int = 2;
            std::string value_str;
            if(Toolbox::Common::mapGetValue(key_int, test::testMap_keyInt, value_str))
            {
                ROS_INFO_STREAM("Key Found: " << key_int);
                ROS_INFO_STREAM("Related Value: " << value_str);
            }
            else
            {
                ROS_ERROR_STREAM("Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("Value: NOT FOUND");
            }
            ROS_INFO(" ");
            ROS_INFO("Test: 2");
            key_int = 22;
            if(Toolbox::Common::mapGetValue(key_int, test::testMap_keyInt, value_str))
            {
                ROS_INFO_STREAM("Key Found: " << key_int);
                ROS_INFO_STREAM("Related Value: " << value_str);
            }
            else
            {
                ROS_ERROR_STREAM("Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("Value: NOT FOUND");
            }


        ROS_INFO(" ");
        ROS_INFO("Test-Map Key-String:");
        ROS_INFO("--------------------");

            ROS_INFO("Test: 1");
            std::string key_str = "LMA";
            int value_int;
            if(Toolbox::Common::mapGetValue(key_str, test::testMap_keyString, value_int))
            {
                ROS_INFO_STREAM("Key Found: " << key_str);
                ROS_INFO_STREAM("Related Value: " << value_int);
            }
            else
            {
                ROS_ERROR_STREAM("Key NOT Found: " << key_str);
                ROS_ERROR_STREAM("Value: NOT FOUND");
            }
            ROS_INFO(" ");
            ROS_INFO("Test: 2");
            key_str = "lma";
            if(Toolbox::Common::mapGetValue(key_str, test::testMap_keyString, value_int, false))
            {
                ROS_INFO_STREAM("Key Found: " << key_str);
                ROS_INFO_STREAM("Related Value: " << value_int);
            }
            else
            {
                ROS_ERROR_STREAM("Key NOT Found: " << key_str);
                ROS_ERROR_STREAM("Value: NOT FOUND");
            }

        ROS_INFO(" ");
        ROS_INFO("Test-Map Key-Enum:");
        ROS_INFO("--------------------");
        
            ROS_INFO("Test: 1");
            test::KinematicSolverType key_enum = test::KinematicSolverType::B;
            if(Toolbox::Common::mapGetValue(key_enum, test::testMap_keyEnum, value_str))
            {
                ROS_INFO_STREAM("Key Found: " << key_enum);
                ROS_INFO_STREAM("Related Value: " << value_str);
            }
            else
            {
                ROS_ERROR_STREAM("Key NOT Found: " << key_enum);
                ROS_ERROR_STREAM("Value: NOT FOUND");
            }
            ROS_INFO(" ");
            ROS_INFO("Test: 2");
            key_enum = static_cast<test::KinematicSolverType>(22);
            if(Toolbox::Common::mapGetValue(key_enum, test::testMap_keyEnum, value_str))
            {
                ROS_INFO_STREAM("Key Found: " << key_enum);
                ROS_INFO_STREAM("Related Value: " << value_str);
            }
            else
            {
                ROS_ERROR_STREAM("Key NOT Found: " << key_enum);
                ROS_ERROR_STREAM("Value: NOT FOUND");
            }


        ROS_INFO(" ");
        ROS_INFO("Test-Map KinematicSolverType:");
        ROS_INFO("--------------------");
        
            ROS_INFO("Test: 1");
            key_str = "LMA";
            test::KinematicSolverType value_enum;
            if(Toolbox::Common::mapGetValue(key_str, test::kinematicSolverTypeMap, value_enum))
            {
                ROS_INFO_STREAM("Key Found: " << key_str);
                ROS_INFO_STREAM("Related Value: " << value_enum);
            }
            else
            {
                ROS_ERROR_STREAM("Key NOT Found: " << key_str);
                ROS_ERROR_STREAM("Value: NOT FOUND");
            }
            ROS_INFO(" ");
            ROS_INFO("Test: 2");
            key_str = "opw";
            if(Toolbox::Common::mapGetValue(key_str, test::kinematicSolverTypeMap, value_enum))
            {
                ROS_INFO_STREAM("Key Found: " << key_str);
                ROS_INFO_STREAM("Related Value: " << value_str);
            }
            else
            {
                ROS_ERROR_STREAM("Key NOT Found: " << key_str);
                ROS_ERROR_STREAM("Value: NOT FOUND");
            }
    } // Function end: testMapGetValues()
    

    // Test: Map Get Keys
    // (Get Keys in Maps using Values)
    // -------------------------------
    void testMapGetKeys()
    {
        ROS_INFO("Map: Search key using value:");
        ROS_INFO("--------------------");
        
        ROS_INFO(" ");
        ROS_INFO("Test-Map Value-Int:");
        ROS_INFO("--------------------");

            ROS_INFO("Test: 1");
            int value_int = 2;
            std::string key_str;
            if(Toolbox::Common::mapGetKey(value_int, test::testMap_keyString, key_str))
            {
                ROS_INFO_STREAM("Value Found: " << value_int);
                ROS_INFO_STREAM("Related Key: " << key_str);
            }
            else
            {
                ROS_ERROR_STREAM("Value NOT Found: " << value_int);
                ROS_ERROR_STREAM("Key: NOT FOUND");
            }
            ROS_INFO(" ");
            ROS_INFO("Test: 2");
            value_int = 22;
            if(Toolbox::Common::mapGetKey(value_int, test::testMap_keyString, key_str))
            {
                ROS_INFO_STREAM("Value Found: " << value_int);
                ROS_INFO_STREAM("Related Key: " << key_str);
            }
            else
            {
                ROS_ERROR_STREAM("Value NOT Found: " << value_int);
                ROS_ERROR_STREAM("Key: NOT FOUND");
            }


        ROS_INFO(" ");
        ROS_INFO("Test-Map Value-String:");
        ROS_INFO("--------------------");

            ROS_INFO("Test: 1");
            std::string value_str = "LMA";
            int key_int;
            if(Toolbox::Common::mapGetKey(value_str, test::testMap_keyInt, key_int))
            {
                ROS_INFO_STREAM("Value Found: " << value_str);
                ROS_INFO_STREAM("Related Key: " << key_int);
            }
            else
            {
                ROS_ERROR_STREAM("Value NOT Found: " << value_str);
                ROS_ERROR_STREAM("Key: NOT FOUND");
            }
            ROS_INFO(" ");
            ROS_INFO("Test: 2");
            value_str = "lma";
            if(Toolbox::Common::mapGetKey(value_str, test::testMap_keyInt, key_int))
            {
                ROS_INFO_STREAM("Value Found: " << value_str);
                ROS_INFO_STREAM("Related Key: " << key_int);
            }
            else
            {
                ROS_ERROR_STREAM("Value NOT Found: " << value_str);
                ROS_ERROR_STREAM("Key: NOT FOUND");
            }

        ROS_INFO(" ");
        ROS_INFO("Test-Map Key-Enum:");
        ROS_INFO("--------------------");
        
            ROS_INFO("Test: 1");
            value_str = "LMA";
            test::KinematicSolverType key_enum;
            if(Toolbox::Common::mapGetKey(value_str, test::testMap_keyEnum, key_enum))
            {
                ROS_INFO_STREAM("Value Found: " << value_str);
                ROS_INFO_STREAM("Related Key: " << key_enum);
            }
            else
            {
                ROS_ERROR_STREAM("Value NOT Found: " << value_str);
                ROS_ERROR_STREAM("Key: NOT FOUND");
            }
            ROS_INFO(" ");
            ROS_INFO("Test: 2");
            value_str = "asd";
            if(Toolbox::Common::mapGetKey(value_str, test::testMap_keyEnum, key_enum))
            {
                ROS_INFO_STREAM("Value Found: " << value_str);
                ROS_INFO_STREAM("Related Key: " << key_enum);
            }
            else
            {
                ROS_ERROR_STREAM("Value NOT Found: " << value_str);
                ROS_ERROR_STREAM("Key: NOT FOUND");
            }


        ROS_INFO(" ");
        ROS_INFO("Test-Map KinematicSolverType:");
        ROS_INFO("--------------------");
        
            ROS_INFO("Test: 1");
            key_str;
            test::KinematicSolverType value_enum = test::KinematicSolverType::D;
            if(Toolbox::Common::mapGetKey(value_enum, test::kinematicSolverTypeMap, key_str))
            {
                ROS_INFO_STREAM("Value Found: " << value_enum);
                ROS_INFO_STREAM("Related Key: " << key_str);
            }
            else
            {
                ROS_ERROR_STREAM("Value NOT Found: " << value_enum);
                ROS_ERROR_STREAM("Key: NOT FOUND");
            }
            ROS_INFO(" ");
            ROS_INFO("Test: 2");
            value_enum = static_cast<test::KinematicSolverType>(83);
            if(Toolbox::Common::mapGetKey(value_enum, test::kinematicSolverTypeMap, key_str))
            {
                ROS_INFO_STREAM("Value Found: " << value_enum);
                ROS_INFO_STREAM("Related Key: " << key_str);
            }
            else
            {
                ROS_ERROR_STREAM("Value NOT Found: " << value_enum);
                ROS_ERROR_STREAM("Key: NOT FOUND");
            }
    } // Function end: testMapGetKeys()


    // Test: Search in Type-Map(s)
    // (Get related key/value by searching in type-map)
    // -------------------------------
    void testSearchTypeMap()
    {
        ROS_INFO("Type Map: Search in type-map to find correlated item:");
        ROS_INFO("--------------------");
        
        ROS_INFO(" ");
        ROS_INFO("Type Map: Key [String] Value [Int]:");
        ROS_INFO("--------------------");

            int search_int;
            int res_int;
            std::string search_str;
            std::string res_str;

            ROS_INFO("Test 1: Find Value by Key");
            search_str = "tracik";
            if(Toolbox::Parameter::searchTypeMap(search_str, test::testMap_keyString, res_int))
            {
                ROS_INFO_STREAM("Search Item Found: " << search_str);
                ROS_INFO_STREAM("Resulting Item: " << res_int);
            }
            else
            {
                ROS_ERROR_STREAM("Search Item NOT Found: " << search_str);
                ROS_ERROR_STREAM("Resulting Item: NOT FOUND");
            }

            ROS_INFO(" ");
            ROS_INFO("Test 2: Find Value by Key");
            search_str = "sadsad";
            if(Toolbox::Parameter::searchTypeMap(search_str, test::testMap_keyString, res_int))
            {
                ROS_INFO_STREAM("Search Item Found: " << search_str);
                ROS_INFO_STREAM("Resulting Item: " << res_int);
            }
            else
            {
                ROS_ERROR_STREAM("Search Item NOT Found: " << search_str);
                ROS_ERROR_STREAM("Resulting Item: NOT FOUND");
            }

            ROS_INFO(" ");
            ROS_INFO("Test 3: Find Key by Value");
            search_int = 4;
            if(Toolbox::Parameter::searchTypeMap(search_int, test::testMap_keyString, res_str))
            {
                ROS_INFO_STREAM("Search Item Found: " << search_int);
                ROS_INFO_STREAM("Resulting Item: " << res_str);
            }
            else
            {
                ROS_ERROR_STREAM("Search Item NOT Found: " << search_int);
                ROS_ERROR_STREAM("Resulting Item: NOT FOUND");
            }

            ROS_INFO(" ");
            ROS_INFO("Test 4: Find Key by Value");
            search_int = 44;
            if(Toolbox::Parameter::searchTypeMap(search_int, test::testMap_keyString, res_str))
            {
                ROS_INFO_STREAM("Search Item Found: " << search_int);
                ROS_INFO_STREAM("Resulting Item: " << res_str);
            }
            else
            {
                ROS_ERROR_STREAM("Search Item NOT Found: " << search_int);
                ROS_ERROR_STREAM("Resulting Item: NOT FOUND");
            }

        ROS_INFO(" ");
        ROS_INFO("Type Map: Key [KinematicSolverType] Value [String]:");
        ROS_INFO("--------------------");

            KinematicSolverType search_enum;
            KinematicSolverType res_enum;
            search_str;
            res_str;

            ROS_INFO("Test 1: Find Value by Key");
            search_enum = static_cast<KinematicSolverType>(4);
            if(Toolbox::Parameter::searchTypeMap(search_enum, test::testMap_keyEnum, res_str))
            {
                ROS_INFO_STREAM("Search Item Found: " << search_enum);
                ROS_INFO_STREAM("Resulting Item: " << res_str);
            }
            else
            {
                ROS_ERROR_STREAM("Search Item NOT Found: " << search_enum);
                ROS_ERROR_STREAM("Resulting Item: NOT FOUND");
            }

            ROS_INFO(" ");
            ROS_INFO("Test 2: Find Value by Key");
            search_enum = KinematicSolverType::E;
            if(Toolbox::Parameter::searchTypeMap(search_enum, test::testMap_keyEnum, res_str))
            {
                ROS_INFO_STREAM("Search Item Found: " << search_enum);
                ROS_INFO_STREAM("Resulting Item: " << res_str);
            }
            else
            {
                ROS_ERROR_STREAM("Search Item NOT Found: " << search_enum);
                ROS_ERROR_STREAM("Resulting Item: NOT FOUND");
            }

            ROS_INFO(" ");
            ROS_INFO("Test 3: Find Key by Value");
            search_str = "KDL";
            if(Toolbox::Parameter::searchTypeMap(search_str, test::testMap_keyEnum, res_enum))
            {
                ROS_INFO_STREAM("Search Item Found: " << search_str);
                ROS_INFO_STREAM("Resulting Item: " << res_enum);
            }
            else
            {
                ROS_ERROR_STREAM("Search Item NOT Found: " << search_str);
                ROS_ERROR_STREAM("Resulting Item: NOT FOUND");
            }

            ROS_INFO(" ");
            ROS_INFO("Test 4: Find Key by Value");
            search_int = 44;
            // if(Toolbox::Parameter::searchTypeMap(static_cast<KinematicSolverType>(22), test::testMap_keyEnum, res_enum))
            if(Toolbox::Parameter::searchTypeMap(search_int, test::testMap_keyEnum, res_enum))
            {
                ROS_INFO_STREAM("Search Item Found: " << search_int);
                ROS_INFO_STREAM("Resulting Item: " << res_enum);
            }
            else
            {
                ROS_ERROR_STREAM("Search Item NOT Found: " << search_int);
                ROS_ERROR_STREAM("Resulting Item: NOT FOUND");
            }

        ROS_INFO(" ");
        ROS_INFO("Test-Map KinematicSolverType:");
        ROS_INFO("Type Map: Value [String] Key [KinematicSolverType]:");
        ROS_INFO("--------------------");
        
            ROS_INFO("Test 1: Find Value by Key");
            search_str = "Cached_kdl";
            if(Toolbox::Parameter::searchTypeMap(search_str, test::kinematicSolverTypeMap, res_enum))
            {
                ROS_INFO_STREAM("Search Item Found: " << search_str);
                ROS_INFO_STREAM("Resulting Item: " << res_enum);
            }
            else
            {
                ROS_ERROR_STREAM("Search Item NOT Found: " << search_str);
                ROS_ERROR_STREAM("Resulting Item: NOT FOUND");
            }

            ROS_INFO(" ");
            ROS_INFO("Test 2: Find Value by Key");
            search_str = "jada";
            if(Toolbox::Parameter::searchTypeMap(search_str, test::kinematicSolverTypeMap, res_enum))
            {
                ROS_INFO_STREAM("Search Item Found: " << search_str);
                ROS_INFO_STREAM("Resulting Item: " << res_enum);
            }
            else
            {
                ROS_ERROR_STREAM("Search Item NOT Found: " << search_str);
                ROS_ERROR_STREAM("Resulting Item: NOT FOUND");
            }

            ROS_INFO("Test 3: Find Key by Value");
            search_enum = KinematicSolverType::F;
            if(Toolbox::Parameter::searchTypeMap(search_enum, test::kinematicSolverTypeMap, res_str))
            {
                ROS_INFO_STREAM("Search Item Found: " << search_enum);
                ROS_INFO_STREAM("Resulting Item: " << res_str);
            }
            else
            {
                ROS_ERROR_STREAM("Search Item NOT Found: " << search_enum);
                ROS_ERROR_STREAM("Resulting Item: NOT FOUND");
            }

            ROS_INFO(" ");
            ROS_INFO("Test 4: Find Value by Key");
            search_int = 3;
            // if(Toolbox::Parameter::searchTypeMap(static_cast<KinematicSolverType>(search_int), test::kinematicSolverTypeMap, res_str))
            if(Toolbox::Parameter::searchTypeMap(search_int, test::kinematicSolverTypeMap, res_str))
            {
                ROS_INFO_STREAM("Search Item Found: " << search_int);
                ROS_INFO_STREAM("Resulting Item: " << res_str);
            }
            else
            {
                ROS_ERROR_STREAM("Search Item NOT Found: " << search_int);
                ROS_ERROR_STREAM("Resulting Item: NOT FOUND");
            }
    } // Function End: testSearchTypeMap()

} // End: Namespace test

// Test Toolbox Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a ROS Node with a node name
    ros::init(argc, argv, "test_toolbox_common_map");   

    // Starting ROS Nodehandle(s)
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Test(s)
    // -------------------------------

        // test::testMapGetValues();
        // test::testMapGetKeys();  
        test::testSearchTypeMap();

    // Shutdown
    // -------------------------------
    // ROS-Loop waiting for shutdown
    // ros::waitForShutdown();

    // Function return
    return 0;
}

