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
    #include <boost/bimap.hpp>

namespace test
{
    enum EnumType
    {
        A,
        B,
        C,
        D,
        E,
        F
    };

    // Test: Map 1
    // (Key: String - Value: Int)
    static std::map<std::string, int> const stdMap_keyString_valueInt =
    {
        {"KDL",     11},
        {"OPW",     22},
        {"TRACIK",  33},
        {"LMA",     44}
    };

    // Test: Map 2
    // (Key: Int - Value: String)
    static std::map<int, std::string> const stdMap_keyInt_valueString =
    {
        {1,     "ONE"},
        {2,     "TWO"},
        {3,     "THREE"},
        {4,     "FOUR"}
    };

    // Test: Map 3
    // (Key: String - Value: Enum)
    static std::map<std::string, EnumType> const stdMap_keyString_valueEnum =
    {
        {"AAA",     EnumType::A},
        {"BBB",     EnumType::B},
        {"CCC",     EnumType::C},
        {"DDD",     EnumType::D}
    };

    // Test: Map 4
    // (Key: Enum - Value: Double)
    static std::map<EnumType, double> const stdMap_keyEnum_valueDouble =
    {
        {EnumType::A, 0.111},
        {EnumType::B, 0.222},
        {EnumType::C, 0.333},
        {EnumType::D, 0.444}
    };




    // Test: Map 1
    // (Key: String - Value: Int)
    typedef boost::bimap<std::string, int> bimap_type1;
    static bimap_type1 biMap_keyString_valueInt;
    

    // Test: Map 2
    // (Key: Int - Value: String)
    typedef boost::bimap<int, std::string> bimap_type2;
    static bimap_type2 biMap_keyInt_valueString;

    // Test: Map 3
    // (Key: String - Value: Enum)
    typedef boost::bimap<std::string, EnumType> bimap_type3;
    static bimap_type3 biMap_keyString_valueEnum;

    // Test: Map 4
    // (Key: Enum - Value: Double)
    typedef boost::bimap<EnumType, double> bimap_type4;
    static bimap_type4 biMap_keyEnum_valueDouble;

    void populateBiMap()
    {
        biMap_keyString_valueInt.insert( bimap_type1::value_type("KDL",     11));
        biMap_keyString_valueInt.insert( bimap_type1::value_type("OPW",     22));
        biMap_keyString_valueInt.insert( bimap_type1::value_type("TRACIK",  33));
        biMap_keyString_valueInt.insert( bimap_type1::value_type("LMA",     44));


        biMap_keyInt_valueString.insert( bimap_type2::value_type(1,     "ONE"));
        biMap_keyInt_valueString.insert( bimap_type2::value_type(2,     "TWO"));
        biMap_keyInt_valueString.insert( bimap_type2::value_type(3,     "THREE"));
        biMap_keyInt_valueString.insert( bimap_type2::value_type(4,     "FOUR"));


        biMap_keyString_valueEnum.insert( bimap_type3::value_type("AAA",     EnumType::A));
        biMap_keyString_valueEnum.insert( bimap_type3::value_type("BBB",     EnumType::B));
        biMap_keyString_valueEnum.insert( bimap_type3::value_type("CCC",     EnumType::C));
        biMap_keyString_valueEnum.insert( bimap_type3::value_type("DDD",     EnumType::D));

        biMap_keyEnum_valueDouble.insert( bimap_type4::value_type(EnumType::A,  0.111));
        biMap_keyEnum_valueDouble.insert( bimap_type4::value_type(EnumType::B,  0.222));
        biMap_keyEnum_valueDouble.insert( bimap_type4::value_type(EnumType::C,  0.333));
        biMap_keyEnum_valueDouble.insert( bimap_type4::value_type(EnumType::D,  0.444));
    }

    // Test: Std-Map Find by Keys
    // (Get Values in Maps using Keys)
    // -------------------------------
    void stdMapFindByKey()
    {
        std::string key_str;
        int key_int;
        EnumType key_enum;

        ROS_INFO("Map: Search value using key:");
        ROS_INFO("--------------------");
        
        ROS_INFO(" ");
        ROS_INFO("Test-Map 1: Std-Map");
        ROS_INFO("Key: String - Value: Int");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            key_str = "KDL";
            if(auto res = Toolbox::Map::searchMapByKey(stdMap_keyString_valueInt, key_str))
            {
                int test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            key_str = "oPw";
            if(auto res = Toolbox::Map::searchMapByKey(stdMap_keyString_valueInt, key_str))
            {
                int test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 3");
            ROS_INFO("--------------------");
            key_str = "asd";
            if(auto res = Toolbox::Map::searchMapByKey(stdMap_keyString_valueInt, key_str))
            {
                int test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 4");
            ROS_INFO("--------------------");
            key_str = "lma";
            bool caseSensitive = false;
            if(auto res = Toolbox::Map::searchMapByKey(stdMap_keyString_valueInt, key_str))
            {
                int test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

        ROS_INFO(" ");
        ROS_INFO("Test-Map 2: Std-Map");
        ROS_INFO("Key: Int - Value: String");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            key_int = 2;
            if(auto res = Toolbox::Map::searchMapByKey(stdMap_keyInt_valueString, key_int))
            {
                std::string test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_int);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            key_int = 4;
            if(auto res = Toolbox::Map::searchMapByKey(stdMap_keyInt_valueString, key_int))
            {
                std::string test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_int);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 3");
            ROS_INFO("--------------------");
            key_int = 123;
            if(auto res = Toolbox::Map::searchMapByKey(stdMap_keyInt_valueString, key_int))
            {
                std::string test= *res;
                ROS_INFO_STREAM("   Key Found: " << key_int);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

        ROS_INFO(" ");
        ROS_INFO("Test-Map 3: Std-Map");
        ROS_INFO("Key: String - Value: Enum");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            key_str = "AAA";
            if(auto res = Toolbox::Map::searchMapByKey(stdMap_keyString_valueEnum, key_str))
            {
                EnumType test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            key_str = "bBb";
            if(auto res = Toolbox::Map::searchMapByKey(stdMap_keyString_valueEnum, key_str))
            {
                EnumType test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 3");
            ROS_INFO("--------------------");
            key_str = "test";
            if(auto res = Toolbox::Map::searchMapByKey(stdMap_keyString_valueEnum, key_str))
            {
                EnumType test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 4");
            ROS_INFO("--------------------");
            key_str = "aaA";
            if(auto res = Toolbox::Map::searchMapByKey(stdMap_keyString_valueEnum, key_str))
            {
                EnumType test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

        ROS_INFO(" ");
        ROS_INFO("Test-Map 4: Std-Map");
        ROS_INFO("Key: Enum - Value: Double");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            key_enum = EnumType::A;
            if(auto res = Toolbox::Map::searchMapByKey(stdMap_keyEnum_valueDouble, key_enum))
            {
                double test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_enum);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            key_enum = static_cast<EnumType>(2);
            if(auto res = Toolbox::Map::searchMapByKey(stdMap_keyEnum_valueDouble, key_enum))
            {
                double test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_enum);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 3");
            ROS_INFO("--------------------");
            key_enum = static_cast<EnumType>(123);
            if(auto res = Toolbox::Map::searchMapByKey(stdMap_keyEnum_valueDouble, key_enum))
            {
                double test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_enum);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }
    } // Function end: stdMapFindByKey()
    

    // Test: Bi-Map Find by Keys
    // (Get Values in Maps using Keys)
    // -------------------------------
    void biMapFindByKey()
    {
        std::string key_str;
        int key_int;
        EnumType key_enum;

        ROS_INFO("Map: Search value using key:");
        ROS_INFO("--------------------");
        
        ROS_INFO(" ");
        ROS_INFO("Test-Map 1: Std-Map");
        ROS_INFO("Key: String - Value: Int");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            key_str = "KDL";
            if(auto res = Toolbox::Map::searchMapByKey(biMap_keyString_valueInt, key_str))
            {
                int test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            key_str = "oPw";
            if(auto res = Toolbox::Map::searchMapByKey(biMap_keyString_valueInt, key_str))
            {
                int test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 3");
            ROS_INFO("--------------------");
            key_str = "asd";
            if(auto res = Toolbox::Map::searchMapByKey(biMap_keyString_valueInt, key_str))
            {
                int test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 4");
            ROS_INFO("--------------------");
            key_str = "lma";
            bool caseSensitive = false;
            if(auto res = Toolbox::Map::searchMapByKey(biMap_keyString_valueInt, key_str))
            {
                int test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

        ROS_INFO(" ");
        ROS_INFO("Test-Map 2: Std-Map");
        ROS_INFO("Key: Int - Value: String");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            key_int = 2;
            if(auto res = Toolbox::Map::searchMapByKey(biMap_keyInt_valueString, key_int))
            {
                std::string test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_int);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            key_int = 4;
            if(auto res = Toolbox::Map::searchMapByKey(biMap_keyInt_valueString, key_int))
            {
                std::string test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_int);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 3");
            ROS_INFO("--------------------");
            key_int = 123;
            if(auto res = Toolbox::Map::searchMapByKey(biMap_keyInt_valueString, key_int))
            {
                std::string test= *res;
                ROS_INFO_STREAM("   Key Found: " << key_int);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

        ROS_INFO(" ");
        ROS_INFO("Test-Map 3: Std-Map");
        ROS_INFO("Key: String - Value: Enum");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            key_str = "AAA";
            if(auto res = Toolbox::Map::searchMapByKey(biMap_keyString_valueEnum, key_str))
            {
                EnumType test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            key_str = "bBb";
            if(auto res = Toolbox::Map::searchMapByKey(biMap_keyString_valueEnum, key_str))
            {
                EnumType test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 3");
            ROS_INFO("--------------------");
            key_str = "test";
            if(auto res = Toolbox::Map::searchMapByKey(biMap_keyString_valueEnum, key_str))
            {
                EnumType test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 4");
            ROS_INFO("--------------------");
            key_str = "aaA";
            if(auto res = Toolbox::Map::searchMapByKey(biMap_keyString_valueEnum, key_str))
            {
                EnumType test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

        ROS_INFO(" ");
        ROS_INFO("Test-Map 4: Std-Map");
        ROS_INFO("Key: Enum - Value: Double");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            key_enum = EnumType::A;
            if(auto res = Toolbox::Map::searchMapByKey(biMap_keyEnum_valueDouble, key_enum))
            {
                double test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_enum);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            key_enum = static_cast<EnumType>(2);
            if(auto res = Toolbox::Map::searchMapByKey(biMap_keyEnum_valueDouble, key_enum))
            {
                double test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_enum);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 3");
            ROS_INFO("--------------------");
            key_enum = static_cast<EnumType>(123);
            if(auto res = Toolbox::Map::searchMapByKey(biMap_keyEnum_valueDouble, key_enum))
            {
                double test = *res;
                ROS_INFO_STREAM("   Key Found: " << key_enum);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Key NOT Found: " << key_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }
    } // Function end: biMapFindByKey()


    // Test: Std-Map Find by Value
    // (Get Keys in Maps using Values)
    // -------------------------------
    void stdMapFindBValue()
    {
        std::string value_str;
        int value_int;
        EnumType value_enum;
        double value_double;

        ROS_INFO("Map: Search key using value:");
        ROS_INFO("--------------------");
        
        ROS_INFO(" ");
        ROS_INFO("Test-Map 1: Std-Map");
        ROS_INFO("Key: String - Value: Int");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            value_int = 22;
            if(auto res = Toolbox::Map::searchMapByValue(stdMap_keyString_valueInt, value_int))
            {
                std::string test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_int);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            value_int = 1;
            if(auto res = Toolbox::Map::searchMapByValue(stdMap_keyString_valueInt, value_int))
            {
                std::string test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_int);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

        ROS_INFO(" ");
        ROS_INFO("Test-Map 2: Std-Map");
        ROS_INFO("Key: Int - Value: String");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            value_str = "ONE";
            if(auto res = Toolbox::Map::searchMapByValue(stdMap_keyInt_valueString, value_str))
            {
                int test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            value_str = "three";
            if(auto res = Toolbox::Map::searchMapByValue(stdMap_keyInt_valueString, value_str))
            {
                int test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 3");
            ROS_INFO("--------------------");
            value_str = "wrong_key";
            if(auto res = Toolbox::Map::searchMapByValue(stdMap_keyInt_valueString, value_str))
            {
                int test= *res;
                ROS_INFO_STREAM("   Value Found: " << value_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

        ROS_INFO(" ");
        ROS_INFO("Test-Map 3: Std-Map");
        ROS_INFO("Key: String - Value: Enum");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            value_enum = EnumType::D;
            if(auto res = Toolbox::Map::searchMapByValue(stdMap_keyString_valueEnum, value_enum))
            {
                std::string test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_enum);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_enum);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            value_enum = static_cast<EnumType>(2);
            if(auto res = Toolbox::Map::searchMapByValue(stdMap_keyString_valueEnum, value_enum))
            {
                std::string test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_enum);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_enum);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 3");
            ROS_INFO("--------------------");
            value_enum = static_cast<EnumType>(99);
            if(auto res = Toolbox::Map::searchMapByValue(stdMap_keyString_valueEnum, value_enum))
            {
                std::string test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_enum);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_enum);
                ROS_ERROR_STREAM("  Results: Failed!");
            }


        ROS_INFO(" ");
        ROS_INFO("Test-Map 4: Std-Map");
        ROS_INFO("Key: Enum - Value: Double");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            value_double = 0.333;
            if(auto res = Toolbox::Map::searchMapByValue(stdMap_keyEnum_valueDouble, value_double))
            {
                double test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_double);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_double);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            value_double = 0.2;
            if(auto res = Toolbox::Map::searchMapByValue(stdMap_keyEnum_valueDouble, value_double))
            {
                double test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_double);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_double);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 3");
            ROS_INFO("--------------------");
            value_double = 2;
            if(auto res = Toolbox::Map::searchMapByValue(stdMap_keyEnum_valueDouble, value_double))
            {
                double test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_double);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_double);
                ROS_ERROR_STREAM("  Results: Failed!");
            }
    } // Function end: stdMapFindByKey()


    // Test: Bi-Map Find by Value
    // (Get Keys in Maps using Values)
    // -------------------------------
    void biMapFindBValue()
    {
        std::string value_str;
        int value_int;
        EnumType value_enum;
        double value_double;

        ROS_INFO("Map: Search key using value:");
        ROS_INFO("--------------------");
        
        ROS_INFO(" ");
        ROS_INFO("Test-Map 1: Std-Map");
        ROS_INFO("Key: String - Value: Int");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            value_int = 22;
            if(auto res = Toolbox::Map::searchMapByValue(biMap_keyString_valueInt, value_int))
            {
                std::string test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_int);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            value_int = 1;
            if(auto res = Toolbox::Map::searchMapByValue(biMap_keyString_valueInt, value_int))
            {
                std::string test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_int);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_int);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

        ROS_INFO(" ");
        ROS_INFO("Test-Map 2: Std-Map");
        ROS_INFO("Key: Int - Value: String");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            value_str = "ONE";
            if(auto res = Toolbox::Map::searchMapByValue(biMap_keyInt_valueString, value_str))
            {
                int test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            value_str = "three";
            if(auto res = Toolbox::Map::searchMapByValue(biMap_keyInt_valueString, value_str))
            {
                int test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 3");
            ROS_INFO("--------------------");
            value_str = "wrong_key";
            if(auto res = Toolbox::Map::searchMapByValue(biMap_keyInt_valueString, value_str))
            {
                int test= *res;
                ROS_INFO_STREAM("   Value Found: " << value_str);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_str);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

        ROS_INFO(" ");
        ROS_INFO("Test-Map 3: Std-Map");
        ROS_INFO("Key: String - Value: Enum");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            value_enum = EnumType::D;
            if(auto res = Toolbox::Map::searchMapByValue(biMap_keyString_valueEnum, value_enum))
            {
                std::string test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_enum);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_enum);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            value_enum = static_cast<EnumType>(2);
            if(auto res = Toolbox::Map::searchMapByValue(biMap_keyString_valueEnum, value_enum))
            {
                std::string test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_enum);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_enum);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 3");
            ROS_INFO("--------------------");
            value_enum = static_cast<EnumType>(99);
            if(auto res = Toolbox::Map::searchMapByValue(biMap_keyString_valueEnum, value_enum))
            {
                std::string test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_enum);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_enum);
                ROS_ERROR_STREAM("  Results: Failed!");
            }


        ROS_INFO(" ");
        ROS_INFO("Test-Map 4: Std-Map");
        ROS_INFO("Key: Enum - Value: Double");
        ROS_INFO("--------------------");

            ROS_INFO(" ");
            ROS_INFO("  Test: 1");
            ROS_INFO("--------------------");
            value_double = 0.333;
            if(auto res = Toolbox::Map::searchMapByValue(biMap_keyEnum_valueDouble, value_double))
            {
                double test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_double);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_double);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 2");
            ROS_INFO("--------------------");
            value_double = 0.2;
            if(auto res = Toolbox::Map::searchMapByValue(biMap_keyEnum_valueDouble, value_double))
            {
                double test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_double);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_double);
                ROS_ERROR_STREAM("  Results: Failed!");
            }

            ROS_INFO(" ");
            ROS_INFO("  Test: 3");
            ROS_INFO("--------------------");
            value_double = 2;
            if(auto res = Toolbox::Map::searchMapByValue(biMap_keyEnum_valueDouble, value_double))
            {
                double test = *res;
                ROS_INFO_STREAM("   Value Found: " << value_double);
                ROS_INFO_STREAM("   Search Results: " << test);
            }
            else
            {
                ROS_ERROR_STREAM("  Value NOT Found: " << value_double);
                ROS_ERROR_STREAM("  Results: Failed!");
            }
    } // Function end: biMapFindByKey()
} // End: Namespace test

boost::optional<std::string> checkInt1(int test)
{
    std::string result;

    if(test == 0)
    {
        ROS_ERROR("INVALID INT");
        return boost::none;
    }
    
    result = "Valid int";
    return result;
}

boost::optional<std::string> checkInt2(int test)
{
    boost::optional<std::string> result;
    if(test == 0)
    {
        
        return result = "Valid int";
    }
    
    // result = "Valid int";
    return result;
}

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
        test::populateBiMap();
        // test::stdMapFindByKey();
        // test::biMapFindByKey();

        // test::stdMapFindBValue();
        test::biMapFindBValue();

    
    // Shutdown
    // -------------------------------
    // ROS-Loop waiting for shutdown
    // ros::waitForShutdown();

    // Function return
    return 0;
}

