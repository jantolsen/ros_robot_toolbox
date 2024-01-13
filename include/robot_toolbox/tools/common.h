// Robotics Toolbox - Common Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Common Tools contains helper and utility members 
//      useful for Robotics applications
//
// Version:
//  0.2 - Split Convert functions out of Common
//        [06.01.2023]  -   Jan T. Olsen
//  0.1 - Initial Version
//        [06.01.2023]  -   Jan T. Olsen
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
#ifndef COMMON_TOOL_H       
#define COMMON_TOOL_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <vector>
    #include <map>

    // Ros
    #include <ros/ros.h>

    // Eigen
    #include <Eigen/Geometry>

    // Messages
    #include <geometry_msgs/Transform.h>

    // TF2
    #include <tf2_ros/transform_listener.h>
    #include <tf2_eigen/tf2_eigen.h>
    #include <tf2/convert.h>
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

    // Eigen
    #include <Eigen/Geometry>

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
    // Structs
    // -------------------------------
        // Axis-Type    
        struct AxisType
        {
            const int id;                     // axis identifer
            const std::string name;           // axis name
            const Eigen::Vector3d unit_vec;   // axis unit vector
        };
    

    // Enums
    // -------------------------------
        // Axis-ID
        enum AxisID
        {
            AXIS_ID_X = 0,
            AXIS_ID_Y = 1,
            AXIS_ID_Z = 2
        };

        // Euler-Rotation-ID
        enum EulerRotID
        {
            EULER_ID_PHI = 0,
            EULER_ID_THETA = 1,
            EULER_ID_PSI = 2
        };

        // Euler-Rotation-Sequence
        enum EulerRotSeq
        {
            XYZ = 0,
            ZYX = 1,
            ZXZ = 2,
            ZYZ = 3
        };


    // Constants
    // -------------------------------
        // Axis
        extern AxisType AXIS_X;
        extern AxisType AXIS_Y;
        extern AxisType AXIS_Z;


    // Case-Insensitive String Comparator
    // -------------------------------
    /** \brief Case-Insensitive String Comperator.
    *  
    * Compare and ignores capilization of letters in given strings.
    * Function returns true if strings are found to be equal.
    * 
    * \param string1    First string to compare [std::string]
    * \param string2    Second string to compare [std::string]
    * 
    * \return Function return: Successful/Unsuccessful (true/false) [bool]
    */
    static bool compareStringsCaseInsensitive(
        const std::string& string1, 
        const std::string& string2) 
    {
        // Compare size of the given strings
        if (string1.size() != string2.size())
        {
            // Function return
            return false;
        }

        // Case-insenstive comparison of strings
        if((strcasecmp(string1.c_str(), string2.c_str()) != 0))
        {
            // Function return
            return false;
        }

        // Function return
        return true;
    } // Function end: compareStringsCaseInsensitive()


    // Is Value in Validation-Set
    // -------------------------------
    // (Function Overloading)
    /** \brief Is value in validation-set
    *  
    * Validate given value against a given variable-set (vector)
    * If given value is found in validation-set the function returns true.
    * If not found, the function returns false.
    *
    * \param value          Value to be validated [std::string]
    * \param validation_set Validation set to compare against [std::string]
    * 
    * \return Function return: Successful/Unsuccessful (true/false) [bool]
    */
    template <typename T>
    static bool isValueInSet(
        const T& value,        
        const std::vector<T>& validation_set)
    {
        // Search for given value against validation set
        auto it = std::find(validation_set.begin(), validation_set.end(), value);
        if (it != validation_set.end()) 
        {
            // Function return
            return true;
        }
        
        // Function return
        return false;
    } // Function end: isValueInSet()


    // Is Value in Validation-Set
    // -------------------------------
    // (Function Overloading)
    /** \brief Is value in validation-set
    *  
    * Validate given value against a given validation-set (vector)
    * If given value is found in validation-set the function returns true.
    * If not found, the function returns false.
    *
    * \param value          Value to be validated [std::string]
    * \param validation_set Validation set to compare against [std::string]
    * 
    * \return Function return: Successful/Unsuccessful (true/false) [bool]
    */
    static bool isValueInSet(
        std::string& value,        
        const std::vector<std::string>& validation_set)
    {
        // Compare given value against validation set
        for (const auto& validation_value : validation_set)
        {
            // Compare given value against validation value
            if (compareStringsCaseInsensitive(value, validation_value))
            {
                // Update value to match validation-value in the case of different capilization
                value = validation_value;

                // Function return
                return true;
            }
        }
        // Function return
        return false;
    } // Function end: isValueInSet()
    
} // End Namespace: Robotics Toolbox
#endif // COMMON_TOOL_H