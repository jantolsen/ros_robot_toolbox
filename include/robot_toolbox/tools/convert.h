// Robotics Toolbox - Convert Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Convert Tools contains helper and utility functions 
//      for converting data and types for Robotics applications
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
#ifndef CONVERT_TOOL_H       
#define CONVERT_TOOL_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <vector>
    #include <map>
    #include <algorithm>
    #include <cctype>  // For std::toupper

    // Ros
    #include <ros/ros.h>

    // Toolbox
    #include "robot_toolbox/tools/common.h"

    // Messages
    #include "robot_toolbox/PoseRPY.h"


    // Eigen
    #include <Eigen/Geometry>

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{

// Convert Tool Class
// -------------------------------
class Convert
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:

        // Convert Degrees to Radians
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert an angle (double) from Degrees to Radians
        *
        * \param deg An angle given in degrees
        *
        * \return An angle given in radians
        */
        static double degToRad(double deg = 1.0);


        // Convert Degrees to Radians
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert a vector of angles (double) from Degrees to Radians
        *
        * \param deg_vec    Vector of angles given in degrees
        *
        * \return Vector of angles given in radians
        */
        static std::vector<double> degToRad(
            std::vector<double> deg_vec);


        // Convert Radians to Degrees 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert an angle (double) from Radians to Degrees 
        *
        * \param rad An angle given in radians
        *
        * \return An angle given in degrees
        */
        static double radToDeg(double rad = 1.0);


        // Convert Radians to Degrees 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert a vector of angles (double) from Radians to Degrees 
        *
        * \param rad_vec Vector of angles given in radians
        *
        * \return A vector of angles given in degrees
        */
        static std::vector<double> radToDeg(
            std::vector<double> rad_vec);
        
        
        // Convert String to Upper-Case String
        // -------------------------------
        // (Function Overloading)
        /** \brief Converts given string to all upper-case letters
        *
        * \param input Input string [std::string]
        * \return Upper-Case String [std::string]
        */
        static std::string stringToUpperCase(
            std::string input);

        // Get Variable Data-Type name
        // -------------------------------
        // (Function Overloading)
        /** \brief Get data-type name of given variable
        *
        * Uses boost::demangle to obtain and return a human-readable type-name
        *
        * \param input  Input variable to get type name of [typename Type]
        * \return Return human-readable type-name [std::string]
        */
        template <typename Type>
        static std::string getTypeName(
            const Type& input)
        {
            // Return human-readable type-name
            return boost::core::demangle(typeid(Type).name());
        } // Function-End: getTypeName()


        // Get Data-Type name
        // -------------------------------
        // (Function Overloading)
        /** \brief Get data-type name of given fundamental type
        *
        * Uses boost::demangle to obtain and return a human-readable type-name
        *
        * \param type_info Fundamental data type to get type name of [std::type_info&]
        * \return Return human-readable type-name [std::string]
        */
        static std::string getTypeName(
            const std::type_info& type_info)
        {
            // Return human-readable type-name
            return boost::core::demangle(type_info.name());
        } // Function-End: getTypeName()


        // Convert Euler to Quaternion 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert an euler rotation (Eigen::Vector3d) 
        *
        * to a Quaternion (Eigen::Quaternion<double>)
        *
        * \param euler Euler-Rotation (rad) [Eigen::Vector3d]
        * \param seq Euler-Sequence for rotation (XYZ = 0, ZYX = 1, ZXZ = 2, ZYZ = 3) [int]
        * \return Rotation in Quaternion [Eigen::Quaterniond]
        */
        static Eigen::Quaternion<double> eulerToQuaternion(
            Eigen::Vector3d euler,
            int seq = XYZ);


        // Convert Euler to Quaternion 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert an euler rotation (double) 
        * to a Quaternion (Eigen::Quaternion<double>)
        *
        * \param phi    1st axis-rotation (deg) [double]
        * \param theta  2nd axis rotation (deg) [double]
        * \param psi    3rd axis rotation (deg) [double]
        * \param seq    Euler-Sequence for rotation (XYZ = 0, ZYX = 1, ZXZ = 2, ZYZ = 3) [int]
        * \return Rotation in Quaternion [Eigen::Quaternion<double>]
        */
        static Eigen::Quaternion<double> eulerToQuaternion(
            double phi,
            double theta,
            double psi,
            int seq = XYZ);


        // Convert Euler to Quaternion 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert an euler rotation (geometry_msgs::Point) 
        * to a Quaternion (geometry_msgs::Quaternion)
        *
        * \param euler Euler-Rotation (rad) [geometry_msgs::Point]
        * \param seq Euler-Sequence for rotation (XYZ = 0, ZYX = 1, ZXZ = 2, ZYZ = 3) [int]
        * \return Rotation in Quaternion [geometry_msgs::Quaternion]
        */
        static geometry_msgs::Quaternion eulerToQuaternion(
            geometry_msgs::Point euler,
            int seq = XYZ);


        // Convert Quaternion to Euler 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert a quaternion rotation (Eigen::Quaternion<double>) 
        * to an euler-rotation (Eigen::Vector3d) 
        *
        * \param q Quaternion-Rotation (rad) [Eigen::Quaternion<double>]
        * \param seq Euler-Sequence for rotation (XYZ = 0, ZYX = 1, ZXZ = 2, ZYZ = 3) [int]
        * \return Rotation in Euler-Angles [Eigen::Vector3d]
        */
        static Eigen::Vector3d quaternionToEuler(
            Eigen::Quaternion<double> q,
            int seq = XYZ);


        // Convert Quaternion to Euler 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert a quaternion rotation (Eigen::Quaternion<double>) 
        * to an euler-rotation (Eigen::Vector3d) 
        *
        * \param w Quaternion scalar coefficient W [double]
        * \param x Quaternion scalar coefficient [double]
        * \param y Quaternion scalar coefficient [double]
        * \param z Quaternion scalar coefficient [double]
        * \param seq Euler-Sequence for rotation (XYZ = 0, ZYX = 1, ZXZ = 2, ZYZ = 3) [int]
        * \return Rotation in Euler-Angles [Eigen::Vector3d]
        */
        static Eigen::Vector3d quaternionToEuler(
            double w, 
            double x, 
            double y, 
            double z,
            int seq = XYZ);


        // Convert Quaternion to Euler 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert a quaternion rotation (geometry_msgs::Quaternion) 
        * to an euler-rotation (geometry_msgs::Point) 
        *
        * \param q Quaternion-Rotation (rad) [geometry_msgs::Quaternion]
        * \param seq Euler-Sequence for rotation (XYZ = 0, ZYX = 1, ZXZ = 2, ZYZ = 3) [int]
        * \return Rotation in Euler-Angles [geometry_msgs::Vector3d]
        */
        static geometry_msgs::Point quaternionToEuler(
            geometry_msgs::Quaternion q,
            int seq = XYZ);


        // Convert Eigen-Vector to Std-Vector 
        // -------------------------------
        /** \brief Convert a Eigen::Vector to Std::Vector
        *
        * \param v_in input vector [Eigen::VectorXd]
        * \return converted vector [std::vector<double>]
        */
        static std::vector<double> vectorEigenToStd(
            Eigen::VectorXd v_in);


        // Convert Std-Vector to Eigen-Vector
        // -------------------------------
        /** \brief Convert a Eigen::Vector to Std::Vector
        *
        * \param vec input vector [std::vector<double>]
        * \return converted vector [Eigen::VectorXd]
        */
        static Eigen::VectorXd vectorStdToEigen(
            std::vector<double> v_in);


        // Convert Pose to Transform
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert Pose to Transform
        *
        * \param pose Pose [geometry_msgs::Pose]
        * \return Transform [geometry_msgs::Transform]
        */
        static geometry_msgs::Transform poseToTransform(
            geometry_msgs::Pose pose);


        // Convert Pose to Transform-Stamped 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert Pose to Transform-Stamped
        *
        * \param pose Pose [geometry_msgs::Pose]
        * \param parent_frame Frame of which pose is relative to
        * \param child_frame Frame of which to accquire pose
        * \return Transform-Stamped [geometry_msgs::TransformStamped]
        */
        static geometry_msgs::TransformStamped poseToTransform(
            geometry_msgs::Pose pose,
            std::string parent_frame,
            std::string child_frame);  


        // Convert Pose to Transform-Stamped 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert Pose-Stamped to Transform-Stamped
        *
        * \param pose_stamped Pose-Stamped [geometry_msgs::PoseStamped]
        * \return Transform-Stamped [geometry_msgs::TransformStamped]
        */
        static geometry_msgs::TransformStamped poseToTransform(
            geometry_msgs::PoseStamped pose_stamped);


        // Convert Transform to Pose 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert Transform to Pose
        *
        * \param transform Transform [geometry_msgs::Transform]
        * \return Pose [geometry_msgs::Pose]
        */
        static geometry_msgs::Pose transformToPose(
            geometry_msgs::Transform transform);


        // Convert Transform to Pose-Stamped 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert Transform to Pose-Stamped
        *
        * \param transform Transform [geometry_msgs::Transform]
        * \param parent_frame Frame of which transform is relative to
        * \return Pose-Stamped [geometry_msgs::Pose]
        */
        static geometry_msgs::PoseStamped transformToPose(
            geometry_msgs::Transform transform,
            std::string parent_frame);  


        // Convert Transform-Stamped to Pose-Stamped
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert Transform-Stamped to Pose-Stamped
        *
        * \param transform Transform-Stamped [geometry_msgs::TransformStamped]
        * \return Pose-Stamped [geometry_msgs::PoseStamped]
        */
        static geometry_msgs::PoseStamped transformToPose(
            geometry_msgs::TransformStamped transform_stamped);


        // Convert Pose to Pose RPY
        // -------------------------------
        /** \brief Convert Pose to Pose RPY
        *
        * \param pose Pose [m, rad] [geometry_msgs::Pose] 
        * \return PoseRPY [m, deg] [robot_toolbox::PoseRPY] 
        */
        static robot_toolbox::PoseRPY poseToPoseRPY(
            geometry_msgs::Pose pose);


        // Convert Pose RPY to Pose
        // -------------------------------
        /** \brief Convert Pose RPY to Pose
        *
        * \param pose PoseRPY [m, deg] [robot_toolbox::PoseRPY]
        * \return Pose [m, rad] [geometry_msgs::Pose]
        */
        static geometry_msgs::Pose poseRPYToPose(
            robot_toolbox::PoseRPY poseRPY);


    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:


    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:

        // Prefix message for class
        static const std::string CLASS_PREFIX;

}; // End Class: Convert
} // End Namespace: Robotics Toolbox
#endif // CONVERT_TOOL_H 