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

    // Ros
    #include <ros/ros.h>

    // Toolbox
    #include "robot_toolbox/tools/common.h"

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
        /** \brief Convert an angle (double) from Degrees to Radians
        * \param deg An angle given in degrees
        * \return An angle given in radians
        */
        static double degToRad(double deg = 1.0);


        // Convert Radians to Degrees 
        // -------------------------------
        /** \brief Convert an angle (double) from Radians to Degrees 
        * \param rad An angle given in radians
        * \return An angle given in degrees
        */
        static double radToDeg(double rad = 1.0);


        // Convert Euler to Quaternion 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert an euler rotation (Eigen::Vector3d) 
        * to a Quaternion (Eigen::Quaternion<double>)
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
        /** \brief Convert an euler rotation (geometry_msgs::Vector3) 
        * to a Quaternion (geometry_msgs::Quaternion)
        * \param euler Euler-Rotation (rad) [geometry_msgs::Vector3]
        * \param seq Euler-Sequence for rotation (XYZ = 0, ZYX = 1, ZXZ = 2, ZYZ = 3) [int]
        * \return Rotation in Quaternion [geometry_msgs::Quaternion]
        */
        static geometry_msgs::Quaternion eulerToQuaternion(
            geometry_msgs::Vector3 euler,
            int seq = XYZ);


        // Convert Quaternion to Euler 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert a quaternion rotation (Eigen::Quaternion<double>) 
        * to an euler-rotation (Eigen::Vector3d) 
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
        * to an euler-rotation (geometry_msgs::Vector3) 
        * \param q Quaternion-Rotation (rad) [geometry_msgs::Quaternion]
        * \param seq Euler-Sequence for rotation (XYZ = 0, ZYX = 1, ZXZ = 2, ZYZ = 3) [int]
        * \return Rotation in Euler-Angles [geometry_msgs::Vector3d]
        */
        static geometry_msgs::Vector3 quaternionToEuler(
            geometry_msgs::Quaternion q,
            int seq = XYZ);


        // Convert Eigen-Vector to Std-Vector 
        // -------------------------------
        /** \brief Convert a Eigen::Vector to Std::Vector
        * \param v_in input vector [Eigen::VectorXd]
        * \return converted vector [std::vector<double>]
        */
        static std::vector<double> vectorEigenToStd(
            Eigen::VectorXd v_in);


        // Convert Std-Vector to Eigen-Vector
        // -------------------------------
        /** \brief Convert a Eigen::Vector to Std::Vector
        * \param vec input vector [std::vector<double>]
        * \return converted vector [Eigen::VectorXd]
        */
        static Eigen::VectorXd vectorStdToEigen(
            std::vector<double> v_in);


        // Convert Pose to Transform
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Convert Pose to Transform
        * \param pose Pose [geometry_msgs::Pose]
        * \return Transform [geometry_msgs::Transform]
        */
        static geometry_msgs::Transform poseToTransform(
            geometry_msgs::Pose pose);


        // Convert Pose to Transform-Stamped 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert Pose to Transform-Stamped
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
        * \param pose_stamped Pose-Stamped [geometry_msgs::PoseStamped]
        * \return Transform-Stamped [geometry_msgs::TransformStamped]
        */
        static geometry_msgs::TransformStamped poseToTransform(
            geometry_msgs::PoseStamped pose_stamped);


        // Convert Transform to Pose 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert Transform to Pose
        * \param transform Transform [geometry_msgs::Transform]
        * \return Pose [geometry_msgs::Pose]
        */
        static geometry_msgs::Pose transformToPose(
            geometry_msgs::Transform transform);


        // Convert Transform to Pose-Stamped 
        // -------------------------------
        // (Function Overloading)
        /** \brief Convert Transform to Pose-Stamped
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
        * \param transform Transform-Stamped [geometry_msgs::TransformStamped]
        * \return Pose-Stamped [geometry_msgs::PoseStamped]
        */
        static geometry_msgs::PoseStamped transformToPose(
            geometry_msgs::TransformStamped transform_stamped);


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