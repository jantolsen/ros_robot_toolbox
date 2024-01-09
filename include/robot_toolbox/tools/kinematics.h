// Robotics Toolbox - Kinematics Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Kinematics Tools contains helper and utility functions 
//      related to kinematic calculations
//
// Version:
//  0.1 - Initial Version
//        [22.02.2023]  -   Jan T. Olsen
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
#ifndef KINEMATIC_TOOL_H       
#define KINEMATIC_TOOL_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <vector>

    // Ros
    #include <ros/ros.h>

    // Ros Messages
    #include "sensor_msgs/JointState.h"

    // Robotics Toolbox
    #include "robot_toolbox/tools/common.h"
    #include "robot_toolbox/tools/convert.h"
    

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{

// Kinematic Tool Class
// -------------------------------
class Kinematics
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:

        // Get Current Joint-State
        // -------------------------------
        // (Function Overloading)
        /** \brief Get Current Joint-State of Robot
        *
        * \param topic          Topic to listen for Joint-State [std::string]
        * \param joint_state    Current Joint-State [sensor_msgs::JointState]
        *
        * \return Function result: Successful/Unsuccessful (true/false)
        */
        static bool getCurrentJointState(
            const std::string& topic,
            sensor_msgs::JointState& joint_state);


        // Get Current Joint-State
        // -------------------------------
        // (Function Overloading)
        /** \brief Get Current Joint-State of Robot
        *
        * \param topic          Topic to listen for Joint-State [std::string]
        * \param joint_position Current Joint-State-Position [std::vector<double>]
        *
        * \return Function result: Successful/Unsuccessful (true/false)
        */
        static bool getCurrentJointState(
            const std::string& topic,
            std::vector<double>& joint_position);


        // Get Current Transform
        // -------------------------------
        // (Function Overloading)
        /** \brief Get Current Pose of a specified frame relative to a reference frame
        *
        * \param target_frame   Target frame for which to acquire current pose [std::string]
        * \param ref_frame      Reference frame for which the target frame is relative to [std::string]
        * \param transform      Current Transform [geometry_msgs::TransformStamped]
        * \param print_result   Print results of current transformation
        *
        * \return Function result: Successful/Unsuccessful (true/false)
        */
        static bool getCurrentTransform(
            const std::string& target_frame,
            const std::string& ref_frame,
            geometry_msgs::TransformStamped& transform,
            bool print_result = false);

        
        // Get Current Transform
        // -------------------------------
        // (Function Overloading)
        /** \brief Get Current Pose of a specified frame relative to a reference frame
        *
        * \param target_frame   Target frame for which to acquire current pose [std::string]
        * \param ref_frame      Reference frame for which the target frame is relative to [std::string]
        * \param transform      Current Transform [geometry_msgs::Transform]
        * \param print_result   Print results of current transformation
        *
        * \return Function result: Successful/Unsuccessful (true/false)
        */
        static bool getCurrentTransform(
            const std::string& target_frame,
            const std::string& ref_frame,
            geometry_msgs::Transform& transform,
            bool print_result = false);


        // Get Current Transform
        // -------------------------------
        // (Function Overloading)
        /** \brief Get Current Pose of a specified frame relative to a reference frame
        *
        * \param target_frame   Target frame for which to acquire current pose [std::string]
        * \param ref_frame      Reference frame for which the target frame is relative to [std::string]
        * \param transform      Current Transform [Eigen::Isometry3d]
        * \param print_result   Print results of current transformation
        *
        * \return Function result: Successful/Unsuccessful (true/false)
        */
        static bool getCurrentTransform(
            const std::string& target_frame,
            const std::string& ref_frame,
            Eigen::Isometry3d& transform,
            bool print_result = false);


        // Get Current Pose
        // -------------------------------
        // (Function Overloading)
        /** \brief Get Current Pose of a specified frame relative to a reference frame
        *
        * \param target_frame   Target frame for which to acquire current pose [std::string]
        * \param ref_frame      Reference frame for which the target frame is relative to [std::string]
        * \param pose           Current Pose [geometry_msgs::PoseStamped]
        * \param print_result   Print results of current transformation
        *
        * \return Function result: Successful/Unsuccessful (true/false)
        */
        static bool getCurrentPose(
            const std::string& target_frame,
            const std::string& ref_frame,
            geometry_msgs::PoseStamped& pose,
            bool print_result = false);


        // Get Current Pose
        // -------------------------------
        // (Function Overloading)
        /** \brief Get Current Pose of a specified frame relative to a reference frame
        *
        * \param target_frame   Target frame for which to acquire current pose [std::string]
        * \param ref_frame      Reference frame for which the target frame is relative to [std::string]
        * \param pose           Current Pose [geometry_msgs::Pose]
        * \param print_result   Print results of current transformation
        *
        * \return Function result: Successful/Unsuccessful (true/false)
        */
        static bool getCurrentPose(
            const std::string& target_frame,
            const std::string& ref_frame,
            geometry_msgs::Pose& pose,
            bool print_result = false);


        // Get Current Pose
        // -------------------------------
        // (Function Overloading)
        /** \brief Get Current Pose of a specified frame relative to a reference frame
        *
        * \param target_frame   Target frame for which to acquire current pose [std::string]
        * \param ref_frame      Reference frame for which the target frame is relative to [std::string]
        * \param pose           Current Pose [Eigen::Isometry3d]
        * \param print_result   Print results of current transformation
        *
        * \return Function result: Successful/Unsuccessful (true/false)
        */
        static bool getCurrentPose(
            const std::string& target_frame,
            const std::string& ref_frame,
            Eigen::Isometry3d& pose,
            bool print_result = false);


        // Check for empty pose
        // -------------------------------
        // (Function Overloading)
        /** \brief Check for empty pose.
        *
        * Check if given pose is empty (all values are zero).
        * Function returns true if pose is empty.
        *
        * \param pose    Pose to check [geometry_msgs::Pose]
        *
        * \return Function return: Pose is empty (true/false) [bool]
        */
        static bool isPoseEmpty(
            const geometry_msgs::Pose& pose);


        // Check for empty pose
        // -------------------------------
        // (Function Overloading)
        /** \brief Check for empty pose.
        *
        * Check if given pose is empty (all values are zero).
        * Function returns true if pose is empty.
        *
        * \param poseStamped    Pose to check [geometry_msgs::PoseStamped]
        *
        * \return Function return: Pose is empty (true/false) [bool]
        */
        static bool isPoseEmpty(
            const geometry_msgs::PoseStamped& poseStamped);

        
        // Check for empty pose
        // -------------------------------
        // (Function Overloading)
        /** \brief Check for empty pose.
        *
        * Check if given pose is empty (all values are zero).
        * Function returns true if pose is empty.
        *
        * \param pose    Pose to check [robot_toolbox::PoseRPY]
        *
        * \return Function return: Pose is empty (true/false) [bool]
        */
        static bool isPoseEmpty(
            const robot_toolbox::PoseRPY poseRPY);
        

        // Check for empty transform
        // -------------------------------
        // (Function Overloading)
        /** \brief Check for empty transform.
        *
        * Check if given transform is empty (all values are zero).
        * Function returns true if transform is empty.
        *
        * \param transform    Transform to check [geometry_msgs::Transform]
        *
        * \return Function return: Transform is empty (true/false) [bool]
        */
        static bool isTransformEmpty(
            const geometry_msgs::Transform& transform);


        // Check for empty transform
        // -------------------------------
        // (Function Overloading)
        /** \brief Check for empty transform.
        *
        * Check if given transform is empty (all values are zero).
        * Function returns true if transform is empty.
        *
        * \param transformStamped   Transform to check [geometry_msgs::TransformStamped]
        *
        * \return Function return: Transform is empty (true/false) [bool]
        */
        static bool isTransformEmpty(
            const geometry_msgs::TransformStamped& transformStamped);
        


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

};  // End Class: Kinematics
} // End Namespace: Robotics Toolbox
#endif // KINEMATIC_TOOL_H