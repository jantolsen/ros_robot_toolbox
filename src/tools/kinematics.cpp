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

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robot_toolbox/tools/kinematics.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Kinematics Tool Class - Members:
// -------------------------------

    // Constants
    // -------------------------------
    const std::string Kinematics::CLASS_PREFIX = "Toolbox::Kinematics::";


    // Get Current Joint-State
    // -------------------------------
    // (Function Overloading)
    bool Kinematics::getCurrentJointState(
            const std::string& topic,
            sensor_msgs::JointState& joint_state)
    {
        // Capture Current-Joint-State message from topic 
        sensor_msgs::JointStateConstPtr current_joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>(topic, ros::Duration(0.5));

        // Evaluate Current-Joint-State message
        if (!current_joint_state)
        {
            // Failed to capture Current-Joint-State message

            // Report to terminal
            ROS_ERROR_STREAM(Kinematics::CLASS_PREFIX << __FUNCTION__ 
                << ": Failed to capture current joint-state message");

            // Function return
            return false;
        }

        // Check for empty joint-state position
        if(current_joint_state->position.empty())
        {
            // Empty Joint-State Position

            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed: Current joint-state position is empty");

            // Function return
            return false;
        }

        // Update Joint-State to acquired current joint-state
        joint_state = *current_joint_state;

        // Function return
        return true;
    }


    // Get Current Joint-State
    // -------------------------------
    // (Function Overloading)
    bool Kinematics::getCurrentJointState(
            const std::string& topic,
            std::vector<double>& joint_position)
    {
        // Define local variable(s)
        sensor_msgs::JointState current_joint_state_;
        bool result_;

        // Get Current Joint-State
        if (Kinematics::getCurrentJointState(topic, current_joint_state_))
        {
            // Update Joint-State Position to acquired current joint-state posstion
            joint_position = current_joint_state_.position;

            // Function return
            return true;
        }
        
        // Get Current Joint-State failed
        return false;
    }


    // Get Current Transform
    // -------------------------------
    // (Function Overloading)
    bool Kinematics::getCurrentTransform(
            const std::string& target_frame,
            const std::string& ref_frame,
            geometry_msgs::TransformStamped& transform,
            bool print_result)
    {
        // Define Transform listener and buffer
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);

        // Try to get the specific transformation
        try
        {
            // Query listener for specific transformation
            transform = tf_buffer.lookupTransform(ref_frame,            // Reference Frame to which data should be transformed relative to
                                                  target_frame,         // Target Frame which data originates
                                                  ros::Time(0),         // Time at which value of transformed is desired (0 will get latest data)
                                                  ros::Duration(0.5));  // Duration before timeout 

        }
        // Catch exception(s)
        catch(tf2::TransformException &ex)
        {
            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Tranformation failed!: " << ex.what());

            // Function return
            return true;
        }
        
        // Print results to terminal
        if(print_result)
        {
            // Convert Quaternion-Orientation to Euler-Orientation
            geometry_msgs::Vector3 rpy = Common::quaternionToEuler(transform.transform.rotation, XYZ);

            ROS_INFO("Toolbox:getCurrentTransform:");
            ROS_INFO("--------------------");

            ROS_INFO("Pose:");
            ROS_INFO(" Position X: (%f)", transform.transform.translation.x);
            ROS_INFO(" Position Y: (%f)", transform.transform.translation.y);
            ROS_INFO(" Position Z: (%f)", transform.transform.translation.z);
            ROS_INFO(" ");
            ROS_INFO(" Orientation (deg) X: (%f)", Common::radToDeg(rpy.x));
            ROS_INFO(" Orientation (deg) Y: (%f)", Common::radToDeg(rpy.y));
            ROS_INFO(" Orientation (deg) Z: (%f)", Common::radToDeg(rpy.z));
            ROS_INFO(" ");
            ROS_INFO(" Orientation (rad) X: (%f)", rpy.x);
            ROS_INFO(" Orientation (rad) Y: (%f)", rpy.y);
            ROS_INFO(" Orientation (rad) Z: (%f)", rpy.z);
            ROS_INFO(" ");
            ROS_INFO(" Orientation (quat) X: (%f)", transform.transform.rotation.x);
            ROS_INFO(" Orientation (quat) Y: (%f)", transform.transform.rotation.y);
            ROS_INFO(" Orientation (quat) Z: (%f)", transform.transform.rotation.z);
            ROS_INFO(" Orientation (quat) W: (%f)", transform.transform.rotation.w);
            ROS_INFO(" ");
        }

        // Function successful
        return true;
    }


    // Get Current Transform
    // -------------------------------
    // (Function Overloading)
    bool Kinematics::getCurrentTransform(
            const std::string& target_frame,
            const std::string& ref_frame,
            geometry_msgs::Transform& transform,
            bool print_result)
    {
        // Define local variable(s)
        geometry_msgs::TransformStamped transform_stamped_;

        // Get Current Transform
        if (Kinematics::getCurrentTransform(target_frame, ref_frame, transform_stamped_, print_result))
        {
            // Get Transform
            transform = transform_stamped_.transform;

            // Function return
            return true;
        }
        
        // Get Current Pose failed
        return false;
    }


    // Get Current Transform
    // -------------------------------
    // (Function Overloading)
    bool Kinematics::getCurrentTransform(
            const std::string& target_frame,
            const std::string& ref_frame,
            Eigen::Isometry3d& transform,
            bool print_result)
    {
        // Define local variable(s)
        geometry_msgs::TransformStamped transform_stamped_;

        // Get Current Transform
        if (Kinematics::getCurrentTransform(target_frame, ref_frame, transform_stamped_, print_result))
        {
            // Convert Geometry-Transform to Eigen-Transform
            transform = tf2::transformToEigen(transform_stamped_);

            // Function return
            return true;
        }
        
        // Get Current Pose failed
        return false;
    }


    // Get Current Pose
    // -------------------------------
    // (Function Overloading)
    bool Kinematics::getCurrentPose(
            const std::string& target_frame,
            const std::string& ref_frame,
            geometry_msgs::PoseStamped& pose,
            bool print_result)
    {
        // Define local variable(s)
        geometry_msgs::TransformStamped transform_stamped_;

        // Get Current Transform
        if (Kinematics::getCurrentTransform(target_frame, ref_frame, transform_stamped_, print_result))
        {
            // Convert Transform-Stamped to Pose-Stamed
            pose = Common::transformToPose(transform_stamped_);

            // Function return
            return true;
        }
        
        // Get Current Pose failed
        return false;
    }


    // Get Current Pose
    // -------------------------------
    // (Function Overloading)
    bool Kinematics::getCurrentPose(
            const std::string& target_frame,
            const std::string& ref_frame,
            geometry_msgs::Pose& pose,
            bool print_result)
    {
        // Define local variable(s)
        geometry_msgs::PoseStamped pose_stamped_;

        // Get Current Transform
        if (Kinematics::getCurrentPose(target_frame, ref_frame, pose_stamped_, print_result))
        {
            // Convert Pose-Stamped to Pose
            pose = pose_stamped_.pose;

            // Function return
            return true;
        }
        
        // Get Current Pose failed
        return false;
    }


    // Get Current Pose
    // -------------------------------
    // (Function Overloading)
    bool Kinematics::getCurrentPose(
            const std::string& target_frame,
            const std::string& ref_frame,
            Eigen::Isometry3d& pose,
            bool print_result)
    {
        // Define local variable(s)
        geometry_msgs::Pose pose_;

        // Get Current Transform
        if (Kinematics::getCurrentPose(target_frame, ref_frame, pose_, print_result))
        {
            // Convert Geometry-Pose to Eigen-Pose
            Eigen::fromMsg(pose_, pose);

            // Function return
            return true;
        }
        
        // Get Current Pose failed
        return false;
    }

} // End Namespace: Robotics Toolbox