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

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robot_toolbox/tools/convert.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Convert Tool Class - Members:
// -------------------------------

    // Constants
    // -------------------------------
    const std::string Convert::CLASS_PREFIX = "Toolbox::Convert::";


    // Convert Degrees to Radians
    // -------------------------------
    // (Function Overloading)
    double Convert::degToRad(
        double deg)
    {
        // Convert deg to rad
        double rad = deg * M_PI / 180.0;

        // Function Output
        return rad;
    }


    // Convert Degrees to Radians
    // -------------------------------
    std::vector<double> Convert::degToRad(
        std::vector<double> deg_vec)
    {
        // Local variable
        std::vector<double> rad_vec;

        // Iterate over given vector of angles
        for (int i = 0; i < deg_vec.size(); i++)
        {
            // Convert deg to rad
            rad_vec.push_back(deg_vec[i] * M_PI / 180.0);
        }
        
        // Function Output
        return rad_vec;
    }


    // Convert Radians to Degrees
    // -------------------------------
    // (Function Overloading)
    double Convert::radToDeg(
        double rad)
    {
        // Convert deg to rad
        double deg = rad * 180.0 / M_PI;

        // Function Output
        return deg;
    }


    // Convert Radians to Degrees
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Convert::radToDeg(
        std::vector<double> rad_vec)
    {
        // Local variable
        std::vector<double> deg_vec;

        // Iterate over given vector of angles
        for (int i = 0; i < rad_vec.size(); i++)
        {
            // Convert deg to rad
            deg_vec.push_back(rad_vec[i] * 180.0 / M_PI);
        }

        // Function Output
        return deg_vec;
    }


    // Convert String to Upper-Case String
    // -------------------------------
    std::string Convert::stringToUpperCase(std::string input)
    {
        // Define local variable(s)
        std::string output;

        // Resize output to match input size
        output.resize(input.size());

        // Convert string to upper-case
        std::transform(input.begin(), input.end(), output.begin(), ::toupper);

        // Function return
        return output;
    } 


    // Convert Euler to Quaternion (eigen-vector)
    // -------------------------------
    // (Function Overloading)
    Eigen::Quaternion<double> Convert::eulerToQuaternion(
        Eigen::Vector3d euler,
        int seq)
    {
        // Define quaternion and euler rotations
        Eigen::Quaternion<double> q;
        Eigen::AngleAxisd phi;      // 1st Euler-Rotation
        Eigen::AngleAxisd theta;    // 2nd Euler-rotation
        Eigen::AngleAxisd psi;      // 3rd Euler-Roation
        
        // Determine Euler-Sequence and calculate rotation
        switch (seq)
        {
            case XYZ:
                // Calculate XYZ Euler-Sequence
                phi     = Eigen::AngleAxisd(euler(EULER_ID_PHI),      Eigen::Vector3d::UnitX());
                theta   = Eigen::AngleAxisd(euler(EULER_ID_THETA),    Eigen::Vector3d::UnitY());
                psi     = Eigen::AngleAxisd(euler(EULER_ID_PSI),      Eigen::Vector3d::UnitZ());

                // Case break
                break;

            case ZYX:
                // Calculate ZYX Euler-Sequence
                phi     = Eigen::AngleAxisd(euler(EULER_ID_PHI),      Eigen::Vector3d::UnitZ());
                theta   = Eigen::AngleAxisd(euler(EULER_ID_THETA),    Eigen::Vector3d::UnitY());
                psi     = Eigen::AngleAxisd(euler(EULER_ID_PSI),      Eigen::Vector3d::UnitX());

                // Case break
                break;

            case ZXZ:
                // Calculate ZXZ Euler-Sequence
                phi     = Eigen::AngleAxisd(euler(EULER_ID_PHI),      Eigen::Vector3d::UnitZ());
                theta   = Eigen::AngleAxisd(euler(EULER_ID_THETA),    Eigen::Vector3d::UnitX());
                psi     = Eigen::AngleAxisd(euler(EULER_ID_PSI),      Eigen::Vector3d::UnitZ());

                // Case break
                break;
                
            case ZYZ:
                // Calculate ZYZ Euler-Sequence
                phi     = Eigen::AngleAxisd(euler(EULER_ID_PHI),      Eigen::Vector3d::UnitZ());
                theta   = Eigen::AngleAxisd(euler(EULER_ID_THETA),    Eigen::Vector3d::UnitY());
                psi     = Eigen::AngleAxisd(euler(EULER_ID_PSI),      Eigen::Vector3d::UnitZ());
                
                // Case break
                break;

            // Unknown sequence
            default:
                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Unknown Euler-Sequence!");

                // Case break
                break;;
        }

        // Compute Quaternion based on calculated euler sequence
        q = phi * theta * psi;

        // Function return
        return q;
    }


    // Convert Euler to Quaternion (scalar)
    // -------------------------------
    // (Function Overloading)
    Eigen::Quaternion<double> Convert::eulerToQuaternion(
        double phi,
        double theta,
        double psi,
        int seq)
    {   
        // Define quaternion and euler rotations as a Eigen-Vector
        Eigen::Quaternion<double> q;
        Eigen::Vector3d euler(degToRad(phi), 
                              degToRad(theta), 
                              degToRad(psi));

        // Convert Euler-Rotation to Quaternion
        q = eulerToQuaternion(euler, seq);

        // Function return
        return q;
    }


    // Convert Euler to Quaternion (geometry_msgs)
    // -------------------------------
    // (Function Overloading)
    geometry_msgs::Quaternion Convert::eulerToQuaternion(
        geometry_msgs::Point euler,
        int seq)
    {
        // Define local variable(s)
        Eigen::Quaternion<double> q_eigen;
        Eigen::Vector3d euler_eigen;
        geometry_msgs::Quaternion q;
        geometry_msgs::Point euler_point;
        
        // Convert Geometry-Vector3 to Geometry-Point
        euler_point.x = euler.x;
        euler_point.y = euler.y;
        euler_point.z = euler.z;

        // Convert Geometry-Message to Eigen-Message
        Eigen::fromMsg(euler_point, euler_eigen);

        // Convert Euler-Rotation to Quaternion
        q_eigen = eulerToQuaternion(euler_eigen, seq);

        // Convert Eigen-Message to Geometry-Message
        q = Eigen::toMsg(q_eigen);

        // Function return
        return q;
    }


    // Convert Quaternion to Euler (quaternion-vector)
    // -------------------------------
    // (Function Overloading)
    Eigen::Vector3d Convert::quaternionToEuler(
        Eigen::Quaternion<double> q,
        int seq)
    {
        // Define rotation matrix and euler rotations as a Eigen-Vector
        Eigen::Matrix3d rm;
        Eigen::Vector3d euler;

        // Convert quaternion to rotation matrix
        rm = q.toRotationMatrix();

        // Determine Euler-Sequence and Compute euler rotation
        switch (seq)
        {
            case XYZ:
                // Calculate Euler-Angles for XYZ Euler-Sequence
                euler = rm.eulerAngles(AXIS_ID_X, AXIS_ID_Y, AXIS_ID_Z);

                // Case break
                break;

            case ZYX:
                // Compute Euler-Angles for ZYX Euler-Sequence
                euler = rm.eulerAngles(AXIS_ID_Z, AXIS_ID_Y, AXIS_ID_X);

                // Case break
                break;

            case ZXZ:
                // Calculate Euler-Angles for ZXZ Euler-Sequence
                euler = rm.eulerAngles(AXIS_ID_Z, AXIS_ID_X, AXIS_ID_Z);

                // Case break
                break;
                
            case ZYZ:
                // Calculate Euler-Angles for ZYZ Euler-Sequence
                euler = rm.eulerAngles(AXIS_ID_Z, AXIS_ID_Y, AXIS_ID_Z);
                
                // Case break
                break;

            // Unknown sequence
            default:
                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Unknown Euler-Sequence!");

                // Case break
                break;;
        }

        // Function return
        return euler;
    }


    // Convert Quaternion to Euler (scalar)
    // -------------------------------
    // (Function Overloading)
    Eigen::Vector3d Convert::quaternionToEuler(
        double w, 
        double x, 
        double y, 
        double z,
        int seq)
    {
        // Define quaternion and euler rotations as a Eigen-Vector
        Eigen::Quaternion<double> q(w, x, y, z);
        Eigen::Vector3d euler;
        
        // Convert Quaternion to Euler-Rotation 
        euler = quaternionToEuler(q, seq);

        // Function return
        return euler;
    }


    // Convert Quaternion to Euler (geometry_msgs)
    // -------------------------------
    // (Function Overloading)
    geometry_msgs::Point Convert::quaternionToEuler(
            geometry_msgs::Quaternion q,
            int seq)
    {
        // Define local variable(s)
        Eigen::Quaternion<double> q_eigen;
        Eigen::Vector3d euler_eigen;
        geometry_msgs::Point euler_point;
        geometry_msgs::Point euler;
        
        // Convert Geometry-Message to Eigen-Message
        Eigen::fromMsg(q, q_eigen);

        // Convert Quaternion to Euler-Rotation 
        euler_eigen = quaternionToEuler(q_eigen, seq);

        // Convert Eigen-Message to Geometry-Message
        euler_point = Eigen::toMsg(euler_eigen);

        // Convert Geometry-Point to Geometry-Vector3
        euler.x = euler_point.x;
        euler.y = euler_point.y;
        euler.z = euler_point.z;

        // Function return
        return euler;
    }
    

    // Convert Eigen-Vector to Std-Vector 
    // -------------------------------
    std::vector<double> Convert::vectorEigenToStd(
        Eigen::VectorXd v_in)
    {
        // Define vector
        std::vector<double> v;

        // Resize vector to equal the size of incomming vector
        v.resize(v_in.size());

        // Convert from Eigen::VectorXd to std::vector<double>
        Eigen::Map<Eigen::VectorXd>(v.data(), v.size()) = v_in;

        // Function return
        return v;
    }


    // Convert Std-Vector to Eigen-Vector
    // -------------------------------
    Eigen::VectorXd Convert::vectorStdToEigen(
        std::vector<double> v_in)
    {
        // Define vector
        Eigen::VectorXd v;

        // Convert from std::vector<double> to Eigen::VectorXd
        v = Eigen::Map<Eigen::VectorXd>(v_in.data(), v_in.size());

        // Function return
        return v;
    }


    // Convert Pose to Transform
    // -------------------------------
    // (Function overloading)
    geometry_msgs::Transform Convert::poseToTransform(
        geometry_msgs::Pose pose)
    {
        // Define Transform variable holder
        geometry_msgs::Transform transform;
        
        // Convert Pose to Transform
        // Translation
        transform.translation.x = pose.position.x;
        transform.translation.y = pose.position.y;
        transform.translation.z = pose.position.z;

        // Rotation
        transform.rotation = pose.orientation;
    
        // Function Return
        return transform;
    }


    // Convert Pose to Transform-Stamped 
    // -------------------------------
    // (Function overloading)
    geometry_msgs::TransformStamped Convert::poseToTransform(
        geometry_msgs::Pose pose,
        std::string parent_frame,
        std::string child_frame)
    {
        // Define Transform-Stamped variable holder
        geometry_msgs::TransformStamped transform_stamped;
        
        // Transform-Stamped Header
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = parent_frame;
        transform_stamped.child_frame_id = child_frame;

        // Convert Pose to Transform
        transform_stamped.transform = poseToTransform(pose);

        // Function Return
        return transform_stamped;
    }


    // Convert Pose to Transform-Stamped 
    // -------------------------------
    // (Function overloading)
    geometry_msgs::TransformStamped Convert::poseToTransform(
        geometry_msgs::PoseStamped pose_stamped)
    {
        // Define Transform-Stamped variable holder
        geometry_msgs::TransformStamped transform_stamped;
        
        // Transform-Stamped Header
        transform_stamped.header = pose_stamped.header;

        // Convert Pose to Transform
        transform_stamped.transform = poseToTransform(pose_stamped.pose);

        // Function Return
        return transform_stamped;
    }


    // Convert Transform to Pose 
    // -------------------------------
    // (Function Overloading)
    geometry_msgs::Pose Convert::transformToPose(
        geometry_msgs::Transform transform)
    {
        // Define Pose variable holder
        geometry_msgs::Pose pose;
        
        // Convert Pose to Transform
        // Translation
        pose.position.x = transform.translation.x; 
        pose.position.y = transform.translation.y; 
        pose.position.z = transform.translation.z; 

        // Orientation
        pose.orientation = transform.rotation;
    
        // Function Return
        return pose;
    }


    // Convert Transform to Pose-Stamped 
    // -------------------------------
    // (Function Overloading)
    geometry_msgs::PoseStamped Convert::transformToPose(
        geometry_msgs::Transform transform,
        std::string parent_frame)
    {
        // Define Pose-Stamped variable holder
        geometry_msgs::PoseStamped pose_stamped;
        
        // Pose-Stamped Header
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = parent_frame;

        // Convert Pose to Transform
        pose_stamped.pose = transformToPose(transform);

        // Function Return
        return pose_stamped;
    }


    // Convert Transform-Stamped to Pose-Stamped
    // -------------------------------
    // (Function Overloading)
    geometry_msgs::PoseStamped Convert::transformToPose(
        geometry_msgs::TransformStamped transform_stamped)
    {
        // Define Pose-Stamped variable holder
        geometry_msgs::PoseStamped pose_stamped;
        
        // Transform-Stamped Header
        pose_stamped.header = transform_stamped.header;

        // Convert Pose to Transform
        pose_stamped.pose = transformToPose(transform_stamped.transform);

        // Function Return
        return pose_stamped;
    }


    // Convert Pose [m, rad] to Pose RPY [m, deg]
    // -------------------------------
     robot_toolbox::PoseRPY Convert::poseToPoseRPY(
        geometry_msgs::Pose pose)
    {
        // Define PoseRPY variable holder
        robot_toolbox::PoseRPY pose_rpy;

        // Convert Pose to PoseRPY
        // Translation
        pose_rpy.position = pose.position;

        // Orientation
        pose_rpy.orientation = quaternionToEuler(pose.orientation);

        // Convert from rad to deg
        pose_rpy.orientation.x = radToDeg(pose_rpy.orientation.x);
        pose_rpy.orientation.y = radToDeg(pose_rpy.orientation.y);
        pose_rpy.orientation.z = radToDeg(pose_rpy.orientation.z);

        // Function Return
        return pose_rpy;
    }


    // Convert Pose RPY [m, deg] to Pose [m, rad]
    // -------------------------------
    geometry_msgs::Pose Convert::poseRPYToPose(
        robot_toolbox::PoseRPY poseRPY)
    {
        // Define Pose variable holder
        geometry_msgs::Pose pose;

        // Convert from deg to rad
        poseRPY.orientation.x = degToRad(poseRPY.orientation.x);
        poseRPY.orientation.y = degToRad(poseRPY.orientation.y);
        poseRPY.orientation.z = degToRad(poseRPY.orientation.z);

        // Convert PoseRPY to Pose
        // Translation
        pose.position = poseRPY.position;

        // Orientation
        pose.orientation = eulerToQuaternion(poseRPY.orientation);

        // Function Return
        return pose;
    }
        
} // End Namespace: Robotics Toolbox