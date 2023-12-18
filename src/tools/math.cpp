// Robotics Toolbox - Math Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Math Tools contains helper and utility functions 
//      related to mathematical calculations
//
// Version:
//  0.1 - Initial Version
//        [06.01.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robot_toolbox/tools/math.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Math Tool Class - Members:
// -------------------------------

    // Constants
    // -------------------------------
    const std::string Math::CLASS_PREFIX = "Toolbox::Math::";

    // Linear Spaced Vector (double)
    // -------------------------------
    // (Function overloading)
    std::vector<double> Math::linspace(
        double p_s, 
        double p_f, 
        int n)
    {
        // Define linear spaced vector
        std::vector<double> linspaced;

        // Check number of points
        if (n == 0)
        {
            // Empty linspace

            // Function return
            return linspaced;
        }
        // Only one point
        else if (n == 1)
        {
            // Assign only end-point
            linspaced.push_back(p_f);

            // Function return
            return linspaced;
        }

        // Calculate delta-spacer
        double delta = (p_f - p_s) / (n - 1);
        
        // Generate linear space
        for (int i = 0; i < (n-1); i++)
        {
            // Assign current point to linspace vector
            linspaced.push_back(p_s + delta*i);
        }
        
        // Assign last element of linspace vector to equal finish-point
        linspaced.push_back(p_f);

        // Function return
        return linspaced;
    }


    // Linear Spaced Vector (Eigen::Vector3d)
    // -------------------------------
    // (Function overloading)
    std::vector<Eigen::Vector3d> Math::linspace(
        Eigen::Vector3d p_s, 
        Eigen::Vector3d p_f, 
        int n)
    {
        // Define linear spaced vector and local variables
        std::vector<Eigen::Vector3d> linspaced;
        std::vector<double> x, y, z;    

        // Generate linear space for each element of the Eigen::Vector3d
        // -------------------------------
        x = linspace(p_s[AXIS_ID_X], p_f[AXIS_ID_X], n);
        y = linspace(p_s[AXIS_ID_Y], p_f[AXIS_ID_Y], n);
        z = linspace(p_s[AXIS_ID_Z], p_f[AXIS_ID_Z], n);

        // Iterate over the number of points
        for (int i = 0; i < n; i++)
        {
            // Assign current element values to a point eigen vector
            Eigen::Vector3d point(x[i], y[i], z[i]);

            // Appned current point to linspace vector
            linspaced.push_back(point);
        }

        // Function return
        return linspaced;
    }


    // Linear Interpolation (double)
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Math::lerp(
        double p_s, 
        double p_f, 
        int n)
    {
        // Define linear interpolation vector
        std::vector<double> interpolation;

        // Compute time interval-vector
        // (interval vector is a closed unit interval [0,1]
        // using linspace to get evenly spaced vector with n-points) 
        std::vector<double> t = linspace(0.0, 1.0, n);

        // Iterate over interval-vector
        for (int i = 0; i < t.size(); i++)
        {
            // Get timestep
            double td = t[i];

            // Calculate interpolation point
            double point = ((1 - td) * p_s) + (td * p_f);

            // Append interpolation point to vector
            interpolation.push_back(point);
        }

        // Function return
        return interpolation;
    }


    // Linear Interpolation (Eigen::Vector3d) 
    // -------------------------------
    // (Function Overloading)
    std::vector<Eigen::Vector3d> Math::lerp(
        Eigen::Vector3d p_s, 
        Eigen::Vector3d p_f, 
        int n)
    {
        // Define linear interpolation vector and local variables
        std::vector<Eigen::Vector3d> lerp_vec;
        std::vector<double> x, y, z;    

        // Generate Linear Interpolation for each element of the Eigen::Vector3d
        // -------------------------------
        x = lerp(p_s[AXIS_ID_X], p_f[AXIS_ID_X], n);
        y = lerp(p_s[AXIS_ID_Y], p_f[AXIS_ID_Y], n);
        z = lerp(p_s[AXIS_ID_Z], p_f[AXIS_ID_Z], n);

        // Iterate over the number of points
        for (int i = 0; i < n; i++)
        {
            // Assign current element values to a point eigen vector
            Eigen::Vector3d point(x[i], y[i], z[i]);

            // Appned current point to linear interpolation vector
            lerp_vec.push_back(point);
        }

        // Function return
        return lerp_vec;
    }


    // Linear Interpolation (double)
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Math::lerp(
        double p_s, 
        double p_f, 
        double dt)
    {
        // Define linear interpolation vector
        std::vector<double> interpolation;

        // Calculate number of steps
        const double distance = std::abs(p_f - p_s);
        int n = std::floor(distance / dt) + 1;

        // Generate Linear-Interpolation
        interpolation = lerp(p_s, p_f, n);

        // Function return
        return interpolation;
    }


    // Linear Interpolation (Eigen::Vector3d) 
    // -------------------------------
    // (Function Overloading)
    std::vector<Eigen::Vector3d> Math::lerp(
        Eigen::Vector3d p_s, 
        Eigen::Vector3d p_f, 
        double dt)
    {
        // Define linear interpolation vector and local variables
        std::vector<Eigen::Vector3d> lerp_vec;
        std::vector<double> x, y, z;    

        // Calculate number of steps
        const Eigen::Vector3d distance = p_f - p_s;
        int n = std::floor(distance.norm() / dt) + 1;

        // Generate Linear-Interpolation
        lerp_vec = lerp(p_s, p_f, n);

        // Function return
        return lerp_vec;
    }

    // Spherical Linear Interpolation (Eigen::Quaterniond)
    // -------------------------------
    // (Function Overloading)
    std::vector<Eigen::Quaterniond> Math::slerp(
        Eigen::Quaternion<double> q_s, 
        Eigen::Quaternion<double> q_f, 
        int n)
    {
        // Define spherical linear interpolation vector and local variables
        std::vector<Eigen::Quaternion<double>> interpolation;

        // Compute time interval-vector
        // (interval vector is a closed unit interval [0,1]
        // using linspace to get evenly spaced vector with n-points) 
        std::vector<double> t = linspace(0.0, 1.0, n);

        // Iterate over interval-vector
        for (int i = 0; i < t.size(); i++)
        {
            // Get timestep
            double td = t[i];

            // Calculate spherical linear interpolation point
            Eigen::Quaternion<double> q = q_s.slerp(td, q_f);

            // Append interpolation point to vector
            interpolation.push_back(q);
        }

        // Function return
        return interpolation;
    }


    // Spherical Linear Interpolation (Eigen::Vector3d)
    // -------------------------------
    // (Function Overloading)
    std::vector<Eigen::Vector3d> Math::slerp(
        Eigen::Vector3d r_s, 
        Eigen::Vector3d r_f, 
        int n,
        int euler_seq)
    {
        // Define spherical linear interpolation vector and local variables
        std::vector<Eigen::Vector3d> interpolation;
        std::vector<Eigen::Quaternion<double>> q_interpolation;
        Eigen::Quaternion<double> q_s; 
        Eigen::Quaternion<double> q_f;

        // Convert Euler-Angles to Quaternion  
        q_s = Convert::eulerToQuaternion(r_s, euler_seq);
        q_f = Convert::eulerToQuaternion(r_f, euler_seq);

        // Calculate Spherical Linear Interpolation
        // (computed with quaternions)
        q_interpolation = slerp(q_s, q_f, n);

        // Convert Quaternion-Interpolation vector to Euler-Interpolation vector
        // Iterate over Quaternion-Interpolation vector
        for (size_t i = 0; i < q_interpolation.size(); i++)
        {
            // Get current quaternion of vector
            Eigen::Quaternion<double> q = q_interpolation[i];

            // Convert Quaternion to Euler-Angles
            Eigen::Vector3d euler = Convert::quaternionToEuler(q, euler_seq);

            // Append Euler-Angles to Interpolation vector
            interpolation.push_back(euler);
        }

        // Function return
        return interpolation;
    }


    // Spherical Linear Interpolation (Eigen::Quaterniond)
    // -------------------------------
    // (Function Overloading)
    std::vector<Eigen::Quaterniond> Math::slerp(
        Eigen::Quaternion<double> q_s, 
        Eigen::Quaternion<double> q_f, 
        double dt)
    {
        // Define spherical linear interpolation vector and local variables
        std::vector<Eigen::Quaternion<double>> interpolation;

        // Calculate number of steps
        const Eigen::Quaternion<double> difference = q_f * q_s.inverse();
        int n = std::floor(difference.norm() / dt) + 1;

        // Generate Linear-Interpolation
        interpolation = slerp(q_s, q_f, n);

        // Function return
        return interpolation;
    }


    // Spherical Linear Interpolation (Eigen::Vector3d)
    // -------------------------------
    // (Function Overloading)
    std::vector<Eigen::Vector3d> Math::slerp(
        Eigen::Vector3d r_s, 
        Eigen::Vector3d r_f, 
        double dt,
        int euler_seq)
    {
        // Define spherical linear interpolation vector and local variables
        std::vector<Eigen::Vector3d> interpolation;

        // Calculate number of steps
        const Eigen::Vector3d distance = r_f - r_s;
        int n = std::floor(distance.norm() / dt) + 1;
        
        // Generate Linear-Interpolation
        interpolation = slerp(r_s, r_f, n);

        // Function return
        return interpolation;
    }


    // Evaluate Polynomial
    // -------------------------------
    // (Function Overloading)
    double Math::polyval(
        std::vector<double> p, 
        double x)
    {
        // Define evaluation value
        double value;

        // Evaluate value of polynomial using Horner's method
        for (int i = 0; i < p.size(); i++)
        {
            value = value * x + p[i];
        }
        
        // Function return
        return value;
    }


    // Evaluate Polynomial
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Math::polyval(
        std::vector<double> p, 
        std::vector<double> x)
    {
        // Define evaluation values
        std::vector<double> values;

        // Evaluate value of polynomial using Horner's method
        // for each evaluation point
        for (int i = 0; i < x.size(); i++)
        {
            // Evaluate at current x-value
            double value = polyval(p, x[i]);

            // Append result to values
            values.push_back(value);
        }
        
        // Function return
        return values;
    }


    // Evaluate Polynomial
    // -------------------------------
    // (Function Overloading)
    double Math::polyval(
        Eigen::VectorXd p, 
        double x)
    {
        // Define evaluation value
        double value;

        // Convert Eigen::VectorXd to std::vector<double>
        std::vector<double> vec;    // Define vector
        vec.resize(p.size());       // Resize to allocate memory 
        Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size()) = p;    // Convert from Eigen::VectorX to std::vector

        // Evaluate polynomial
        value = polyval(vec, x);

        // Function return
        return value;
    }


    // Evaluate Polynomial
    // -------------------------------
    // (Function Overloading)
    Eigen::VectorXd Math::polyval(
        Eigen::VectorXd p, 
        Eigen::VectorXd x)
    {
        // Define evaluation value
        Eigen::VectorXd values(x.size());

        // Evaluate value of polynomial using Horner's method
        // for each evaluation point
        for (int i = 0; i < x.size(); i++)
        {
            // Evaluate at current x-value
            double value = polyval(p, x[i]);

            // Append result to values
            values[i] = value;
        }

        // Function return
        return values;
    }


    // Get Normal Vector (Transformation Matrix)
    // -------------------------------
    // (Function Overloading)
    Eigen::Vector3d Math::getNormalVector(
        Eigen::Isometry3d tm,
        AxisType axis_type)
    {
        // Local variable holder(s)
        Eigen::Vector3d normal_vector;  // Normal-Vector of specifed axis (function return)

        // Calculate Axis-Type Normal-Vector represented as relative to given Transformation Matrix
        normal_vector = tm.rotation() * axis_type.unit_vec;

        // Function return
        return normal_vector;
    }   


    // Get Normal Vector (Pose)
    // -------------------------------
    // (Function Overloading)
    Eigen::Vector3d Math::getNormalVector(
        geometry_msgs::Pose pose,
        AxisType axis_type)
    {
        // Local variable holder(s)
        Eigen::Vector3d normal_vector;  // Normal-Vector of specifed axis (function return)
        Eigen::Isometry3d tm;           // Isometry Transformation Matrix

        // Get Transformation Matrix of Pose
        tf2::fromMsg(pose, tm);

        // Calculate Axis-Type Normal-Vector represented as relative to given Transformation Pose
        normal_vector = tm.rotation() * axis_type.unit_vec;

        // Function return
        return normal_vector;
    }


    // Rotation Matrix - Quaternion-Vector
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMat(
        Eigen::Quaternion<double> q)
    {
        // Define Rotation Matrix
        Eigen::Matrix3d rm;
        
        // Calculate Rotation Matrix
        rm = q.toRotationMatrix();

        // Function return
        return rm;
    }


    // Rotation Matrix - Quaternion-Scalar
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMat(
        double w, 
        double x, 
        double y, 
        double z)
    {
        // Define Rotation Matrix and Quaternion
        Eigen::Matrix3d rm;
        Eigen::Quaternion<double> q(w, x, y, z);

        // Calculate Rotation Matrix
        rm = rotMat(q);

        // Function return
        return rm;
    }


    // Rotation Matrix - Euler-Vector
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMat(
        Eigen::Vector3d euler,
        int seq)
    {
        // Define Rotation Matrix and Euler-Angles
        Eigen::Matrix3d rm;
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
                ROS_ERROR_STREAM(Math::CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Unknown Euler-Sequence!");

                // Case break
                break;;
        }

        // Compute Rotation Matrix based on calculated euler sequence
        rm = phi * theta * psi;

        // Function return
        return rm;
    }


    // Rotation Matrix - Euler-Scalar
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMat(
        double phi,
        double theta,
        double psi,
        int seq)
    {
        // Define Rotation Matrix and Eigen-Vector
        Eigen::Matrix3d rm;
        Eigen::Vector3d euler(Convert::degToRad(phi), 
                              Convert::degToRad(theta), 
                              Convert::degToRad(psi));
                              
        // Calculate Rotation Matrix
        rm = rotMat(euler, seq);

        // Function return
        return rm;
    }


    // Rotation Matrix - Euler XYZ-Sequence
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatXYZ(
        Eigen::Vector3d euler)
    {
        // Define Rotation Matrix
        Eigen::Matrix3d rm;
        
        // Calculate Rotation Matrix
        rm = rotMat(euler, XYZ);

        // Function return
        return rm;
    }


    // Rotation Matrix - Euler-Scalar XYZ-Sequence
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatXYZ(
        double rot_x, 
        double rot_y, 
        double rot_z)
    {
        // Define Rotation Matrix
        Eigen::Matrix3d rm;
        
        // Calculate Rotation Matrix
        rm = rotMat(rot_x, rot_y, rot_z, XYZ);

        // Function return
        return rm;
    }


    // Rotation Matrix - ZYX-Sequence
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatZYX(
        Eigen::Vector3d euler)
    {
        // Define Rotation Matrix
        Eigen::Matrix3d rm;
        
        // Calculate Rotation Matrix
        rm = rotMat(euler, ZYX);

        // Function return
        return rm;
    }


    // Rotation Matrix - Euler-Scalar ZYX-Sequence
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatZYX(
        double rot_z, 
        double rot_y, 
        double rot_x)
    {
        // Define Rotation Matrix
        Eigen::Matrix3d rm;
        
        // Calculate Rotation Matrix
        rm = rotMat(rot_z, rot_y, rot_x, ZYX);

        // Function return
        return rm;
    }


    // Rotation Matrix - Euler ZXZ-Sequence
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatZXZ(
        Eigen::Vector3d euler)
    {
        // Define Rotation Matrix
        Eigen::Matrix3d rm;
        
        // Calculate Rotation Matrix
        rm = rotMat(euler, ZXZ);

        // Function return
        return rm;
    }


    // Rotation Matrix - Euler-Scalar ZXZ-Sequence
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatZXZ(
        double rot_z1, 
        double rot_x, 
        double rot_z2)
    {
        // Define Rotation Matrix
        Eigen::Matrix3d rm;
        
        // Calculate Rotation Matrix
        rm = rotMat(rot_z1, rot_x, rot_z2, ZXZ);

        // Function return
        return rm;
    }


    // Rotation Matrix - Euler ZYZ-Sequence
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatZYZ(
        Eigen::Vector3d euler)
    {
        // Define Rotation Matrix
        Eigen::Matrix3d rm;
        
        // Calculate Rotation Matrix
        rm = rotMat(euler, ZYZ);

        // Function return
        return rm;
    }


    // Rotation Matrix - Euler-Scalar ZYZ-Sequence
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatZYZ(
        double rot_z1, 
        double rot_y, 
        double rot_z2)
    {
        // Define Rotation Matrix
        Eigen::Matrix3d rm;
        
        // Calculate Rotation Matrix
        rm = rotMat(rot_z1, rot_y, rot_z2, ZYZ);

        // Function return
        return rm;
    }


    // Transformation Matrix
    // -------------------------------
    // (Function Overloading)
    Eigen::Isometry3d Math::transMat(
        Eigen::Vector3d pos_vec, 
        Eigen::Matrix3d rot_mat)
    {
        // Define Transformation-Matrix
        Eigen::Isometry3d tm;

        // Calculate Transformation Matrix        
        tm.translation() = pos_vec; // Translation
        tm.linear() = rot_mat;      // Rotation

        // Function return
        return tm;
    }


    // Transformation Matrix
    // -------------------------------
    // (Function Overloading)
    Eigen::Isometry3d Math::transMat(
        Eigen::Vector3d pos_vec, 
        Eigen::Quaternion<double> quat)
    {
        // Define Transformation-Matrix
        Eigen::Isometry3d tm;
        
        // Calculate Transformation Matrix        
        tm.translation() = pos_vec;     // Translation
        tm.linear() = quat.matrix();    // Rotation

        // Function return
        return tm;
    }


    // Transformation Matrix - Euler XYZ
    // -------------------------------
    // (Function Overloading)
    Eigen::Isometry3d Math::transMat(
        Eigen::Vector3d pos_vec, 
        Eigen::Vector3d rot_vec, 
        int euler_seq)
    {
        // Define Transformation-Matrix and Rotation-Matrix
        Eigen::Isometry3d tm;
        Eigen::Matrix3d rm;

        // Calculate Rotation-Matrix
        rm = rotMat(rot_vec, euler_seq);

        // Calculate Transformation Matrix        
        tm.translation() = pos_vec; // Translation
        tm.linear() = rm;           // Rotation

        // Function return
        return tm;
    }


    // Transformation Matrix - Euler
    // -------------------------------
    // (Function Overloading)
    Eigen::Isometry3d Math::transMat(
        double x, 
        double y, 
        double z,
        double phi, 
        double theta, 
        double psi,
        int euler_seq)
    {
        // Define Transformation-Matrix, Position and Rotation-Vector
        Eigen::Isometry3d tm;
        Eigen::Vector3d pos_vec(x, y, z);
        Eigen::Vector3d rot_vec(Convert::degToRad(phi), 
                                Convert::degToRad(theta), 
                                Convert::degToRad(psi));
        
        // Transformation-Matrix
        tm = transMat(pos_vec, rot_vec, euler_seq);

        // Function return
        return tm;                       
    }


    // Numerical Differentiation
    // -------------------------------
    double Math::numericalDifferentiation(
        double& x, 
        double& x_prev, 
        double& dt)
    {
        // Check for postive dt
        if(dt < 0) return 0.0;

        // Calculate derivative using numerical differentiation
        double x_dt = (x - x_prev) / dt;

        // Function return
        return x_dt;
    }
} // End Namespace: Robotics Toolbox