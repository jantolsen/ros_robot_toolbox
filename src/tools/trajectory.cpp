// Robotics Toolbox - Trajectory Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Trajectory Tools contains helper and utility functions 
//      related to trajectory generation
//
// Version:
//  0.1 - Initial Version
//        [09.01.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robot_toolbox/tools/trajectory.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Trajectory Tool Class - Members:
// -------------------------------
    
    // Constants
    // -------------------------------
    const std::string Trajectory::CLASS_PREFIX = "Toolbox::Trajectory::";


    // Linear Segment with Parabolic Blends 
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Trajectory::lspb(
        const double &p_s, 
        const double &p_f, 
        const std::vector<double> &t)
    {
        // Define local variables
        double pos;                     // Position
        double vel;                     // Velocity
        double acc;                     // Acceleration
        std::vector<double> trajectory; // LSBP trajectory (position)
        std::vector<double> pos_traj;   // Position trajectory
        std::vector<double> vel_traj;   // Velocity trajectory
        std::vector<double> acc_traj;   // Acceleration trajectory
        std::string func_prefix = "lspb";
        
        // Illegal argument handling
        // -------------------------------
            // Check trajectory period
            if (!validateTrajectoryPeriod(t, p_f, func_prefix, &trajectory))
            {
                // Function return
                return trajectory;
            }

            // Identical start- and end-point
            if(p_s == p_f)
            {
                // Assign values to trajectory
                trajectory = std::vector<double>(t.size(), p_s);    // Fill with start-point values
                pos_traj = std::vector<double>(t.size(), p_s);      // Fill with start-point values
                vel_traj = std::vector<double>(t.size(), 0.0);      // Fill with zeros
                acc_traj = std::vector<double>(t.size(), 0.0);      // Fill with zeros

                // Report to terminal
                ROS_ERROR_STREAM(Trajectory::CLASS_PREFIX + func_prefix 
                    << ": Failed! End-Point equals End-Point, returning trajectory with only end-point values");

                // Function return
                return trajectory;
            }

        // Calculation
        // -------------------------------
            // Get final time 
            // (equal to last element of time-vector)
            double tf = t.back();

            // Compute velocity
            double v = ((p_f - p_s) / tf) * 1.5;

            // Compute blending time
            double tb = (p_s - p_f + (v * tf)) / v;

            // Compute alpha
            // (Helper variable)
            double alpha = v / tb;

            // Iterate over the time-vector
            // (calculate trajectory components)
            for (int i = 0; i < t.size(); i++)
            {
                // Get timestep
                double td = t[i];

                // Initial blending motion
                if (td <= tb)
                {
                    // Calculate trajectory components
                    pos = p_s + ((alpha / 2) * pow(td, 2)); // quadratic polynomial
                    vel = alpha * td;                       // linear ramp 
                    acc = alpha;                            // constant acceleration
                }
                // Linear motion
                else if (td <= (tf - tb))
                {
                    // Calculate trajectory components
                    pos = (p_f + p_s - (v * tf)) / 2 + (v * td);    // linear position
                    vel = v;                                        // constant velocity 
                    acc = 0;                                        // zero acceleration
                }
                // Final blending motion
                else
                {
                    pos = p_f - ((alpha / 2) * pow(tf, 2)) + (alpha * tf * td) - ((alpha / 2) * pow(td, 2));    // quadratic polynomial
                    vel = (alpha * tf) - (alpha * td);                  // linear ramp 
                    acc = -alpha;                                       // constant acceleration
                }
                
                // Append trajectory components to respective vectors
                pos_traj.push_back(pos);
                vel_traj.push_back(vel);
                acc_traj.push_back(acc);

                // Append trajectory points (position) to LSPB-Trajectory
                trajectory.push_back(pos);
            }

        // Function return
        return trajectory;
    }


    // Linear Segment with Parabolic Blends 
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Trajectory::lspb(
        const double &p_s, 
        const double &p_f, 
        const int &n)
    {
        // Define trajectory and lcoal variables
        std::vector<double> trajectory;
        std::string func_prefix = "lspb";

        // Illegal argument handling
        // -------------------------------
            // Check trajectory period
            if (!validateTrajectoryPeriod(n, p_f, func_prefix, &trajectory))
            {
                // Function return
                return trajectory;
            }

        // Calculation
        // -------------------------------
            // Compute time-vector 
            // (using linspace to get evenly spaced vector with n-points) 
            std::vector<double> t = Math::linspace(0.0, (n-1), n);

            // Calculate Linear Segment with Parabolic Blends 
            trajectory = lspb(p_s, p_f, t);

        // Function return
        return trajectory;
    }


    // Linear Segment with Parabolic Blends 
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Trajectory::lspb(
        const double &p_s, 
        const double &p_f, 
        const double &dt)
    {
        // Define trajectory and lcoal variables
        std::vector<double> trajectory;
        std::string func_prefix = "lspb: ";
        
        // Calculate number of steps
        const double distance = std::abs(p_f - p_s);
        int n = std::floor(distance / dt) + 1;

        // Calculate Linear Segment with Parabolic Blends 
        trajectory = lspb(p_s, p_f, n);

        // Function return
        return trajectory;
    }
    

    // Quintic Polynomial Trajectory
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Trajectory::polyQuintic(
        double p_s, 
        double p_f, 
        std::vector<double> t,
        double v_s, 
        double v_f)
    {
        // Define local variables
        std::vector<double> trajectory; // Polynomial trajectory (position)
        Eigen::VectorXd pos_traj;       // Position trajectory
        Eigen::VectorXd vel_traj;       // Velocity trajectory
        Eigen::VectorXd acc_traj;       // Acceleration trajectory
        std::string func_prefix = "polynomialQuintic";

        // Illegal argument handling
        // -------------------------------
            // Check trajectory period
            if (!validateTrajectoryPeriod(t, p_f, func_prefix, &trajectory))
            {
                // Function return
                return trajectory;
            }

            // Identical start- and end-point
            if(p_s == p_f)
            {
                // Assign values to trajectory
                trajectory = std::vector<double>(t.size(), p_s);    // Fill with start-point values
                pos_traj = Eigen::VectorXd::Ones(t.size()) * p_f;   // Fill with start-point values
                vel_traj = Eigen::VectorXd::Zero(t.size());         // Fill with zeros
                acc_traj = Eigen::VectorXd::Zero(t.size());         // Fill with zeros
                
                // Function return
                return trajectory;
            }

        // Calculation
        // -------------------------------
            // Get final time 
            // (equal to last element of time-vector)
            double tf = t.back();

            // Define the polynomial as a system of linear equation expressed in matrix form (Ax = b)
            // Then use x = inv(A) * b to solve for the polynomial coefficients

            // Matrix-equation
            Eigen::MatrixXd m(6,6);
            m << 0,                 0,                  0,                  0,              0,          1,      // q-start equation (position)
                 pow(tf, 5),        pow(tf, 4),         pow(tf, 3),         pow(tf, 2),     tf,         1,      // q-final quation (position)
                 0,                 0,                  0,                  0,              1,          0,      // qd-start equation (velocity)
                 5 * pow(tf, 4),    4 * pow(tf, 3),     3 * pow(tf, 2),     2 *tf,          1,          0,      // qd-final equation (velocity)
                 0,                 0,                  0,                  2,              0,          0,      // qdd-start equation (acceleration)
                 20 * pow(tf, 3),   12 * pow(tf, 2),    6 * tf,             2,              0,          0;      // qdd-final equation (acceleration)
            
            // Initial- and final-values as equation solutions
            Eigen::VectorXd q(6);
            q << p_s, p_f, v_s, v_f, 0, 0;

            // Calculate Coefficients for position
            // (using x = inv(A) * b )
            Eigen::VectorXd c(6);       
            c = m.inverse() * q;    
            
            // Calculate Coefficients for velocity
            // (Multiplying derivative values with position-coefficients)  
            Eigen::VectorXd c_d(5);     
            c_d << (5 * c[0]),
                   (4 * c[1]),
                   (3 * c[2]),
                   (2 * c[3]), 
                   (1 * c[4]);

            // Coefficients for acceleration
            // (Multiplying derivative values with velocity-coefficients)
            Eigen::VectorXd c_dd(4);
            c_dd << (4 * c_d[0]),
                    (3 * c_d[1]),
                    (2 * c_d[2]),
                    (1 * c_d[3]);

            // Evaluate polynomials
            // (converting time-vector from std::Vector<> to Eigen::VectorX)
            pos_traj = Math::polyval(c, Convert::vectorStdToEigen(t));
            vel_traj = Math::polyval(c_d, Convert::vectorStdToEigen(t));
            acc_traj = Math::polyval(c_dd, Convert::vectorStdToEigen(t));

        // Resize polynomial trajectory to equal time-period size
        trajectory.resize(t.size());

        // Convert Trajectory Eigen::VectorXd to std::vector<double>
        trajectory = Convert::vectorEigenToStd(pos_traj);

        // Function return
        return trajectory;
    }


    // Quintic Polynomial Trajectory
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Trajectory::polyQuintic(
        double p_s, 
        double p_f, 
        int n,
        double v_s, 
        double v_f)
    {
        // Define polynomial trajectory
        std::vector<double> trajectory; // Polynomial trajectory (position)
        std::string func_prefix = "polynomialQuintic"; 

        // Illegal argument handling
        // -------------------------------
            // Check trajectory period
            if (!validateTrajectoryPeriod(n, p_f, func_prefix, &trajectory))
            {
                // Function return
                return trajectory;
            }

        // Calculation
        // -------------------------------
            // Compute time-vector 
            // (using linspace to get evenly spaced vector with n-points) 
            std::vector<double> t = Math::linspace(0.0, (n-1), n);

            // Calculate Quintic-Polynomial
            trajectory = polyQuintic(p_s, p_f, t, v_s, v_f);

        // Function return
        return trajectory;
    }


    // Quintic Polynomial Trajectory
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Trajectory::polyQuintic(
        double p_s, 
        double p_f, 
        double dt,
        double v_s, 
        double v_f)
    {
        // Define trajectory and lcoal variables
        std::vector<double> trajectory;
        
        // Calculate number of steps
        const double distance = std::abs(p_f - p_s);
        int n = std::floor(distance / dt) + 1;

        // Calculate Quintic-Polynomial
        trajectory = polyQuintic(p_s, p_f, n, v_s, v_f);

        // Function return
        return trajectory;
    }


    // Cubic Polynomial Trajectory
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Trajectory::polyCubic(
        double p_s, 
        double p_f, 
        std::vector<double> t,
        double v_s, 
        double v_f)
    {
        // Define local variables
        std::vector<double> trajectory; // Polynomial trajectory (position)
        Eigen::VectorXd pos_traj;       // Position trajectory
        Eigen::VectorXd vel_traj;       // Velocity trajectory
        std::string func_prefix = "polynomialCubic"; 

        // Illegal argument handling
        // -------------------------------
            // Check trajectory period
            if (!validateTrajectoryPeriod(t, p_f, func_prefix, &trajectory))
            {
                // Function return
                return trajectory;
            }

            // Identical start- and end-point
            if(p_s == p_f)
            {
                // Assign values to trajectory
                trajectory = std::vector<double>(t.size(), p_s);     // Fill with start-point values
                pos_traj = Eigen::VectorXd::Ones(t.size()) * p_f;   // Fill with start-point values
                vel_traj = Eigen::VectorXd::Zero(t.size());         // Fill with zeros
                
                // Function return
                return trajectory;
            }

        // Calculation
        // -------------------------------
            // Get final time 
            // (equal to last element of time-vector)
            double tf = t.back();

            // Define the polynomial as a system of linear equation expressed in matrix form (Ax = b)
            // Then use x = inv(A) * b to solve for the polynomial coefficients

            // Matrix-equation
            Eigen::MatrixXd m(4,4);
            m << 0,                 0,                  0,                  1,      // q-start equation (position)
                 pow(tf, 3),        pow(tf, 2),         tf,                 1,      // q-final quation (position)
                 0,                 0,                  1,                  0,      // qd-start equation (velocity)
                 3 * pow(tf, 2),    2 * tf,             1,                  0;      // qd-final equation (velocity)
            
            // Initial- and final-values as equation solutions
            Eigen::VectorXd q(4);
            q << p_s, p_f, v_s, v_f;

            // Calculate Coefficients for position
            // (using x = inv(A) * b )
            Eigen::VectorXd c(4);       
            c = m.inverse() * q;    
            
            // Calculate Coefficients for velocity
            // (Multiplying derivative values with position-coefficients)  
            Eigen::VectorXd c_d(3);     
            c_d << (3 * c[0]),
                   (2 * c[1]),
                   (1 * c[2]);
                    
            // Evaluate polynomials
            // (converting time-vector from std::Vector<> to Eigen::VectorX)
            pos_traj = Math::polyval(c, Convert::vectorStdToEigen(t));
            vel_traj = Math::polyval(c_d, Convert::vectorStdToEigen(t));

        // Resize polynomial trajectory to equal time-period sizeQuintic
        trajectory.resize(t.size());

        // Convert Trajectory Eigen::VectorXd to std::vector<double>
        trajectory = Convert::vectorEigenToStd(pos_traj);

        // Function return
        return trajectory;
    }


    // Cubic Polynomial Trajectory
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Trajectory::polyCubic(
        double p_s, 
        double p_f, 
        int n,
        double v_s, 
        double v_f)
    {
        // Define polynomial trajectory
        std::vector<double> trajectory; // Polynomial trajectory (position)
        std::string func_prefix = "polynomialCubic"; 

        // Illegal argument handling
        // -------------------------------
            // Check trajectory period
            if (!validateTrajectoryPeriod(n, p_f, func_prefix, &trajectory))
            {
                // Function return
                return trajectory;
            }

        // Calculation
        // -------------------------------
            // Compute time-vector 
            // (using linspace to get evenly spaced vector with n-points) 
            std::vector<double> t = Math::linspace(0.0, (n-1), n);

            // Calculate Quintic-Polynomial
            trajectory = polyCubic(p_s, p_f, t, v_s, v_f);

        // Function return
        return trajectory;
    }


    // Cubic Polynomial Trajectory
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Trajectory::polyCubic(
        double p_s, 
        double p_f, 
        double dt,
        double v_s, 
        double v_f)
    {
        // Define trajectory and lcoal variables
        std::vector<double> trajectory;
        
        // Calculate number of steps
        const double distance = std::abs(p_f - p_s);
        int n = std::floor(distance / dt) + 1;

        // Calculate Cubic-Polynomial
        trajectory = polyCubic(p_s, p_f, n, v_s, v_f);

        // Function return
        return trajectory;
    }


    // Generate Linear Trajectory
    // -------------------------------
    std::vector<Eigen::Isometry3d> Trajectory::trajectoryLinear(
        const Eigen::Isometry3d &pose_start,
        const Eigen::Isometry3d &pose_end,
        const int &steps)
    {
        // Define trajectory and lcal variables
        std::vector<Eigen::Isometry3d> trajectory;      // Trajectory of Transformations Matrices
        Eigen::Isometry3d transformation;
        std::string func_prefix = "LinearTrajectory: "; 

        // Initialization
        // -------------------------------
            // Extract the translational part of the transformations
            Eigen::Vector3d p_start(pose_start.translation());
            Eigen::Vector3d p_end(pose_end.translation());

            // Extract the orientational part of the transformations
            Eigen::Quaterniond q_start(pose_start.rotation());
            Eigen::Quaterniond q_end(pose_end.rotation());

        // Calculation
        // -------------------------------
            // Compute translation trajectory using linear interpolation
            std::vector<Eigen::Vector3d> translations = Math::lerp(p_start, p_end, steps);

            // Compute orientation trajectory using spherical linear interpolation
            std::vector<Eigen::Quaterniond> orientations = Math::slerp(q_start, q_end, steps);

            // Iterate over the number of points
            for (int i = 0; i < steps; i++)
            {
                // Compute transformation for each step
                transformation.translation() = translations[i];
                transformation.linear() = orientations[i].matrix();
                 
                // Append transformation to trajectory
                trajectory.push_back(transformation);
            }

        // Function return
        return trajectory;
    }


    // Generate Linear Trajectory
    // -------------------------------
    std::vector<Eigen::Isometry3d> Trajectory::trajectoryLinear(
        const Eigen::Isometry3d &pose_start,
        const Eigen::Isometry3d &pose_end,
        const double &step_inc)
    {
        // Define trajectory and lcal variables
        std::vector<Eigen::Isometry3d> trajectory;      // Trajectory of Transformations Matrices
        Eigen::Isometry3d transformation;
        
        // Initialization
        // -------------------------------
            // Extract the translational part of the transformations
            Eigen::Vector3d p_start(pose_start.translation());
            Eigen::Vector3d p_end(pose_end.translation());

            // Extract the orientational part of the transformations
            Eigen::Quaterniond q_start(pose_start.rotation());
            Eigen::Quaterniond q_end(pose_end.rotation());

            // Calculate number of steps
            const Eigen::Vector3d distance = p_end - p_start;
            int p_n = std::floor(distance.norm() / step_inc) + 1;
            
            const Eigen::Quaternion<double> difference = q_end * q_start.inverse();
            int q_n = std::floor(difference.norm() / step_inc) + 1;

            // Determine the largest number of steps
            int steps = std::max(p_n, q_n);

        // Calculation
        // -------------------------------
            // Compute translation trajectory using linear interpolation
            std::vector<Eigen::Vector3d> translations = Math::lerp(p_start, p_end, steps);

            // Compute orientation trajectory using spherical linear interpolation
            std::vector<Eigen::Quaterniond> orientations = Math::slerp(q_start, q_end, steps);

            // Iterate over the number of points
            for (int i = 0; i < steps; i++)
            {
                // Compute transformation for each step
                transformation.translation() = translations[i];
                transformation.linear() = orientations[i].matrix();
                 
                // Append transformation to trajectory
                trajectory.push_back(transformation);
            }

        // Function return
        return trajectory;
    }


    // Generate Circular Trajectory
    // -------------------------------
    std::vector<Eigen::Isometry3d> Trajectory::trajectoryCircular(
        Eigen::Vector3d center,
        double radius,
        double angle,
        int steps)
    {
        // Local variables
        Eigen::Isometry3d tm;                   // Transformation Matrix
        std::vector<Eigen::Isometry3d> traj;    // Trajectory Transformations Matrices
        std::vector<double> thetas;             // Circle angle
        std::vector<double> phis;               // Free Axis rotation angle

        // Initialize
        thetas = Math::linspace(0.0, 2*M_PI, steps);    // Circular steps
        phis = Math::linspace(0.0, 2*M_PI, steps);      // Free Axis rotation
        traj.reserve(steps);                            // Reserve transformation matrices
        tm = Eigen::Isometry3d::Identity();             // Initialize transformation as identity matrix
        
        // Iterate over each circle step
        for (size_t i = 0; i < steps; i++)
        {
            // Current circle step and free axis rotation angle
            double theta = thetas[i];
            double phi = phis[i];
            
            // Translation
            Eigen::Vector3d pos_vec;
            pos_vec(0) = center(0) + radius * cos(theta);   // X-Position
            pos_vec(1) = center(1) + radius * sin(theta);   // Y-Position
            pos_vec(2) = center(2);                         // Z-Position

            Eigen::Vector3d rot_vec;
            rot_vec(0) = theta;     // Z1-Rotation
            rot_vec(1) = angle;     // Y-Rotation
            rot_vec(2) = phi;       // Z2-Position

            // Rotation Matrix
            Eigen::Matrix3d rot_mat = Math::rotMatZYZ(rot_vec);

            // Create Transformation Matrix
            tm.translation() = pos_vec; // Translation
            tm.linear() = rot_mat;      // Rotation

            // Append current transformation matrix
            traj.push_back(tm);
        }

        // Function return
        return traj;
    }


    // Validate Trajectory Period
    // -------------------------------
    // (Function Overloading)
    bool Trajectory::validateTrajectoryPeriod(
        const std::vector<double> &t,
        const double &p_f,
        const std::string &func_prefix,
        std::vector<double> *ptr_trajectory)
    {
        // Assert trajectory pointer
        ROS_ASSERT(ptr_trajectory);

        // Check for empty time period
        if (t.empty())
        {
            // Assign zero-trajectory
            ptr_trajectory->push_back(0);

            // Report to terminal
            ROS_ERROR_STREAM(Trajectory::CLASS_PREFIX + func_prefix 
                << ": Failed! Time-Period is empty, returning empty trajectory");

            // Function return
            return false;
        }
        // Last element of time period is zero
        else if (t.back() == 0.0)
        {
            // Assign zero-trajectory
            ptr_trajectory->push_back(0);

            // Report to terminal
            ROS_ERROR_STREAM(Trajectory::CLASS_PREFIX + func_prefix 
                << ": Failed! Time-Period's last elemen equals zero, returning empty trajectory");

            // Function return
            return false;
        }

        // Time period only contains one step
        else if (t.size() == 1)
        {
            // Assign trajectory with end-point
            ptr_trajectory->push_back(p_f);

            // Report to terminal
            ROS_ERROR_STREAM(Trajectory::CLASS_PREFIX + func_prefix 
                << ": Failed! Time-Period's size equals one, returning trajectory with only end-point");

            // Function return
            return false;
        }

        // Function return
        return true;
    }


    // Validate Trajectory Period
    // -------------------------------
    // (Function Overloading)
    bool Trajectory::validateTrajectoryPeriod(
        const int &n,
        const double &p_f,
        const std::string &func_prefix,
        std::vector<double> *ptr_trajectory)
    {
        // Assert trajectory pointer
        ROS_ASSERT(ptr_trajectory);

        // Check for zero number of steps
        if (n == 0)
        {
            // Assign zero-trajectory
            ptr_trajectory->push_back(0);

            // Report to terminal
            ROS_ERROR_STREAM(Trajectory::CLASS_PREFIX + func_prefix
                << ": Failed! Number-of-steps is empty, returning empty/zero trajectory");

            // Function return
            return false;
        }

        // Only one step
        else if (n == 1)
        {
            // Assign trajectory with end-point
            ptr_trajectory->push_back(p_f);

            // Report to terminal
            ROS_ERROR_STREAM(Trajectory::CLASS_PREFIX + func_prefix 
                << ": Failed! Number-of-steps equals one, returning trajectory with only end-point");

            // Function return
            return false;
        }

        // Function return
        return true;
    }
    
} // End Namespace: Robotics Toolbox