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

// Include guard:
// -------------------------------
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
#ifndef TRAJECTORY_TOOL_H       
#define TRAJECTORY_TOOL_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <vector>

    // Ros
    #include <ros/ros.h>

    // RVIZ Visualization
    #include <visualization_msgs/MarkerArray.h>
    #include <std_msgs/ColorRGBA.h>

    // Geometry
    #include "geometry_msgs/PoseStamped.h"

    // TF2
    #include <tf2_eigen/tf2_eigen.h>

    // Eigen
    // #include <Eigen/Core>
    #include <Eigen/Geometry>
    #include <eigen_conversions/eigen_msg.h>

    // Robotics Toolbox
    #include "robot_toolbox/tools/common.h"
    #include "robot_toolbox/tools/convert.h"
    #include "robot_toolbox/tools/math.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Trajectory Tool Class
// -------------------------------
class Trajectory
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:

        // Linear Segment with Parabolic Blends 
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Linear Segment with Parabolic Blends trajectory
        * also known as a Trapozodial Trajectory
        * This type of trajectory has a trapezodial velocity profile,
        * which is appropriate when constant velocity is desired along 
        * a portion of the path, resulting in a parabolic position profile.
        * Trajectory starts at p_s and ends at p_f over a time period t
        * The trajectory consists of 3 parts:
        * ts -> tb: Linear ramped velocity, giving a quadratic polynomial motion
        * tb:       Blending time, with a constant velocity giving a linear motion
        * tb -> tf: Linear ramped down velocity, giving a quadratic polynomial motion
        * \param p_s    Trajectory start point [double]
        * \param p_f    Trajectory finish point [double]
        * \param t      Trajectory time vector [std::vector<double>]
        * \return       Linear Segment with Parabolic Blends trajectory [std::vector<double>]
        */
        static std::vector<double> lspb(
            const double &p_s, 
            const double &p_f, 
            const std::vector<double> &t);


        // Linear Segment with Parabolic Blends 
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Linear Segment with Parabolic Blends trajectory
        * also known as a Trapozodial Trajectory
        * This type of trajectory has a trapezodial velocity profile,
        * which is appropriate when constant velocity is desired along 
        * a portion of the path, resulting in a parabolic position profile.
        * Trajectory starts at p_s and ends at p_f with a total of n number points
        * The trajectory consists of 3 parts:
        * ts -> tb: Linear ramped velocity, giving a quadratic polynomial motion
        * tb:       Blending time, with a constant velocity giving a linear motion
        * tb -> tf: Linear ramped down velocity, giving a quadratic polynomial motion
        * \param p_s    Trajectory start point [double]
        * \param p_f    Trajectory finish point [double]
        * \param n      Trajectory total number of steps [int]
        * \return       Linear Segment with Parabolic Blends trajectory [std::vector<double>]
        */
        static std::vector<double> lspb(
            const double &p_s, 
            const double &p_f, 
            const int &n); 


        // Linear Segment with Parabolic Blends 
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Linear Segment with Parabolic Blends trajectory
        * also known as a Trapozodial Trajectory
        * This type of trajectory has a trapezodial velocity profile,
        * which is appropriate when constant velocity is desired along 
        * a portion of the path, resulting in a parabolic position profile.
        * Trajectory starts at p_s and ends at p_f with a total of n number points
        * The trajectory consists of 3 parts:
        * ts -> tb: Linear ramped velocity, giving a quadratic polynomial motion
        * tb:       Blending time, with a constant velocity giving a linear motion
        * tb -> tf: Linear ramped down velocity, giving a quadratic polynomial motion
        * \param p_s    Trajectory start point [double]
        * \param p_f    Trajectory finish point [double]
        * \param dt     Trajectory step increment [double]
        * \return       Linear Segment with Parabolic Blends trajectory [std::vector<double>]
        */
        static std::vector<double> lspb(
            const double &p_s, 
            const double &p_f, 
            const double &dt);


        // Linear Segment with Parabolic Blends 
        // -------------------------------
        // (Function Overloading)
        /** \brief Compute a Linear Segment with Parabolic Blends trajectory
        * also known as a Trapozodial Trajectory
        * This type of trajectory has a trapezodial velocity profile,
        * which is appropriate when constant velocity is desired along 
        * a portion of the path, resulting in a parabolic position profile.
        * Trajectory starts at p_s and ends at p_f over a time period t
        * The trajectory consists of 3 parts:
        * ts -> tb: Linear ramped velocity, giving a quadratic polynomial motion
        * tb:       Blending time, with a constant velocity giving a linear motion
        * tb -> tf: Linear ramped down velocity, giving a quadratic polynomial motion
        * \param p_s    Trajectory start point [Eigen::VectorXd]
        * \param p_f    Trajectory finish point [Eigen::VectorXd]
        * \param t      Trajectory time vector [std::vector<double>]
        * \return       Linear Segment with Parabolic Blends trajectory [std::vector<Eigen::VectorXd>]
        */
        template<int Dim>
        static std::vector<Eigen::Matrix<double, Dim, 1>> lspb(
            const Eigen::Matrix<double, Dim, 1> &p_s, 
            const Eigen::Matrix<double, Dim, 1> &p_f, 
            const std::vector<double> &t)
        {
            // Define trajectory and local variables
            std::vector<Eigen::Matrix<double, Dim, 1>> trajectory;
            Eigen::Matrix<double, Dim, 1> points;
            int dim = p_s.size();
            std::string func_prefix = "lspb: ";

            // Illegal argument handling
            // -------------------------------
                // Check trajectory start- and end-Point
                if (!validateTrajectoryPoints(p_s, p_f, func_prefix))
                {
                    // Function return
                    return trajectory;
                }

                // Check trajectory period
                if (!validateTrajectoryPeriod(t, p_f, func_prefix, &trajectory))
                {
                    // Function return
                    return trajectory;
                }

            // Calculation
            // -------------------------------
                // Resize Points-Vector to equal the length of start- and end-points
                points.resize(dim);

                // Iterate over the time-series vector
                for (int time = 0; time < t.size(); time++)
                {
                    // Iterate over the dimensions of points-vector
                    for (int d = 0; d < dim; d++)
                    {
                        // Compute trajectory for the current dimension element
                        std::vector<double> point_trajectory = lspb(p_s[d], p_f[d], t);

                        // Assign the trajectory point at the current time for the current dimension
                        points(d) = point_trajectory[time];
                    }

                    // Append the points-vector at the current time to the trajectory
                    trajectory.push_back(points);
                }
            
            // Function return
            return trajectory;
        }


        // Linear Segment with Parabolic Blends 
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Linear Segment with Parabolic Blends trajectory
        * also known as a Trapozodial Trajectory
        * This type of trajectory has a trapezodial velocity profile,
        * which is appropriate when constant velocity is desired along 
        * a portion of the path, resulting in a parabolic position profile.
        * Trajectory starts at p_s and ends at p_f with a total of n number points
        * The trajectory consists of 3 parts:
        * ts -> tb: Linear ramped velocity, giving a quadratic polynomial motion
        * tb:       Blending time, with a constant velocity giving a linear motion
        * tb -> tf: Linear ramped down velocity, giving a quadratic polynomial motion
        * \param p_s    Trajectory start point [Eigen::VectorXd]
        * \param p_f    Trajectory finish point [Eigen::VectorXd]
        * \param n      Trajectory total number of steps [int]
        * \return       Linear Segment with Parabolic Blends trajectory [std::vector<Eigen::Vector3d>]
        */
        template<int Dim>
        static std::vector<Eigen::Matrix<double, Dim, 1>> lspb(
            const Eigen::Matrix<double, Dim, 1> &p_s, 
            const Eigen::Matrix<double, Dim, 1> &p_f, 
            const int &n)
        {
            // Define trajectory and local variables
            std::vector<Eigen::Matrix<double, Dim, 1>> trajectory;
            std::string func_prefix = "lspb: ";

            // Illegal argument handling
            // -------------------------------
                // Check trajectory start- and end-Point
                if (!validateTrajectoryPoints(p_s, p_f, func_prefix))
                {
                    // Function return
                    return trajectory;
                }

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

                // Compute Trajectory
                trajectory = lspb(p_s, p_f, t);

            // Function return
            return trajectory;
        }


        // Linear Segment with Parabolic Blends 
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Linear Segment with Parabolic Blends trajectory
        * also known as a Trapozodial Trajectory
        * This type of trajectory has a trapezodial velocity profile,
        * which is appropriate when constant velocity is desired along 
        * a portion of the path, resulting in a parabolic position profile.
        * Trajectory starts at p_s and ends at p_f with a total of n number points
        * The trajectory consists of 3 parts:
        * ts -> tb: Linear ramped velocity, giving a quadratic polynomial motion
        * tb:       Blending time, with a constant velocity giving a linear motion
        * tb -> tf: Linear ramped down velocity, giving a quadratic polynomial motion
        * \param p_s    Trajectory start point [Eigen::VectorXd]
        * \param p_f    Trajectory finish point [Eigen::VectorXd]
        * \param dt     Trajectory step increment [double]
        * \return       Linear Segment with Parabolic Blends trajectory [std::vector<Eigen::Vector3d>]
        */
        template<int Dim>
        static std::vector<Eigen::Matrix<double, Dim, 1>> lspb(
            const Eigen::Matrix<double, Dim, 1> &p_s, 
            const Eigen::Matrix<double, Dim, 1> &p_f, 
            const double &dt)
        {
            // Define trajectory and local variables
            std::vector<Eigen::Matrix<double, Dim, 1>> trajectory;

            // Calculate number of steps
            const Eigen::Matrix<double, Dim, 1> distance = p_f - p_s;
            int n = std::floor(distance.norm() / dt) + 1;

            // Calculate Linear Segment with Parabolic Blends
            trajectory = lspb(p_s, p_f, n);

            // Function return
            return trajectory;
        }


        // Quintic Polynomial Trajectory
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Quintic Polynomial trajectory (5th order polynomial)
        * Trajectory varies smoothly from start-point p_s and to end-point at p_f 
        * over a time period t.
        * As an option it is possible to specify the initial and final velocity of the trajectory
        * (where these values defaults to zero)
        * \param p_s    Trajectory start point [double]
        * \param p_f    Trajectory finish point [double]
        * \param t      Trajectory time vector [std::vector<double>]
        * \param v_s    Trajectory initial velocity (default = 0) [double]
        * \param v_f    Trajectory final velocity (default = 0) [double]
        * \return       Quintic Polynomial trajectory [std::vector<double>]
        */
        static std::vector<double> polyQuintic(
            double p_s, 
            double p_f, 
            std::vector<double> t,
            double v_s = 0, 
            double v_f = 0); 


        // Quintic Polynomial Trajectory
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Quintic Polynomial trajectory (5th order polynomial)
        * Trajectory varies smoothly from start-point p_s and to end-point at p_f 
        * with a total of n number points.
        * As an option it is possible to specify the initial and final velocity of the trajectory
        * (where these values defaults to zero)
        * \param p_s    Trajectory start point [double]
        * \param p_f    Trajectory finish point [double]
        * \param n      Trajectory total number of steps [int]
        * \param v_s    Trajectory initial velocity (default = 0) [double]
        * \param v_f    Trajectory final velocity (default = 0) [double]
        * \return       Quintic Polynomial trajectory [std::vector<double>]
        */
        static std::vector<double> polyQuintic(
            double p_s, 
            double p_f, 
            int n,
            double v_s = 0, 
            double v_f = 0); 


        // Quintic Polynomial Trajectory
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Quintic Polynomial trajectory (5th order polynomial)
        * Trajectory varies smoothly from start-point p_s and to end-point at p_f 
        * with a total of n number points.
        * As an option it is possible to specify the initial and final velocity of the trajectory
        * (where these values defaults to zero)
        * \param p_s    Trajectory start point [double]
        * \param p_f    Trajectory finish point [double]
        * \param dt     Trajectory step increment [double]
        * \param v_s    Trajectory initial velocity (default = 0) [double]
        * \param v_f    Trajectory final velocity (default = 0) [double]
        * \return       Quintic Polynomial trajectory [std::vector<double>]
        */
        static std::vector<double> polyQuintic(
            double p_s, 
            double p_f, 
            double dt,
            double v_s = 0, 
            double v_f = 0); 


        // Quintic Polynomial Trajectory
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Quintic Polynomial trajectory (5th order polynomial)
        * Trajectory varies smoothly from start-point p_s and to end-point at p_f 
        * over a time period t.
        * As an option it is possible to specify the initial and final velocity of the trajectory
        * (where these values defaults to zero)
        * \param p_s    Trajectory start point [Eigen::VectorXd]
        * \param p_f    Trajectory finish point [Eigen::VectorXd]
        * \param t      Trajectory time vector [std::vector<double>]
        * \param v_s    Trajectory initial velocity (default = 0) [Eigen::VectorXd]
        * \param v_f    Trajectory final velocity (default = 0) [Eigen::VectorXd]
        * \return       Quintic Polynomial trajectory [std::vector<Eigen::VectorXd>]
        */
        template<int Dim>
        static std::vector<Eigen::Matrix<double, Dim, 1>> polyQuintic(
            const Eigen::Matrix<double, Dim, 1> &p_s, 
            const Eigen::Matrix<double, Dim, 1> &p_f, 
            const std::vector<double> &t,
            Eigen::Matrix<double, Dim, 1> v_s = Eigen::Matrix<double, Dim, 1>::Zero(0), 
            Eigen::Matrix<double, Dim, 1> v_f = Eigen::Matrix<double, Dim, 1>::Zero(0))
        {
            // Define trajectory and local variables
            std::vector<Eigen::Matrix<double, Dim, 1>> trajectory;
            Eigen::Matrix<double, Dim, 1> points;
            int dim = p_s.size();
            std::string func_prefix = "polynomialQuintic: ";

            // Illegal argument handling
            // -------------------------------
                // Check trajectory start- and end-point
                if (!validateTrajectoryPoints(p_s, p_f, func_prefix))
                {
                    // Function return
                    return trajectory;
                }

                // Check trajectory period
                if (!validateTrajectoryPeriod(t, p_f, func_prefix, &trajectory))
                {
                    // Function return
                    return trajectory;
                }

                // Check trajectory velocity-start- and finish-point
                if (!validateTrajectoryVelocityPoints(v_s, v_f, func_prefix, dim))
                {
                    // Function return
                    return trajectory;
                }

            // Calculation
            // -------------------------------
                // Resize Points-Vector to equal the length of start- and end-points
                points.resize(dim);

                // Iterate over the time-series vector
                for (int time = 0; time < t.size(); time++)
                {
                    // Iterate over the dimensions of points-vector
                    for (int d = 0; d < dim; d++)
                    {
                        // Compute trajectory for the current dimension element
                        std::vector<double> point_trajectory = polyQuintic(p_s[d], p_f[d], t, v_s[d], v_f[d]);

                        // Assign the trajectory point at the current time for the current dimension
                        points(d) = point_trajectory[time];
                    }

                    // Append the points-vector at the current time to the trajectory
                    trajectory.push_back(points);
                }
            
            // Function return
            return trajectory;
        }


        // Quintic Polynomial Trajectory
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Quintic Polynomial trajectory (5th order polynomial)
        * Trajectory varies smoothly from start-point p_s and to end-point at p_f 
        * with a total of n number points.
        * As an option it is possible to specify the initial and final velocity of the trajectory
        * (where these values defaults to zero)
        * \param p_s    Trajectory start point [Eigen::VectorXd]
        * \param p_f    Trajectory finish point [Eigen::VectorXd]
        * \param n      Trajectory total number of steps [int]
        * \param v_s    Trajectory initial velocity (default = 0) [Eigen::VectorXd]
        * \param v_f    Trajectory final velocity (default = 0) [Eigen::VectorXd]
        * \return       Quintic Polynomial trajectory [std::vector<Eigen::VectorXd>]
        */
        template<int Dim>
        static std::vector<Eigen::Matrix<double, Dim, 1>> polyQuintic(
            const Eigen::Matrix<double, Dim, 1> &p_s, 
            const Eigen::Matrix<double, Dim, 1> &p_f, 
            const int &n,
            Eigen::Matrix<double, Dim, 1> v_s = Eigen::Matrix<double, Dim, 1>::Zero(0), 
            Eigen::Matrix<double, Dim, 1> v_f = Eigen::Matrix<double, Dim, 1>::Zero(0))
        {
            // Define trajectory and local variables
            std::vector<Eigen::Matrix<double, Dim, 1>> trajectory;
            int dim = p_s.size();
            std::string func_prefix = "polynomialQuintic: ";

            // Illegal argument handling
            // -------------------------------
                // Check trajectory start- and end-Point
                if (!validateTrajectoryPoints(p_s, p_f, func_prefix))
                {
                    // Function return
                    return trajectory;
                }

                // Check trajectory period
                if (!validateTrajectoryPeriod(n, p_f, func_prefix, &trajectory))
                {
                    // Function return
                    return trajectory;
                }

                // Check trajectory velocity-start- and finish-point
                if (!validateTrajectoryVelocityPoints(v_s, v_f, func_prefix, dim))
                {
                    // Function return
                    return trajectory;
                }

            // Calculation
            // -------------------------------
                // Compute time-vector 
                // (using linspace to get evenly spaced vector with n-points) 
                std::vector<double> t = Math::linspace(0.0, (n-1), n);

                // Compute Trajectory
                trajectory = polyQuintic(p_s, p_f, t, v_s, v_f);

            // Function return
            return trajectory;
        }


        // Quintic Polynomial Trajectory
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Quintic Polynomial trajectory (5th order polynomial)
        * Trajectory varies smoothly from start-point p_s and to end-point at p_f 
        * with a total of n number points.
        * As an option it is possible to specify the initial and final velocity of the trajectory
        * (where these values defaults to zero)
        * \param p_s    Trajectory start point [Eigen::VectorXd]
        * \param p_f    Trajectory finish point [Eigen::VectorXd]
        * \param dt     Trajectory step increment [double]
        * \param v_s    Trajectory initial velocity (default = 0) [Eigen::VectorXd]
        * \param v_f    Trajectory final velocity (default = 0) [Eigen::VectorXd]
        * \return       Quintic Polynomial trajectory [std::vector<Eigen::VectorXd>]
        */
        template<int Dim>
        static std::vector<Eigen::Matrix<double, Dim, 1>> polyQuintic(
            const Eigen::Matrix<double, Dim, 1> &p_s, 
            const Eigen::Matrix<double, Dim, 1> &p_f, 
            const double &dt,
            Eigen::Matrix<double, Dim, 1> v_s = Eigen::Matrix<double, Dim, 1>::Zero(0), 
            Eigen::Matrix<double, Dim, 1> v_f = Eigen::Matrix<double, Dim, 1>::Zero(0))
        {
            // Define trajectory and local variables
            std::vector<Eigen::Matrix<double, Dim, 1>> trajectory;

            // Calculate number of steps
            const Eigen::Matrix<double, Dim, 1> distance = p_f - p_s;
            int n = std::floor(distance.norm() / dt) + 1;

            // Calculate Quintic-Polynomial
            trajectory = polyQuintic(p_s, p_f, n, v_s, v_f);

            // Function return
            return trajectory;
        }


        // Cubic Polynomial Trajectory
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Cubic Polynomial trajectory (3rd order polynomial)
        * Trajectory varies smoothly from start-point p_s and to end-point at p_f 
        * over a time period t.
        * As an option it is possible to specify the initial and final velocity of the trajectory
        * (where these values defaults to zero)
        * \param p_s    Trajectory start point [double]
        * \param p_f    Trajectory finish point [double]
        * \param t      Trajectory time vector [std::vector<double>]
        * \param v_s    Trajectory initial velocity (default = 0) [double]
        * \param v_f    Trajectory final velocity (default = 0) [double]
        * \return       Cubic Polynomial trajectory [std::vector<double>]
        */
        static std::vector<double> polyCubic(
            double p_s, 
            double p_f, 
            std::vector<double> t,
            double v_s = 0, 
            double v_f = 0); 


        // Cubic Polynomial Trajectory
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Cubic Polynomial trajectory (3rd order polynomial)
        * Trajectory varies smoothly from start-point p_s and to end-point at p_f 
        * with a total of n number points.
        * As an option it is possible to specify the initial and final velocity of the trajectory
        * (where these values defaults to zero)
        * \param p_s    Trajectory start point [double]
        * \param p_f    Trajectory finish point [double]
        * \param n      Trajectory total number of steps [int]
        * \param v_s    Trajectory initial velocity (default = 0) [double]
        * \param v_f    Trajectory final velocity (default = 0) [double]
        * \return       Quintic Polynomial trajectory [std::vector<double>]
        */
        static std::vector<double> polyCubic(
            double p_s, 
            double p_f, 
            int n,
            double v_s = 0, 
            double v_f = 0); 


        // Cubic Polynomial Trajectory
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Cubic Polynomial trajectory (3rd order polynomial)
        * Trajectory varies smoothly from start-point p_s and to end-point at p_f 
        * with a total of n number points.
        * As an option it is possible to specify the initial and final velocity of the trajectory
        * (where these values defaults to zero)
        * \param p_s    Trajectory start point [double]
        * \param p_f    Trajectory finish point [double]
        * \param dt     Trajectory step increment [double]
        * \param v_s    Trajectory initial velocity (default = 0) [double]
        * \param v_f    Trajectory final velocity (default = 0) [double]
        * \return       Quintic Polynomial trajectory [std::vector<double>]
        */
        static std::vector<double> polyCubic(
            double p_s, 
            double p_f, 
            double dt,
            double v_s = 0, 
            double v_f = 0); 


        // Cubic Polynomial Trajectory
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Cubic Polynomial trajectory (3rd order polynomial)
        * Trajectory varies smoothly from start-point p_s and to end-point at p_f 
        * with a total of n number points.
        * As an option it is possible to specify the initial and final velocity of the trajectory
        * (where these values defaults to zero)
        * \param p_s    Trajectory start point [Eigen::VectorXd]
        * \param p_f    Trajectory finish point [Eigen::VectorXd]
        * \param t      Trajectory time vector [std::vector<double>]
        * \param v_s    Trajectory initial velocity (default = 0) [Eigen::VectorXd]
        * \param v_f    Trajectory final velocity (default = 0) [Eigen::VectorXd]
        * \return       Quintic Polynomial trajectory [std::vector<Eigen::VectorXd>]
        */
        template<int Dim>
        static std::vector<Eigen::Matrix<double, Dim, 1>> polyCubic(
            const Eigen::Matrix<double, Dim, 1> &p_s, 
            const Eigen::Matrix<double, Dim, 1> &p_f, 
            const std::vector<double> &t,
            Eigen::Matrix<double, Dim, 1> v_s = Eigen::Matrix<double, Dim, 1>::Zero(0), 
            Eigen::Matrix<double, Dim, 1> v_f = Eigen::Matrix<double, Dim, 1>::Zero(0))
        {
            // Define trajectory and local variables
            std::vector<Eigen::Matrix<double, Dim, 1>> trajectory;
            Eigen::Matrix<double, Dim, 1> points;
            int dim = p_s.size();
            std::string func_prefix = "polynomialCubic: ";

            // Illegal argument handling
            // -------------------------------
                // Check trajectory start- and end-point
                if (!validateTrajectoryPoints(p_s, p_f, func_prefix))
                {
                    // Function return
                    return trajectory;
                }

                // Check trajectory period
                if (!validateTrajectoryPeriod(t, p_f, func_prefix, &trajectory))
                {
                    // Function return
                    return trajectory;
                }

                // Check trajectory velocity-start- and finish-point
                if (!validateTrajectoryVelocityPoints(v_s, v_f, func_prefix, dim))
                {
                    // Function return
                    return trajectory;
                }

            // Calculation
            // -------------------------------
                // Resize Points-Vector to equal the length of start- and end-points
                points.resize(dim);

                // Iterate over the time-series vector
                for (int time = 0; time < t.size(); time++)
                {
                    // Iterate over the dimensions of points-vector
                    for (int d = 0; d < dim; d++)
                    {
                        // Compute trajectory for the current dimension element
                        std::vector<double> point_trajectory = polyCubic(p_s[d], p_f[d], t, v_s[d], v_f[d]);

                        // Assign the trajectory point at the current time for the current dimension
                        points(d) = point_trajectory[time];
                    }

                    // Append the points-vector at the current time to the trajectory
                    trajectory.push_back(points);
                }
            
            // Function return
            return trajectory;
        }


        // Cubic Polynomial Trajectory
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Cubic Polynomial trajectory (3rd order polynomial)
        * Trajectory varies smoothly from start-point p_s and to end-point at p_f 
        * with a total of n number points.
        * As an option it is possible to specify the initial and final velocity of the trajectory
        * (where these values defaults to zero)
        * \param p_s    Trajectory start point [Eigen::VectorXd]
        * \param p_f    Trajectory finish point [Eigen::VectorXd]
        * \param n      Trajectory total number of steps [int]
        * \param v_s    Trajectory initial velocity (default = 0) [Eigen::VectorXd]
        * \param v_f    Trajectory final velocity (default = 0) [Eigen::VectorXd]
        * \return       Quintic Polynomial trajectory [std::vector<Eigen::VectorXd>]
        */
        template<int Dim>
        static std::vector<Eigen::Matrix<double, Dim, 1>> polyCubic(
            const Eigen::Matrix<double, Dim, 1> &p_s, 
            const Eigen::Matrix<double, Dim, 1> &p_f, 
            const int &n,
            Eigen::Matrix<double, Dim, 1> v_s = Eigen::Matrix<double, Dim, 1>::Zero(0), 
            Eigen::Matrix<double, Dim, 1> v_f = Eigen::Matrix<double, Dim, 1>::Zero(0))
        {
            // Define trajectory and local variables
            std::vector<Eigen::Matrix<double, Dim, 1>> trajectory;
            int dim = p_s.size();
            std::string func_prefix = "polynomialCubic: ";

            // Illegal argument handling
            // -------------------------------
                // Check trajectory start- and end-Point
                if (!validateTrajectoryPoints(p_s, p_f, func_prefix))
                {
                    // Function return
                    return trajectory;
                }

                // Check trajectory period
                if (!validateTrajectoryPeriod(n, p_f, func_prefix, &trajectory))
                {
                    // Function return
                    return trajectory;
                }

                // Check trajectory velocity-start- and finish-point
                if (!validateTrajectoryVelocityPoints(v_s, v_f, func_prefix, dim))
                {
                    // Function return
                    return trajectory;
                }

            // Calculation
            // -------------------------------
                // Compute time-vector 
                // (using linspace to get evenly spaced vector with n-points) 
                std::vector<double> t = Math::linspace(0.0, (n-1), n);

                // Compute Trajectory
                trajectory = polyCubic(p_s, p_f, t, v_s, v_f);

            // Function return
            return trajectory;
        }


        // Cubic Polynomial Trajectory
        // -------------------------------
        // (Function Overloading)
        /** \brief Generate a Cubic Polynomial trajectory (3rd order polynomial)
        * Trajectory varies smoothly from start-point p_s and to end-point at p_f 
        * with a total of n number points.
        * As an option it is possible to specify the initial and final velocity of the trajectory
        * (where these values defaults to zero)
        * \param p_s    Trajectory start point [Eigen::VectorXd]
        * \param p_f    Trajectory finish point [Eigen::VectorXd]
        * \param dt     Trajectory step increment [double]
        * \param v_s    Trajectory initial velocity (default = 0) [Eigen::VectorXd]
        * \param v_f    Trajectory final velocity (default = 0) [Eigen::VectorXd]
        * \return       Quintic Polynomial trajectory [std::vector<Eigen::VectorXd>]
        */
        template<int Dim>
        static std::vector<Eigen::Matrix<double, Dim, 1>> polyCubic(
            const Eigen::Matrix<double, Dim, 1> &p_s, 
            const Eigen::Matrix<double, Dim, 1> &p_f, 
            const double &dt,
            Eigen::Matrix<double, Dim, 1> v_s = Eigen::Matrix<double, Dim, 1>::Zero(0), 
            Eigen::Matrix<double, Dim, 1> v_f = Eigen::Matrix<double, Dim, 1>::Zero(0))
        {
            // Define trajectory and local variables
            std::vector<Eigen::Matrix<double, Dim, 1>> trajectory;

            // Calculate number of steps
            const Eigen::Matrix<double, Dim, 1> distance = p_f - p_s;
            int n = std::floor(distance.norm() / dt) + 1;

            // Calculate Cubic Polynomial
            trajectory = polyCubic(p_s, p_f, n);

            // Function return
            return trajectory;
        }


        // Generate Linear Trajectory
        // -------------------------------
        /** \brief Generate Linear Trajectory
        * \param pose_start Start-Pose [Eigen::Isometry3d]
        * \param pose_end End-Pose [Eigen::Isometry3d]
        * \param steps Resolution number of steps for trajectory [int]
        * \return Trajectory [std::vector<Eigen::Isometry3d>]
        */
        static std::vector<Eigen::Isometry3d> trajectoryLinear(
            const Eigen::Isometry3d &pose_start,
            const Eigen::Isometry3d &pose_end,
            const int &steps);


        // Generate Linear Trajectory
        // -------------------------------
        /** \brief Generate Linear Trajectory
        * \param pose_start Start-Pose [Eigen::Isometry3d]
        * \param pose_end End-Pose [Eigen::Isometry3d]
        * \param step_inc Step increments for trajectory [double]
        * \return Trajectory [std::vector<Eigen::Isometry3d>]
        */
        static std::vector<Eigen::Isometry3d> trajectoryLinear(
            const Eigen::Isometry3d &pose_start,
            const Eigen::Isometry3d &pose_end,
            const double &step_inc);


        // Generate Circular Trajectory
        // -------------------------------
        /** \brief Generate Circular Trajectory
        * \param center Circle center point [Eigen::Vector3d]
        * \param radius Circle radius [double]
        * \param angle Normal vector angle (rad) [double]
        * \param steps Resolution number of steps for trajectory [int]
        * \return Trajectory [std::vector<Eigen::Vector3d>]
        */
        static std::vector<Eigen::Isometry3d> trajectoryCircular(
            Eigen::Vector3d center,
            double radius,
            double angle,
            int steps);


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


        // Validate Trajectory Period
        // -------------------------------
        // (Function Overloading)
        /** \brief Validate that the time-vector has legal values, 
        * Time-vector has governing values for calculation of the trajectory period 
        * \param t              Trajectory time vector [std::vector<double>]
        * \param p_f            Trajectory finish point [double]
        * \param func_prefix    Function prefix [std::string]
        * \param ptr_trajectory Trajectory pointer [std::vector<double>]
        * \return               Validation result (true/false) [bool]
        */ 
        static bool validateTrajectoryPeriod(
            const std::vector<double> &t,
            const double &p_f,
            const std::string &func_prefix,
            std::vector<double> *ptr_trajectory);


        // Validate Trajectory Period
        // -------------------------------
        // (Function Overloading)
        /** \brief Validate that the time-vector has legal values, 
        * Time-vector has governing values for calculation of the trajectory period 
        * \param t              Trajectory time vector [std::vector<double>]
        * \param p_f            Trajectory finish point [Eigen::VectorXd]
        * \param func_prefix    Function prefix [std::string]
        * \param ptr_trajectory Trajectory pointer [std::vector<Eigen::VectorXd>]
        * \return               Validation result (true/false) [bool]
        */ 
        template<int Dim>
        static bool validateTrajectoryPeriod(
            const std::vector<double> &t,
            const Eigen::Matrix<double, Dim, 1> &p_f,
            const std::string &func_prefix,
            std::vector<Eigen::Matrix<double, Dim, 1>> *ptr_trajectory)
        {
            // Assert trajectory pointer
            ROS_ASSERT(ptr_trajectory);

            // Check for empty time period
            if (t.empty())
            {
                // Assign zero-trajectory
                ptr_trajectory->push_back(Eigen::Matrix<double, Dim, 1>::Zero(Dim));

                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX + func_prefix 
                    << ": Time-Period is empty, returning empty trajectory");

                // Function return
                return false;
            }
            // Last element of time period is zero
            else if (t.back() == 0.0)
            {
                // Assign zero-trajectory
                ptr_trajectory->push_back(Eigen::Matrix<double, Dim, 1>::Zero(Dim));

                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX + func_prefix 
                    << ": Time-Period's last elemen equals zero, returning empty trajectory");

                // Function return
                return false;
            }

            // Time period only contains one step
            else if (t.size() == 1)
            {
                // Assign trajectory with end-point
                ptr_trajectory->push_back(p_f);

                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX + func_prefix 
                    << ": Time-Period's size equals one, returning trajectory with only end-point");

                // Function return
                return false;
            }

            // Function return
            return true;
        }


        // Validate Trajectory Period
        // -------------------------------
        // (Function Overloading)
        /** \brief Validate that number of steps has legal value, 
        * Number of steps is a governing value for calculation of the trajectory period
        * \param n              Trajectory total number of steps [int]
        * \param p_f            Trajectory finish point [double]
        * \param func_prefix    Function prefix [std::string]
        * \param ptr_trajectory Trajectory pointer [std::vector<double>]
        * \return               Validation result (true/false) [bool]
        */ 
        static bool validateTrajectoryPeriod(
            const int &n,
            const double &p_f,
            const std::string &func_prefix,
            std::vector<double> *ptr_trajectory);


        // Validate Trajectory Period
        // -------------------------------
        // (Function Overloading)
        /** \brief Validate that number of steps has legal value, 
        * Number of steps is a governing value for calculation of the trajectory period 
        * \param n              Trajectory total number of steps [int]
        * \param p_f            Trajectory finish point [Eigen::VectorXd]
        * \param func_prefix    Function prefix [std::string]
        * \param ptr_trajectory Trajectory pointer [std::vector<Eigen::VectorXd>]
        * \return               Validation result (true/false) [bool]
        */ 
        template<int Dim>
        static bool validateTrajectoryPeriod(
            const int &n,
            const Eigen::Matrix<double, Dim, 1> &p_f,
            const std::string &func_prefix,
            std::vector<Eigen::Matrix<double, Dim, 1>> *ptr_trajectory)
        {
            // Assert trajectory pointer
            ROS_ASSERT(ptr_trajectory);

            // Check for zero number of steps
            if (n == 0)
            {
                // Assign zero-trajectory
                ptr_trajectory->push_back(Eigen::Matrix<double, Dim, 1>::Zero(Dim));

                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX + func_prefix
                    << ": Number-of-steps is empty, returning empty/zero trajectory");

                // Function return
                return false;
            }

            // Only one step
            else if (n == 1)
            {
                // Assign trajectory with end-point
                ptr_trajectory->push_back(p_f);

                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX + func_prefix 
                    << ": Number-of-steps equals one, returning trajectory with only end-point");

                // Function return
                return false;
            }

            // Function return
            return true;
        }


        // Validate Trajectory Start- and End-Points
        // -------------------------------
        // (Function Overloading)
        /** \brief Validate Start- and End-Points given as inputs to trajectory calculation
        * \param p_s            Trajectory start point [Eigen::VectorXd]
        * \param p_f            Trajectory finish point [Eigen::VectorXd]
        * \param func_prefix    Function prefix [std::string]
        * \return               Validation result (true/false) [bool]
        */ 
        template<int Dim>
        static bool validateTrajectoryPoints(
            const Eigen::Matrix<double, Dim, 1> &p_s, 
            const Eigen::Matrix<double, Dim, 1> &p_f,
            const std::string &func_prefix)
        {
            // Empty Vector: Start-Points
            if (p_s.size() == 0)
            {
                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX + func_prefix 
                    << ": Start-Points vector is empty");

                // Throw execption
                throw std::invalid_argument(CLASS_PREFIX + func_prefix 
                    + ": Start-Points vector is empty");

                // Function return
                return false;
            }

            // Empty Vector: End-Points
            else if (p_f.size() == 0)
            {
                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX + func_prefix 
                    << ": End-Points vector is empty");

                // Throw execption
                throw std::invalid_argument(CLASS_PREFIX + func_prefix 
                    + ": End-Points vector is empty");

                // Function return
                return false;
            }

            // Points vector size does not match
            else if (p_s.size() != p_f.size())
            {
                // Report to terminal
                ROS_ERROR_STREAM(CLASS_PREFIX + func_prefix 
                    << ": Start- and End-Point has different size");

                // Throw execption
                throw std::invalid_argument(CLASS_PREFIX + func_prefix 
                    + ": Start- and End-Point has different size");

                // Function return
                return false;
            }

            // Function return
            return true;
        }


        // Validate Trajectory Velocity-Initial- and Velocity-End-Points
        // -------------------------------
        // (Function Overloading)
        /** \brief Validate Velocity-Initial- and Velocity-End-Points given as inputs to trajectory calculation
        * \param v_s            Trajectory initial velocity [Eigen::VectorXd]
        * \param v_f            Trajectory finish velocity [Eigen::VectorXd]
        * \param func_prefix    Function prefix [std::string]
        * \param dim            Dimension of trajectory start point [int]
        * \return               Validation result (true/false) [bool]
        */ 
        template<int Dim>
        static bool validateTrajectoryVelocityPoints( 
            Eigen::Matrix<double, Dim, 1> &v_s,
            Eigen::Matrix<double, Dim, 1> &v_f,
            const std::string &func_prefix,
            const int dim)
        {
            // Evaluate Velocity-Initial-Points
            // -------------------------------
                // Default Velocity-Initial-Points
                // (empty vector)
                if (v_s.size() == 0)
                {
                    // Assign all zeros to the vector  
                    // with equal length to the start-point vector
                    v_s.resize(dim); 
                    v_s.setZero();

                }
                // Non-default Velocity-Initial-Points
                // (check size)
                else if(v_s.size() != dim)
                {
                    // Report to terminal
                    ROS_ERROR_STREAM(CLASS_PREFIX + func_prefix 
                        << ": Velocity-Initial-Points and Start-Points has different size");

                    // Throw execption
                    throw std::invalid_argument(CLASS_PREFIX + func_prefix 
                        + ": Velocity-Initial-Points and Start-Point has different size");

                    // Function return
                    return false;
                }

            // Evaluate Velocity-Final-Points
            // -------------------------------
                // Default Velocity-Final-Points
                // (empty vector)
                if (v_f.size() == 0)
                {
                    // Assign all zeros to the vector  
                    // with equal length to the start-point vector
                    // v_f = Eigen::Matrix<double, Dim, 1>::Zero(p_s.size());

                    v_f.resize(dim); 
                    v_f.setZero();
                }
                // Non-default Velocity-Final-Points
                // (check size)
                else if(v_f.size() != dim)
                {
                    // Report to terminal
                    ROS_ERROR_STREAM(CLASS_PREFIX + func_prefix 
                        << ": Velocity-Final-Points and Start-Points has different size");

                    // Throw execption
                    throw std::invalid_argument(CLASS_PREFIX + func_prefix 
                        + ": Velocity-Final-Points and Start-Points has different size");
                    
                    // Function return
                    return false;
                }

            // Function return
            return true;
        }

}; // End Class: Trajectory
} // End Namespace: Robotics Toolbox
#endif // TRAJECTORY_TOOL_H 