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

// Degree
void degreeRad()
{
    // Degree
    double deg = Toolbox::Common::radToDeg(M_PI/2);
    ROS_INFO_STREAM("Degree: " << deg);

    double rad = Toolbox::Common::degToRad(45.0);
    ROS_INFO_STREAM("Radians: " << rad);
}

// Colors
void colors()
{
    // Color
    std_msgs::ColorRGBA test_color = Toolbox::Visual::COLOR_RED;

    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM(" Color Test: ");
    ROS_INFO_STREAM(" Color R: " << test_color.r);
    ROS_INFO_STREAM(" Color G: " << test_color.g);
    ROS_INFO_STREAM(" Color B: " << test_color.b);
    ROS_INFO_STREAM(" Color A: " << test_color.a);

    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM(" Color GREEN: ");
    ROS_INFO_STREAM(" Color R: " << Toolbox::Visual::COLOR_GREEN.r);
    ROS_INFO_STREAM(" Color G: " << Toolbox::Visual::COLOR_GREEN.g);
    ROS_INFO_STREAM(" Color B: " << Toolbox::Visual::COLOR_GREEN.b);
    ROS_INFO_STREAM(" Color A: " << Toolbox::Visual::COLOR_GREEN.a);
    ROS_INFO_STREAM(" ");
}

// Axis type
void axistype()
{
    // Axis
        Toolbox::AxisType a = Toolbox::Common::AXIS_X; 

        ROS_INFO_STREAM(" Axis X: ");
        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" Id: "     << a.id);
        ROS_INFO_STREAM(" Name: "   << a.name);

        ROS_INFO_STREAM(" Axis Z: ");
        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" Id: "     << Toolbox::Common::AXIS_Z.id);
        ROS_INFO_STREAM(" Name: "   << Toolbox::Common::AXIS_Z.name);
        ROS_INFO_STREAM(" ");
}

// Trajectory
void traj()
{
    //  TRAJECTORY
    // -------------------------------
    Eigen::Isometry3d tm;
    std::vector<Eigen::Isometry3d> traj;
    Eigen::Vector3d pos;
    geometry_msgs::Pose pose;
    pos(0) = 3.0;   // X-Position
    pos(1) = 3.0;   // Y-Position
    pos(2) = 1.0;   // Z-Position

    traj = Toolbox::Trajectory::trajectoryCircular(pos,
                                                1.0,
                                                Toolbox::Common::degToRad(22.5),
                                                4);


    ROS_INFO_STREAM(" Trajectory: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM(" Size: "         << traj.size());
    ROS_INFO_STREAM(" ----------- ");
    for (size_t i = 0; i < traj.size(); i++)
    {
        tf::poseEigenToMsg(traj[i], pose);

        
        ROS_INFO_STREAM(" Point" << i);
        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" Pose: ");
        ROS_INFO_STREAM(" X: " << pose.position.x);
        ROS_INFO_STREAM(" Y: " << pose.position.y);
        ROS_INFO_STREAM(" Z: " << pose.position.z);
        ROS_INFO_STREAM(" ");
    }
}

// linspace
void linspace()
{
    //  LINSPACE
    // -------------------------------
    std::vector<double> linspace;
    linspace = Toolbox::Math::linspace(0.0, 1.0, 10);

    ROS_INFO_STREAM(" Linspace: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM(" Size: "         << linspace.size());
    ROS_INFO_STREAM(" Point 0: "      << linspace[0]);
    ROS_INFO_STREAM(" Point 1: "      << linspace[1]);
    ROS_INFO_STREAM(" Point 2: "      << linspace[2]);
    ROS_INFO_STREAM(" Point 3: "      << linspace[3]);
    ROS_INFO_STREAM(" ");
}

// linspace-vector
void linspace_vec()
{
    //  LINSPACE
    // -------------------------------
    std::vector<Eigen::Vector3d> linspace;
    Eigen::Vector3d start(0, 10, 10);
    Eigen::Vector3d end(10, 30, 40);
    linspace = Toolbox::Math::linspace(start, end, 10);

    ROS_INFO_STREAM(" Linspace: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < linspace.size(); i++)
    {
       ROS_INFO_STREAM(" Point " << i << ": ");
        std::cout << linspace[i] << std::endl;
    }
    ROS_INFO_STREAM(" ");
}

// rotation
void rotmat()
{
    //  Rotation Matrix
    // -------------------------------
    double phi = 15;
    double theta = -58;
    double psi = 37;

    // Quat
    Eigen::AngleAxisd pitch(Toolbox::Common::degToRad(phi), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd roll(Toolbox::Common::degToRad(theta), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(Toolbox::Common::degToRad(psi), Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q = pitch * roll * yaw;
    Eigen::Matrix3d rot_quat = Toolbox::Math::rotMat(q);

    ROS_INFO_STREAM(" Quaternion: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM(" Quat W: " << rot_quat(0));
    ROS_INFO_STREAM(" Quat X: " << rot_quat(1));
    ROS_INFO_STREAM(" Quat Y: " << rot_quat(2));
    ROS_INFO_STREAM(" Quat Z: " << rot_quat(3));
    ROS_INFO_STREAM("Rotation Matrix: ");
    std::cout << rot_quat << std::endl;
    ROS_INFO_STREAM(" ");

    // Euler XYZ
    Eigen::Matrix3d rot_xyz = Toolbox::Math::rotMatXYZ(phi, theta, psi);
    ROS_INFO_STREAM(" Rot XYZ: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Rotation Matrix: ");
    std::cout << rot_xyz << std::endl;
    ROS_INFO_STREAM(" ");

    // Euler ZYX
    Eigen::Matrix3d rot_zyx = Toolbox::Math::rotMatZYX(phi, theta, psi);
    ROS_INFO_STREAM(" Rot ZYX: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Rotation Matrix: ");
    std::cout << rot_zyx << std::endl;
    ROS_INFO_STREAM(" ");

    // Euler ZXZ
    Eigen::Matrix3d rot_zxz = Toolbox::Math::rotMatZXZ(phi, theta, psi);
    ROS_INFO_STREAM(" Rot ZXZ: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Rotation Matrix: ");
    std::cout << rot_zxz << std::endl;
    ROS_INFO_STREAM(" ");

    // Euler ZYZ
    Eigen::Matrix3d rot_zyz = Toolbox::Math::rotMatZYZ(phi, theta, psi);
    ROS_INFO_STREAM(" Rot ZYZ: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Rotation Matrix: ");
    std::cout << rot_zyz << std::endl;
    ROS_INFO_STREAM(" ");
}

// transformation
void transformation()
{
    //  Rotation Matrix
    // -------------------------------
    double x_pos = 8.7;
    double y_pos = 9.6;
    double z_pos = -4.4;
    Eigen::Vector3d pos_vec(x_pos, y_pos, z_pos);
    
    double phi = 15;
    double theta = -58;
    double psi = 37;
    Eigen::Vector3d rot_vec(Toolbox::Common::degToRad(phi), Toolbox::Common::degToRad(theta), Toolbox::Common::degToRad(psi));
    Eigen::Matrix3d rot_xyz = Toolbox::Math::rotMatXYZ(rot_vec);

    // Quat
    Eigen::AngleAxisd pitch(Toolbox::Common::degToRad(phi), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd roll(Toolbox::Common::degToRad(theta), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(Toolbox::Common::degToRad(psi), Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q = pitch * roll * yaw;
    Eigen::Matrix3d rot_quat = Toolbox::Math::rotMat(q);

    // Pos + Rot Mat
    Eigen::Isometry3d tm_rotmat = Toolbox::Math::transMat(pos_vec, rot_xyz);
    ROS_INFO_STREAM(" Transformation Matrix: Rotation Mat: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Translation: ");
    std::cout << tm_rotmat.translation() << std::endl;
    ROS_INFO_STREAM("Rotation: ");
    std::cout << tm_rotmat.rotation() << std::endl;
    ROS_INFO_STREAM(" ");

    // Pos + Quat
    Eigen::Isometry3d tm_quat = Toolbox::Math::transMat(pos_vec, rot_quat);
    ROS_INFO_STREAM(" Transformation Matrix: Quat Rot: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Translation: ");
    std::cout << tm_quat.translation() << std::endl;
    ROS_INFO_STREAM("Rotation: ");
    std::cout << tm_quat.rotation() << std::endl;
    ROS_INFO_STREAM(" ");

    // Pos + Rot Vec
    Eigen::Isometry3d tm_rotvec = Toolbox::Math::transMat(pos_vec, rot_vec);
    ROS_INFO_STREAM(" Transformation Matrix: Rot Vec: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Translation: ");
    std::cout << tm_rotvec.translation() << std::endl;
    ROS_INFO_STREAM("Rotation: ");
    std::cout << tm_rotvec.rotation() << std::endl;
    ROS_INFO_STREAM(" ");

    // Scalar
    Eigen::Isometry3d tm_scalar = Toolbox::Math::transMat(x_pos, y_pos, z_pos, phi, theta, psi);
    ROS_INFO_STREAM(" Transformation Matrix: Scalar: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Translation: ");
    std::cout << tm_scalar.translation() << std::endl;
    ROS_INFO_STREAM("Rotation: ");
    std::cout << tm_scalar.rotation() << std::endl;
    ROS_INFO_STREAM(" ");

}

// lspb
void lspb()
{
    // LSPB
    // -------------------------------
    std::vector<double> lspb_vec;
    double start = 0;
    double end = 1;
    int steps = 10;
    
    lspb_vec = Toolbox::Trajectory::lspb(start, end, steps);

    ROS_INFO_STREAM(" LSPB: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < lspb_vec.size(); i++)
    {
    //    ROS_INFO_STREAM(" Point " << i << ": ");
        std::cout << lspb_vec[i] << std::endl;
    }
    ROS_INFO_STREAM(" ");
}

// lspb
void lspb_time()
{
    // LSPB
    // -------------------------------
    std::vector<double> lspb_vec;
    double start = 0;
    double end = 1;
    std::vector<double> t = Toolbox::Math::linspace(0,5,10);

    lspb_vec = Toolbox::Trajectory::lspb(start, end, t);

    ROS_INFO_STREAM(" LSPB Time: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < lspb_vec.size(); i++)
    {
    //    ROS_INFO_STREAM(" Point " << i << ": ");
        std::cout << lspb_vec[i] << std::endl;
    }
    ROS_INFO_STREAM(" ");
}

// lspb vec
void lspb_vec()
{
    // LSPB
    // -------------------------------
    std::vector<Eigen::Vector3d> lspb_vec;
    Eigen::Vector3d start(0, 0, 0);
    Eigen::Vector3d end(1, 10, 100);
    int steps = 10;
    // lspb_vec = Toolbox::Trajectory::lspb(start, end, steps);

    ROS_INFO_STREAM(" LSPB: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < lspb_vec.size(); i++)
    {
       ROS_INFO_STREAM(" Point " << i << ": ");
        std::cout << lspb_vec[i] << std::endl;
    }
    ROS_INFO_STREAM(" ");
}

// lspb vec
void lspb_vec_time()
{
    // LSPB
    // -------------------------------
    std::vector<Eigen::Vector3d> lspb_vec;
    Eigen::Vector3d start(0, 0, 0);
    Eigen::Vector3d end(1, 10, 100);
    std::vector<double> t = Toolbox::Math::linspace(0,5,10);
    lspb_vec = Toolbox::Trajectory::lspb(start, end, t);

    ROS_INFO_STREAM(" LSPB: Time");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < lspb_vec.size(); i++)
    {
        ROS_INFO_STREAM(" Point " << i << ": ");
        std::cout << lspb_vec[i] << std::endl;
    }
    ROS_INFO_STREAM(" ");
}

// lerp (n number of steps)
void lerp()
{
    // LERP
    // -------------------------------
    std::vector<double> lerp_vec;
    double start = 0;
    double end = 10;
    int steps = 10;
    lerp_vec = Toolbox::Math::lerp(start, end, steps);

    ROS_INFO_STREAM(" LERP - N: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < lerp_vec.size(); i++)
    {
    //    ROS_INFO_STREAM(" Point " << i << ": ");
        std::cout << lerp_vec[i] << std::endl;
    }
    ROS_INFO_STREAM(" ");
}

// slerp 
void slerp()
{
    // SLERP
    // -------------------------------
    std::vector<Eigen::Quaterniond> slerp_vec;

    double start_rotx = Toolbox::Common::degToRad(0);
    double start_roty = Toolbox::Common::degToRad(0);
    double start_rotz = Toolbox::Common::degToRad(0);

    double end_rotx = Toolbox::Common::degToRad(45);
    double end_roty = Toolbox::Common::degToRad(0);
    double end_rotz = Toolbox::Common::degToRad(0);

    Eigen::Quaterniond start =  Eigen::AngleAxisd(start_rotx, Eigen::Vector3d::UnitX())
                                * Eigen::AngleAxisd(start_roty, Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(start_rotz, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond end = Eigen::AngleAxisd(end_rotx, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(end_roty, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(end_rotz, Eigen::Vector3d::UnitZ());
    
    int steps = 10;
    slerp_vec = Toolbox::Math::slerp(start, end, steps);

    ROS_INFO_STREAM(" LERP - DT: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < slerp_vec.size(); i++)
    {
    //    ROS_INFO_STREAM(" Point " << i << ": ");
        ROS_INFO_STREAM(" Quaternion: ");
        std::cout << slerp_vec[i].w() << std::endl;
        std::cout << slerp_vec[i].vec() << std::endl;
        
        ROS_INFO_STREAM(" Euler: ");
        Eigen::Vector3d euler = Toolbox::Common::quaternionToEuler(slerp_vec[i], Toolbox::XYZ);
        std::cout << euler*Toolbox::Common::radToDeg() << std::endl;

        ROS_INFO_STREAM(" ----------- ");
    }
    ROS_INFO_STREAM(" ");
}

void polyval()
{
    std::vector<double> v = {2, -6, 2, -1};
    Eigen::VectorXd poly;
    poly = Eigen::VectorXd::Map(v.data(), v.size());

    double x = 3;
    double result = Toolbox::Math::polyval(poly,x);

    ROS_INFO_STREAM(" POLY VAL: ");
    ROS_INFO_STREAM(" ----------- ");
    std::cout << result << std::endl;
    // std::cout << poly << std::endl;
    ROS_INFO_STREAM(" ");
}

void polyval_vec()
{
    std::vector<double> v = {2, -6, 2, -1};
    Eigen::VectorXd poly;
    poly = Eigen::VectorXd::Map(v.data(), v.size());

    std::vector<double> x = {3, 0};
    Eigen::VectorXd xs;
    xs = Eigen::VectorXd::Map(x.data(), x.size());

    Eigen::VectorXd result = Toolbox::Math::polyval(poly,xs);

    ROS_INFO_STREAM(" POLY VAL: ");
    ROS_INFO_STREAM(" ----------- ");
    std::cout << result << std::endl;
    // std::cout << poly << std::endl;
    ROS_INFO_STREAM(" ");
}

void quintic_poly()
{
    // Define matrix-equation
    double p_s  = 1;
    double p_f = 2;
    int n = 5;
    double v_s = 3;
    double v_f = 4;

    std::vector<double> traj = Toolbox::Trajectory::polyQuintic(p_s, p_f, n, v_s, v_f);
    
    ROS_INFO_STREAM(" Quintic POLYNOMIAL: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < traj.size(); i++)
    {
    //    ROS_INFO_STREAM(" Point " << i << ": ");
        std::cout << traj[i] << std::endl;
    }
    ROS_INFO_STREAM(" ");
}

void quintic_poly_time()
{
    // Define matrix-equation
    double p_s  = 1;
    double p_f = 2;
    double v_s = 3;
    double v_f = 4;

    std::vector<double> t = Toolbox::Math::linspace(0, 10, 5);

    std::vector<double> traj = Toolbox::Trajectory::polyQuintic(p_s, p_f, t, v_s, v_f);
    
    ROS_INFO_STREAM(" Quintic POLYNOMIAL - Time: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < traj.size(); i++)
    {
    //    ROS_INFO_STREAM(" Point " << i << ": ");
        std::cout << traj[i] << std::endl;
    }
    ROS_INFO_STREAM(" ");
}

void quintic_poly_vec()
{
    
    std::vector<Eigen::Vector3d> traj;
    Eigen::Vector3d p_s(0, 0, 0);
    Eigen::Vector3d p_f(1, 10, 100);
    int n = 10;

    traj = Toolbox::Trajectory::polyQuintic(p_s, p_f, n);
    
    ROS_INFO_STREAM(" Quintic POLYNOMIAL - Vector: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < traj.size(); i++)
    {
        ROS_INFO_STREAM(" Point " << i << ": ");
        std::cout << traj[i] << std::endl;
    }
    ROS_INFO_STREAM(" ");
}

void quintic_poly_vec_time()
{
    
    std::vector<Eigen::Vector3d> traj;
    Eigen::Vector3d p_s(0, 0, 0);
    Eigen::Vector3d p_f(1, 10, 100);
    std::vector<double> t = Toolbox::Math::linspace(0, 10, 10);

    traj = Toolbox::Trajectory::polyQuintic(p_s, p_f, t);
    
    ROS_INFO_STREAM(" Quintic POLYNOMIAL - Vector Time: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < traj.size(); i++)
    {
        ROS_INFO_STREAM(" Point " << i << ": ");
        std::cout << traj[i] << std::endl;
    }
    ROS_INFO_STREAM(" ");
}

void vec_convert()
{
    Eigen::VectorXd eigen_vec_in(5);
    eigen_vec_in << 9, 7, 5, 3, 1;

    std::vector<double> std_vec_in;
    
    std_vec_in = {123, 456, 789, 111};

    std::vector<double> std_vec_res;
    std_vec_res = Toolbox::Common::vectorEigenToStd(eigen_vec_in);
    Eigen::VectorXd  eigen_vec_res = Toolbox::Common::vectorStdToEigen(std_vec_in);

    ROS_INFO_STREAM(" Convert Vector: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM(" Eigen to Std: ");
    for (int i = 0; i < std_vec_res.size(); i++)
    {
        std::cout << std_vec_res[i] << std::endl;
    }
    ROS_INFO_STREAM(" Std to Eigen: ");
    std::cout << eigen_vec_res << std::endl;
    ROS_INFO_STREAM(" ");

}

void cubic_poly()
{
    // Define matrix-equation
    double p_s  = 1;
    double p_f = 2;
    int n = 5;
    double v_s = 3;
    double v_f = 4;

    std::vector<double> traj = Toolbox::Trajectory::polyCubic(p_s, p_f, n, v_s, v_f);
    
    ROS_INFO_STREAM(" cubic POLYNOMIAL: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < traj.size(); i++)
    {
    //    ROS_INFO_STREAM(" Point " << i << ": ");
        std::cout << traj[i] << std::endl;
    }
    ROS_INFO_STREAM(" ");
}

void cubic_poly_time()
{
    // Define matrix-equation
    double p_s  = 1;
    double p_f = 2;
    double v_s = 3;
    double v_f = 4;

    std::vector<double> t = Toolbox::Math::linspace(0, 10, 5);

    std::vector<double> traj = Toolbox::Trajectory::polyCubic(p_s, p_f, t, v_s, v_f);
    
    ROS_INFO_STREAM(" cubic POLYNOMIAL - Time: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < traj.size(); i++)
    {
    //    ROS_INFO_STREAM(" Point " << i << ": ");
        std::cout << traj[i] << std::endl;
    }
    ROS_INFO_STREAM(" ");
}

void cubic_poly_vec()
{
    
    std::vector<Eigen::Vector3d> traj;
    Eigen::Vector3d p_s(0, 0, 0);
    Eigen::Vector3d p_f(1, 10, 100);
    int n = 10;

    traj = Toolbox::Trajectory::polyCubic(p_s, p_f, n);
    
    ROS_INFO_STREAM(" cubic POLYNOMIAL - Vector: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < traj.size(); i++)
    {
        ROS_INFO_STREAM(" Point " << i << ": ");
        std::cout << traj[i] << std::endl;
    }
    ROS_INFO_STREAM(" ");
}

void cubic_poly_vec_time()
{
    
    std::vector<Eigen::Vector3d> traj;
    Eigen::Vector3d p_s(0, 0, 0);
    Eigen::Vector3d p_f(1, 10, 100);
    std::vector<double> t = Toolbox::Math::linspace(0, 10, 10);

    traj = Toolbox::Trajectory::polyCubic(p_s, p_f, t);
    
    ROS_INFO_STREAM(" cubic POLYNOMIAL - Vector Time: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < traj.size(); i++)
    {
        ROS_INFO_STREAM(" Point " << i << ": ");
        std::cout << traj[i] << std::endl;
    }
    ROS_INFO_STREAM(" ");
}

void linear_trajectory()
{
    // Transformation Start
    // -------------------------------
        // Translation
        double x0 = 0;
        double y0 = 0;
        double z0 = 0;
        Eigen::Vector3d pos0(x0, y0, z0);

        // Rotation
        double phi0 = 0;
        double theta0 = 0;
        double psi0 = 0;
        Eigen::Vector3d rot0(Toolbox::Common::degToRad(phi0), 
                            Toolbox::Common::degToRad(theta0), 
                            Toolbox::Common::degToRad(psi0));

        // Transformation
        Eigen::Isometry3d tm0 = Toolbox::Math::transMat(pos0, rot0);
        
    // Transformation Start
    // -------------------------------
        // Translation
        double x1 = 10;
        double y1 = 5;
        double z1 = 50;
        Eigen::Vector3d pos1(x1, y1, z1);

        // Rotation
        double phi1 = 0;
        double theta1 = 0;
        double psi1 = 90;
        Eigen::Vector3d rot1(Toolbox::Common::degToRad(phi1), 
                            Toolbox::Common::degToRad(theta1), 
                            Toolbox::Common::degToRad(psi1));

        // Transformation
        Eigen::Isometry3d tm1 = Toolbox::Math::transMat(pos1, rot1);

    // Trajectory
    // -------------------------------
        int steps = 5;
        std::vector<Eigen::Isometry3d> trajectory = Toolbox::Trajectory::trajectoryLinear(tm0, tm1, steps);

    // Debug
    // -------------------------------
    ROS_INFO_STREAM(" Transformation Trajectory: ");
    ROS_INFO_STREAM(" ----------- ");
    for (int i = 0; i < trajectory.size(); i++)
    {
        ROS_INFO_STREAM(" Point " << i << ": ");
        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM("Translation: ");
        std::cout << trajectory[i].translation() << std::endl;

        ROS_INFO_STREAM(" ");
        ROS_INFO_STREAM("Orientation: ");
            Eigen::Quaterniond q(trajectory[i].rotation());

            Eigen::Vector3d euler = Toolbox::Common::quaternionToEuler(q, Toolbox::XYZ);
            euler = euler * Toolbox::Common::radToDeg();
            std::cout << euler << std::endl;

        // std::cout << q.w() << std::endl;
        // std::cout << q.vec() << std::endl;
        // std::cout << q.vec()*Toolbox::Common::radToDeg() << std::endl;
        
        ROS_INFO_STREAM(" ");
    }
    ROS_INFO_STREAM(" ");
}

// Test Toolbox Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a ROS Node with a node name
    ros::init(argc, argv, "toolbox_node");   
    
    // Starting ROS Nodehandle(s)
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // ROS Publisher(s)
    // -------------------------------
    // Initialize publisher for trajectory visualization marker
    ros::Publisher plot_pub = nh.advertise<std_msgs::Float64MultiArray>(
                                "plot_pub",     // Topic name
                                1);             // Queue size 

    // Main Code    
    // -------------------------------
        
        // lspb_vec_time();
        
        // vec_convert();

        // rotmat();
        // transformation();

        // linspace_vec();

        // interpolate_lin_vec();

        // linspace();
        // lerp();

        // slerp();

        // polyval();
        // polyval_vec();

        // lspb();
        // lspb_time();

        // lspb_vec();
        // lspb_vec_time();


        // quintic_poly();
        // quintic_poly_time();

        // quintic_poly_vec();
        // quintic_poly_vec_time();

        // cubic_poly();
        // cubic_poly_time();

        // cubic_poly_vec();
        // cubic_poly_vec_time();


        // linear_trajectory();

        Eigen::Isometry3d pose;
        Toolbox::Kinematics::getCurrentPose("robot_tcp", "world", pose, true);

        // slerp();

        // while (ros::ok())
        // {
        //     plot_pub.publish(msg);
        // }
        

    

    // Shutdown
    // -------------------------------
    // ROS-Loop waiting for shutdown
    ros::waitForShutdown();

    // Function return
    return 0;
}
