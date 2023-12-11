// Robotics Toolbox - Visualization Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Visualization Tools contains helper and utility functions 
//      related to display of objects in RVIZ
//
// Version:
//  0.1 - Initial Version
//        [06.01.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robot_toolbox/tools/visual.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Visualization Tool Class - Members:
// -------------------------------

    // Constants
    // -------------------------------
    const std::string Visual::CLASS_PREFIX = "Toolbox::Visual::";
    const std_msgs::ColorRGBA Visual::COLOR_RED = setColorRed();
    const std_msgs::ColorRGBA Visual::COLOR_BLUE = setColorBlue();
    const std_msgs::ColorRGBA Visual::COLOR_GREEN = setColorGreen();
    const std_msgs::ColorRGBA Visual::COLOR_YELLOW = setColorYellow();


    // Set Color Red (Color-Pointer)
    // -------------------------------
    // (Function overloading)
    void Visual::setColorRed(std_msgs::ColorRGBA& color)
    {
        // Assign RGBA-values
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
    }


    // Set Color Red (Return color)
    // -------------------------------
    // (Function overloading)
    std_msgs::ColorRGBA Visual::setColorRed()
    {
        // Define Color
        std_msgs::ColorRGBA color;
        
        // Set Color Red
        setColorRed(color);

        // Function return:
        return color;
    }


    // Set Color Blue (Color-Pointer)
    // -------------------------------
    // (Function overloading)
    void Visual::setColorBlue(std_msgs::ColorRGBA& color)
    {
        // Assign RGBA-values
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
    }


    // Set Color Blue (Return color)
    // -------------------------------
    // (Function overloading)
    std_msgs::ColorRGBA Visual::setColorBlue()
    {
        // Define Color
        std_msgs::ColorRGBA color;
        
        // Set Color Blue
        setColorBlue(color);

        // Function return:
        return color;
    }


    // Set Color Green (Color-Pointer)
    // -------------------------------
    // (Function overloading)
    void Visual::setColorGreen(std_msgs::ColorRGBA& color)
    {
        // Assign RGBA-values
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
    }


    // Set Color Green (Return color)
    // -------------------------------
    // (Function overloading)
    std_msgs::ColorRGBA Visual::setColorGreen()
    {
        // Define Color
        std_msgs::ColorRGBA color;
        
        // Set Color Green
        setColorGreen(color);

        // Function return:
        return color;
    }


    // Set Color Yellow (Color-Pointer)
    // -------------------------------
    // (Function overloading)
    void Visual::setColorYellow(std_msgs::ColorRGBA& color)
    {
        // Assign RGBA-values
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
    }


    // Set Color Yellow (Return color)
    // -------------------------------
    // (Function overloading)
    std_msgs::ColorRGBA Visual::setColorYellow()
    {
        // Define Color
        std_msgs::ColorRGBA color;
        
        // Set Color Yellow
        setColorYellow(color);

        // Function return:
        return color;
    }
    

    // Visualize Line
    // -------------------------------
    // (Function Overloading)
    visualization_msgs::Marker Visual::visualLine(
        std::vector<Eigen::Vector3d> points,
        std::string ns,
        std_msgs::ColorRGBA color,
        double width,
        std::string ref_frame)
    {
        // Define rviz marker
        visualization_msgs::Marker marker;
        geometry_msgs::Point point;

        // Configure line marker
        // -------------------------------
        marker.ns = ns;
        marker.id = 0;
        marker.color = color;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0);
        marker.header.frame_id = ref_frame;
        marker.scale.x = width;                 // Line width
        marker.points.reserve(points.size());   // reserve points for marker

        // Create line Markers
        // -------------------------------
        // Iterate over points in vector
        for(unsigned int i = 0; i < points.size(); i++)
        {
            // Convert Eigen-Vector to Point
            tf::pointEigenToMsg(points[i], point);

            // Append current point
            marker.points.push_back(point);
        }

        // Function return
        return marker;
    }


    // Visualize Line
    // -------------------------------
    // (Function Overloading)
    visualization_msgs::Marker Visual::visualLine(
        std::vector<geometry_msgs::Point> points,
        std::string ns,
        std_msgs::ColorRGBA color,
        double width,
        std::string ref_frame)
    {
        // Define rviz marker
        visualization_msgs::Marker marker;

        // Configure line marker
        // -------------------------------
        marker.ns = ns;
        marker.id = 0;
        marker.color = color;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0);
        marker.header.frame_id = ref_frame;
        marker.scale.x = width;                 // Line width
        marker.points.reserve(points.size());   // reserve points for marker

        // Create line Markers
        // -------------------------------
        // Iterate over points in vector
        for(unsigned int i = 0; i < points.size(); i++)
        {
            // Append current point
            marker.points.push_back(points[i]);
        }

        // Function return
        return marker;
    }


    // Visualize Pose
    // -------------------------------
    // (Function Overloading)
    visualization_msgs::Marker Visual::visualPose(
            Eigen::Isometry3d pose_tm,
            std::string ns,
            AxisType axis_type,
            std_msgs::ColorRGBA color,
            double scale,
            std::string ref_frame)
    {
        // Define rviz marker
        visualization_msgs::Marker marker;

        // Configure arrow marker
        // -------------------------------
        marker.ns = ns;
        marker.id = axis_type.id;
        marker.color = color;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0);
        marker.header.frame_id = ref_frame;
        marker.scale.x = scale / 15;    // arrow shaft diameter
        marker.scale.y = scale / 10;    // arrow head diameter
        marker.scale.z = scale / 5;     // arrow head length
        marker.points.reserve(2);       // reserve points for marker (start and end of arrow)

        // Create Axis Markers
        // ------------------------------- 
        geometry_msgs::Point p_start;   // Pose arrow start-point    
        geometry_msgs::Point p_end;     // Pose arrow end-point

        // Calculate Start- and End-Vector
        Eigen::Vector3d vec_start = pose_tm.translation();                  // get translation from transformation matrix
        Eigen::Vector3d vec_end = pose_tm * (axis_type.unit_vec * scale);   // direction based on unit-vector and scale

        // Calculate end point of axis
        tf::pointEigenToMsg(vec_start, p_start);
        tf::pointEigenToMsg(vec_end, p_end);

        // Assign start- and end point to arrow marker
        marker.points.push_back(p_start);
        marker.points.push_back(p_end);

        // Function return
        return marker;
    }
    

    // Visualize Pose
    // -------------------------------
    // (Function Overloading)
    visualization_msgs::Marker Visual::visualPose(
            geometry_msgs::PoseStamped pose,
            std::string ns,
            AxisType axis_type,
            std_msgs::ColorRGBA color,
            double scale)
    {
        // Define rviz marker
        visualization_msgs::Marker marker;

        // Configure arrow marker
        // -------------------------------
        marker.ns = ns;
        marker.id = axis_type.id;
        marker.color = color;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0);
        marker.header.frame_id = pose.header.frame_id;
        marker.scale.x = scale / 15;    // arrow shaft diameter
        marker.scale.y = scale / 10;    // arrow head diameter
        marker.scale.z = scale / 5;     // arrow head length
        marker.points.reserve(2);       // reserve points for marker (start and end of arrow)

        // Create Axis Markers
        // -------------------------------
        Eigen::Isometry3d pose_tm;      // Pose Isometry Transformation Matrix
        geometry_msgs::Point p_start;   // Pose arrow start-point    
        geometry_msgs::Point p_end;     // Pose arrow end-point

        // Set marker start point
        p_start = pose.pose.position;

        // Get Transformation Matrix of Pose-CSYS
        tf2::fromMsg(pose.pose, pose_tm);

        // Calculate marker direction based on unit-vector and scale
        Eigen::Vector3d marker_dir = pose_tm * (axis_type.unit_vec * scale);
        
        // Calculate end point of axis
        tf::pointEigenToMsg(marker_dir, p_end);

        // Assign start- and end point to arrow marker
        marker.points.push_back(p_start);
        marker.points.push_back(p_end);

        // Function return
        return marker;
    }

    
    // Visualize Pose Coordinate System
    // -------------------------------
    // (Function Overloading)
    visualization_msgs::MarkerArray Visual::visualPoseCsys(
        Eigen::Isometry3d pose_csys_tm,
        std::string name, 
        double scale,
        std::string ref_frame)
    {
        // Define rviz markers
        visualization_msgs::Marker x_axis, y_axis, z_axis;
        visualization_msgs::MarkerArray csys;

        // Create CSYS axis arrow markers
        // -------------------------------
            // X-Axis Arrow Marker
            x_axis = Toolbox::Visual::visualPose(pose_csys_tm,          // Pose of Coordinate system
                                                name + "/x_axis",       // Namespace for arrow marker
                                                AXIS_X,                 // Axis of axis-arrow-marker
                                                Visual::COLOR_RED,      // Color of axis-arrow marker 
                                                scale,                  // Scale of axis-arrow marker
                                                ref_frame);             // Reference Frame for axis-arrow marker            

            // Y-Axis Arrow Marker
            y_axis = Toolbox::Visual::visualPose(pose_csys_tm,          // Pose of Coordinate system
                                                name + "/y_axis",       // Namespace for arrow marker  
                                                AXIS_Y,                 // Axis of axis-arrow-marker 
                                                Visual::COLOR_GREEN,    // Color of axis-arrow marker   
                                                scale,                  // Scale of axis-arrow marker
                                                ref_frame);             // Reference Frame for axis-arrow marker  

            // Z-Axis Arrow Marker
            z_axis = Toolbox::Visual::visualPose(pose_csys_tm,          // Pose of Coordinate system
                                                name + "/z_axis",       // Namespace for arrow marker
                                                AXIS_Z,                 // Axis of axis-arrow-marker
                                                Visual::COLOR_BLUE,     // Color of axis-arrow marker
                                                scale,                  // Scale of axis-arrow marker
                                                ref_frame);             // Reference Frame for axis-arrow marker  

        // Assign Axis-Markers to CSYS Marker Array
        csys.markers.push_back(x_axis);
        csys.markers.push_back(y_axis);
        csys.markers.push_back(z_axis);

        // Function return
        return csys;
    }


    // Visualize Pose Coordinate System
    // -------------------------------
    // (Function Overloading)
    visualization_msgs::MarkerArray Visual::visualPoseCsys(
        geometry_msgs::PoseStamped pose_csys, 
        std::string name,
        double scale)
    {
        // Define rviz markers
        visualization_msgs::Marker x_axis, y_axis, z_axis;
        visualization_msgs::MarkerArray csys;

        // Create CSYS axis arrow markers
        // -------------------------------
            // X-Axis Arrow Marker
            x_axis = Toolbox::Visual::visualPose(pose_csys,             // Pose of Coordinate system
                                                name + "/x_axis",       // Namespace for arrow marker
                                                AXIS_X,                 // Axis of axis-arrow-marker
                                                Visual::COLOR_RED,      // Color of axis-arrow marker 
                                                scale);                 // Scale of axis-arrow marker

            // Y-Axis Arrow Marker
            y_axis = Toolbox::Visual::visualPose(pose_csys,             // Pose of Coordinate system
                                                name + "/y_axis",       // Namespace for arrow marker
                                                AXIS_Y,                 // Axis of axis-arrow-marker 
                                                Visual::COLOR_GREEN,    // Color of axis-arrow marker   
                                                scale);                 // Scale of axis-arrow marker

            // Z-Axis Arrow Marker
            z_axis = Toolbox::Visual::visualPose(pose_csys,             // Pose of Coordinate system
                                                name + "/z_axis",       // Namespace for arrow marker
                                                AXIS_Z,                 // Axis of axis-arrow-marker
                                                Visual::COLOR_BLUE,     // Color of axis-arrow marker
                                                scale);                 // Scale of axis-arrow marker

        // Assign Axis-Markers to CSYS Marker Array
        csys.markers.push_back(x_axis);
        csys.markers.push_back(y_axis);
        csys.markers.push_back(z_axis);

        // Function return
        return csys;
    }


    // Visualize Normal-Vector
    // -------------------------------
    visualization_msgs::Marker Visual::visualNormalVector(
            Eigen::Isometry3d pose,
            std::string ns,
            AxisType axis_type,
            std_msgs::ColorRGBA color,
            double scale)
    {
        // Define rviz marker
        visualization_msgs::Marker marker;

        // Configure arrow marker
        // -------------------------------
        marker.ns = ns;
        marker.id = axis_type.id;
        marker.color = color;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0);
        marker.header.frame_id = "world";
        marker.scale.x = scale / 15;    // arrow shaft diameter
        marker.scale.y = scale / 10;    // arrow head diameter
        marker.scale.z = scale / 5;     // arrow head length
        marker.points.reserve(2);       // reserve points for marker (start and end of arrow)

        // Local variable holders
        Eigen::Vector3d v_start;        // Start-Point vector  
        Eigen::Vector3d v_end;          // End-Point vector
        Eigen::Vector3d v_normal;       // Normal vector

        geometry_msgs::Point p_start;   // Pose arrow start-point    
        geometry_msgs::Point p_end;     // Pose arrow end-point

        // Calculation
        // -------------------------------
        // Calculate start-point vector 
        v_start = pose.translation();

        // Calculate Normal-Vector
        v_normal = Math::getNormalVector(pose, axis_type);

        // Calculate end-point vector
        v_end = v_start + v_normal*scale;

        // Create Arrow Marker
        // -------------------------------
        // Convert eigen vector to point
        tf::pointEigenToMsg(v_start, p_start);
        tf::pointEigenToMsg(v_end, p_end);

        // Assign start- and end point to arrow marker
        marker.points.push_back(p_start);
        marker.points.push_back(p_end);

        // Function return
        return marker;
    }


    // Visualize Normal-Vector
    // -------------------------------
    visualization_msgs::Marker Visual::visualNormalVector(
            geometry_msgs::Pose pose,
            std::string ns,
            AxisType axis_type,
            std_msgs::ColorRGBA color,
            double scale)
    {
        // Define rviz marker
        visualization_msgs::Marker marker;

        // Pose Transformation Matrix
        Eigen::Isometry3d pose_tm;

        // Convert Pose [geometry_msgs::Pose] to Pose Transformation [Eigen::Isometry3d]
        tf::poseMsgToEigen(pose, pose_tm);

        // Get Normal-Vector arrow marker
        marker = visualNormalVector(pose_tm, ns, axis_type, color, scale);

        // Function return
        return marker;
    }
 

    // Visualize Pose Trajectory
    // -------------------------------
    // (Function Overloading)
    visualization_msgs::MarkerArray Visual::visualPoseTrajectory(
        std::vector<Eigen::Isometry3d> pose_trajectory,
        double csys_distance,
        double csys_scale)
    {
        // Define local variables and rviz markers
        std::vector<Eigen::Vector3d> points;
        Eigen::Vector3d point;
        Eigen::Vector3d prev_point;
        double distance = 0;
        double distance_last_csys = 0;
        visualization_msgs::Marker line;
        visualization_msgs::MarkerArray csys, trajectory;
                                            
        // Iterate over each pose of pose trajectory vector
        for (size_t i = 0; i < pose_trajectory.size(); i++)
        {
            // Point Marker Namespace
            std::string point_name = "point_" + std::to_string(i); 

            // Get Point of Pose Trajectory
            point = pose_trajectory[i].translation();

            // Append Point to Point Vector
            points.push_back(point);

            // Calculate distance between previous point
            distance = (point - prev_point).norm();
            
            // Trajectory CSYS Marker
            // -------------------------------
            // CSYS marker is added at certain distance interval
            // Check if distance between points or distance since last csys execeeds threshold
            if ((distance > csys_distance) || (distance_last_csys > csys_distance))
            {
                // Get CSYS Axis marker
                csys = Visual::visualPoseCsys(pose_trajectory[i], 
                                              point_name,
                                              csys_scale);

                // Iterate over csys array of arrow markers
                for (size_t j = 0; j < csys.markers.size(); j++)
                {
                    // Assign CSYS-Markers to Trajectory Marker Array
                    trajectory.markers.push_back(csys.markers[j]);
                }

                // Reset distance since last csys
                distance_last_csys = 0;
            }
            // CSYS marker is not added
            else
            {
                // Increase the distance since last csys marker
                distance_last_csys += distance;
            }
            
            // Update previous point
            prev_point = point;
        }

        // Trajectory Line marker
        // -------------------------------
        line = Toolbox::Visual::visualLine(points,                  // Pose Trajectory
                                           "line",                  // Namespace for line marker
                                           Visual::COLOR_YELLOW,    // Color of line marker 
                                           0.005,                   // Width of line marker
                                           "world");                // Reference Frame for Line Marker

        // Assign Line Marker to Trajectory Marker Array
        trajectory.markers.push_back(line);

        // Function return
        return trajectory;
    }


    // Visualize Pose Trajectory
    // -------------------------------
    // (Function Overloading)
    visualization_msgs::MarkerArray Visual::visualPoseTrajectory(
        std::vector<geometry_msgs::PoseStamped> pose_trajectory,
        double csys_distance,
        double csys_scale)
    {
        // Define local variables and rviz markers
        std::vector<geometry_msgs::Point> points;
        Eigen::Vector3d point;
        Eigen::Vector3d prev_point;
        double distance = 0;
        double distance_last_csys = 0;
        visualization_msgs::Marker line;
        visualization_msgs::MarkerArray csys, trajectory;
                                            
        // Iterate over each pose of pose trajectory vector
        for (size_t i = 0; i < pose_trajectory.size(); i++)
        {
            // Point Marker Namespace0
            std::string point_name = "point_" + std::to_string(i); 

            // Get Point of Pose Trajectory
            points.push_back(pose_trajectory[i].pose.position);
            tf::pointMsgToEigen(pose_trajectory[i].pose.position, point);

            // Calculate distance between previous point
            distance = (point - prev_point).norm();
            
            // Trajectory CSYS Marker
            // -------------------------------
            // CSYS marker is added at certain distance interval
            // Check if distance between points or distance since last csys execeeds threshold
            if ((distance > csys_distance) || (distance_last_csys > csys_distance))
            {
                // Get CSYS Axis marker
                csys = Visual::visualPoseCsys(pose_trajectory[i], 
                                              point_name,
                                              csys_scale);

                // Iterate over csys array of arrow markers
                for (size_t j = 0; j < csys.markers.size(); j++)
                {
                    // Assign CSYS-Markers to Trajectory Marker Array
                    trajectory.markers.push_back(csys.markers[j]);
                }

                // Reset distance since last csys
                distance_last_csys = 0;
            }
            // CSYS marker is not added
            else
            {
                // Increase the distance since last csys marker
                distance_last_csys += distance;
            }
            
            // Update previous point
            prev_point = point;
        }

        // Trajectory Line marker
        // -------------------------------
        line = Toolbox::Visual::visualLine(points,                  // Pose Trajectory
                                           "line",                  // Namespace for line marker
                                           Visual::COLOR_YELLOW,    // Color of line marker 
                                           0.005,                   // Width of line marker
                                           "world");                // Reference Frame for Line Marker

        // Assign Line Marker to Trajectory Marker Array
        trajectory.markers.push_back(line);

        // Function return
        return trajectory;
    }
    
} // End Namespace: Robotics Toolbox