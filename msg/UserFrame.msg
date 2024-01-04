# Toolbox - User Frame
# -------------------------------
# Description:
#   Message-type for User Frame
#   Used to define a custom user frame
#
# Version:
#   0.1 -   Initial version
#           [04.01.2024] - Jan T. Olsen
# -------------------------------

# Information header
string name         # Name of user frame
string ref_frame    # Reference frame for which the custom user frame is relative to

# Pose RPY
# (user friendly configuration of pose, typically the custom configuration)
robot_toolbox/PoseRPY poseRPY

# Transform Stamped
# (data type used by tf2 broadcasting, typically calculated from "poseRPY" data)
geometry_msgs/TransformStamped transformStamped