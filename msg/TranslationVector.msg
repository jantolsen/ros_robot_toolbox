# Toolbox - Translation Vector
# -------------------------------
# Description:
#   Message-type for Translational Vector
#   Typically used for approach and retract motion
#
# Version:
#   0.1 -   Initial version
#           [04.01.2024] - Jan T. Olsen
# -------------------------------

# Information header
string name         # Identifier name for translation vector
string ref_frame    # Reference frame/target the translation vector is associated with
int32 seq_id        # Sequence ID for identifiying execution order of multiple translation vectors

# Direction of translation
# (Specifies the axis of direction to the reference frame/target)
geometry_msgs/Vector3 direction 

# Distance of translation
# (Specifies the distance for the defined direction to the refenence frame/target)
float64 distance 

