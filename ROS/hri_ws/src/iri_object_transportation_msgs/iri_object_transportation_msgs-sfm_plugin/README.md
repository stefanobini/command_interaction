# iri_object_transportation_msgs

IRI package with a custom ROS message.

## Structure

wrenchStampedArray:
    std_msgs/Header header
    geometry_msgs/WrenchStamped[] wrench_array

localForcesSFM:
    std_msgs/Header header
    geometry_msgs/WrenchStamped goal_force
    iri_object_transportation_msgs/wrenchStampedArray obstacles_forces
    geometry_msgs/WrenchStamped total_force
    
localForcesCoefficients:
    std_msgs/Header header
    float32 goal_coefficient
    float32[] obstacles_coefficients
    float32 total_coefficient

narrowPathMarkersArray:
    std_msgs/Header header
    geometry_msgs/PoseStamped[] marker_pose_array
    geometry_msgs/WrenchStamped[] marker_wrench_array
    
twistStamped:
    std_msgs/Header header
    geometry_msgs/Twist twist

twistStampedArray:
    std_msgs/Header header
    iri_object_transportation_msgs/twistStamped[] twist_array
