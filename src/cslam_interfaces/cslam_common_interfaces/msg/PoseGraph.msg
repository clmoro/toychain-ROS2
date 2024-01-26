uint32 robot_id
uint32 origin_robot_id

RobotIds connected_robots

cslam_common_interfaces/PoseGraphEdge[] edges
cslam_common_interfaces/PoseGraphValue[] values

# The following fields are for logging purposes only
# and should be left empty otherwise.
# They are not used by the CSLAM system.
uint32[] gps_values_idx
sensor_msgs/NavSatFix[] gps_values
uint32 nb_matches
uint32 nb_failed_matches
uint32 nb_vertices_transmitted
uint32 nb_matches_selected
uint64 front_end_cumulative_communication_bytes
float64 sparsification_cumulative_computation_time
cslam_common_interfaces/InterRobotMatches spectral_matches
