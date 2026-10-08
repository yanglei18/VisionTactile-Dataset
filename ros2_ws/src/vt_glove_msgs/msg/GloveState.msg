# Solved hand state only. Never contains raw gyro/accelerometer measurements.
# header.stamp == host_realtime_ns, NOT ROS publication time. A zero stamp is invalid.
# frame_id=glove_left_reference or glove_right_reference: wrist origin,
# calibration-reference axes, NOT world. Each message describes exactly one hand.
std_msgs/Header header
uint16 schema_version
string hand
string source_mode
string session_id
uint64 frame_index
uint32 calibration_version
string config_sha256
string model_rest_sha256
string algorithm
string quality
string reason
bool pose_valid
bool time_valid
# Reference is the palm sample used by this solve; host receive != hardware sample time.
uint64 host_realtime_ns
uint64 host_monotonic_ns
uint8 reference_node_id
uint64 solve_completed_monotonic_ns
string clock_domain
string timing_source
# Parallel arrays identify exact per-node inputs, not simultaneous hardware samples.
uint8[] source_node_ids
uint64[] source_realtime_ns
uint64[] source_monotonic_ns
uint64[] source_sample_indices
uint32[] source_filter_epochs
# 16 MANO rotations x 3 rad, including root; empty when pose unavailable.
float64[] mano_pose_rotvec
# Root rotation in xyzw order, already included in the MANO pose and joint positions.
float64[] root_quat_xyzw
float64[] shape_params
# Optional 21 x 3 meters, wrist origin, already root-rotated. Do not rotate twice.
float64[] joint_positions_m
# 16-segment provenance: observed does not mean independent anatomical truth.
bool[] segment_observed_mask
bool[] joint_observed_mask
bool[] inferred_mask
string[] source_labels
bool units_verified
bool sample_rate_verified
bool anatomical_accuracy_verified
bool yaw_drift_corrected
uint64 producer_dropped_total
# Compact JSON diagnostics/algorithm/config metadata, not IMU values or MANO assets.
string quality_json
string metadata_json
# Bridge diagnostics never replace input stamps above.
uint64 bridge_received_realtime_ns
uint64 bridge_received_monotonic_ns
uint64 bridge_sequence_gaps_total
uint64 bridge_rejected_total
