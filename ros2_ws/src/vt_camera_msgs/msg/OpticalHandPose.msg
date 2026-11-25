# One result per processed cube observation. header is the original colour image.
# Both wrist pose and the 21 joint positions are in this colour optical frame.
std_msgs/Header header
uint16 schema_version
string camera_name
string camera_serial
string hand
string status
bool wrist_valid
bool joints_valid
geometry_msgs/Pose wrist
# Empty unless joints_valid; 21 ordered XYZ triples in metres.
float64[] joint_positions_m
string cube_instance_id
uint64 cube_source_image_index
string layout_sha256
string intrinsics_sha256
string mount_sha256
# The common host receive/callback clock is CLOCK_REALTIME, not sensor exposure.
uint64 color_host_realtime_ns
uint64 color_frame_number
# Joint shape is from one real GloveState, nearest within a valid bracket.
string glove_session_id
string glove_config_sha256
string glove_model_rest_sha256
uint32 glove_calibration_version
uint64 glove_frame_index
uint64 glove_host_realtime_ns
int64 glove_minus_camera_ns
uint64 glove_before_ns
uint64 glove_after_ns
