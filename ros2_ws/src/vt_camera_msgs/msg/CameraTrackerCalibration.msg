# Versioned camera-to-Tracker calibration; translation is in metres.
# header.frame_id is the Tracker frame. tracker_from_camera is parent_from_child.
# This message does not claim ownership of any TF edge.
std_msgs/Header header
uint32 schema_version
string manager_instance_id
string camera_name
string camera_model
string camera_serial
string camera_frame_id
string tracker_role
string tracker_id
string world_frame_id
geometry_msgs/Transform tracker_from_camera
string revision
string sha256
uint64 activated_ns
# Live identity assertion. Consumers must also require now <= valid_until_ns.
bool valid
uint64 valid_until_ns
string reason
string quality_grade
