# header matches the raw source image (stamp and color optical frame).
# A tracked pose describes the 30 mm cube centre in that frame.
# No wrist offset, world transform, or extrapolated hand pose is applied.
std_msgs/Header header
uint16 schema_version
string camera_name
string instance_id
string layout_sha256
string intrinsics_sha256
string hand
string status
bool valid
# Pose, center_px, tag_ids and reprojection_px are meaningful only if valid.
geometry_msgs/Pose pose
float64[2] center_px
uint8[] tag_ids
float64 reprojection_px
# Count of accepted cube candidates in this source frame, irrespective of binding.
uint32 visible_cube_count
# Local process sequence and dropped pending images; DDS and hardware loss are outside these counts.
uint64 source_image_index
uint64 skipped_image_total
