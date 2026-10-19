uint8 DISCONNECTED=0
uint8 CONNECTED_NO_TRACKING=1
uint8 TRACKING=2
uint8 INVALID_DATA=3

std_msgs/Header header
string role
string tracker_id
uint8 state
uint8 tracking_status
uint64 valid_sample_count
uint64 invalid_report_count
uint64 dropped_queue_count
uint64 last_report_monotonic_ns
uint64 last_valid_pose_monotonic_ns
