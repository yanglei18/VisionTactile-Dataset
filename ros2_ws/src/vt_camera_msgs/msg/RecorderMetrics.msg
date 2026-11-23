std_msgs/Header header
uint32 schema_version
string session_id
string bag_path
string recorder_instance_id
uint64 recorder_pid
# CLOCK_MONOTONIC on the recorder host; never a sensor timestamp.
uint64 observed_monotonic_ns
bool available
bool closed
bool rate_available
float64 storage_returned_mb_s
uint64 writer_errors
uint64 storage_errors
string storage_write_semantics
RecorderTopicMetrics[] topics
string detail
