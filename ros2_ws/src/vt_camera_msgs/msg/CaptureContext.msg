# Recorder lifecycle facts, serialized with all control work.
# state uses the CaptureStatus constants. COMPLETE is not a quality verdict.
string controller_instance_id
uint64 state_revision
uint8 state
string session_id
string start_request_id
string session_label
string output_path
bool recorder_alive
float64 elapsed_sec
uint32 planned_duration_sec
string[] topics
uint32 max_duration_sec
string detail
