# Raw acquisition is independent of calibration, pose solving and display rate.
# Header is the final sample's original host realtime stamp, zero if unavailable.
std_msgs/Header header
uint16 schema_version
string hand
string source_mode
string session_id
string config_sha256
# Starts at 1 per session; includes failed enqueue/send attempts.
uint64 batch_sequence
string clock_domain
string timing_source
RawImuSample[<=64] samples
# Cumulative for this producer process, including bounded-queue and send failures.
uint64 producer_dropped_batches_total
uint64 producer_dropped_samples_total
# Bridge receipt clocks never replace sample clocks.
uint64 bridge_received_realtime_ns
uint64 bridge_received_monotonic_ns
uint64 bridge_sequence_gaps_total
uint64 bridge_rejected_total
