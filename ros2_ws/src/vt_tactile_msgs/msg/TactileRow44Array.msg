# Loss-aware decoded measurements; no raw serial bytes, no IMU data.
# All blocks in a message completed in the same host read, in wire order.
# Equal receive times within a batch do NOT mean simultaneous hardware sampling.
# Schema 2: native 44-byte rows, each with 20 ADC values; row range 0..19.
std_msgs/Header header
uint16 schema_version
string hand
string session_id
uint32 stream_epoch
# Software batch sequence (including status events), not a hardware sequence.
uint64 sequence
# Software index of the first decoded packet; indices include queue-dropped packets.
uint64 first_packet_index
# header.stamp == host_realtime_ns (read completion, NOT publish time).
uint64 host_realtime_ns
uint64 host_monotonic_ns
string clock_domain
string timing_source
bool connected
bool time_valid
string reason
# Batches (including empty events) dropped by the bounded producer queue.
uint64 producer_dropped_total
uint64 discarded_bytes_total
uint64 malformed_packets_total
# Empty for connection/fault events; never interpreted as a zero-touch sample.
TactileRow44[] blocks
