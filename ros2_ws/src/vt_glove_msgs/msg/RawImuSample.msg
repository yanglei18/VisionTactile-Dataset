# One decoded node observation, before guard substitution. No SI-unit claim.
uint8 node_id
# Same session-wide node index and filter epoch referenced by GloveState.
uint64 sample_index
uint32 filter_epoch
# Original integer host receive clocks, never reconstructed from float seconds.
# Zero means unavailable in legacy replay; time_valid is false in that case.
uint64 host_realtime_ns
uint64 host_monotonic_ns
bool time_valid
# Wire int16 values: gx, gy, gz, ax, ay, az. Preserve rejected measurements.
int16[6] raw_values
# Uninterpreted wire byte, not a verified checksum or hardware clock.
uint8 status
bool status_known
bool gyro_rejected
bool accel_rejected
# Whether this observation advanced the fusion filter (possibly using guarded inputs).
bool fusion_updated
