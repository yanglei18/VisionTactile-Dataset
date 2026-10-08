# Derived rolling snapshot, NOT a simultaneous scan and NOT the archival source.
# Schema 2: physical 20x20 grid, layout row20_col_group_stride8_v2.
# Schema 1 / 80x80 recordings are legacy wire-address views; crop [:20, :20].
std_msgs/Header header
uint16 schema_version
string hand
string session_id
uint32 stream_epoch
uint64 source_sequence
# Last contributing read's clocks. Never advance these for a stale display.
uint64 host_realtime_ns
uint64 host_monotonic_ns
# Snapshot computation clock; only for age/health, not acquisition time.
uint64 snapshot_monotonic_ns
string clock_domain
string timing_source
string layout_id
string unit
uint16 rows
uint16 cols
bool connected
bool time_valid
string reason
# Row-major arrays of rows*cols elements. Missing ADC placeholders are zero;
# valid_mask determines usability. Zero ADC is itself a valid measurement.
# Schema 2: 400 values per array, index = row*20 + col (0 <= row,col < 20).
uint16[] adc
bool[] valid_mask
bool[] updated_mask
# rows*8 block clocks; index = row*8 + column_group. Zero means unobserved.
# Schema 2: 160 block clocks. Samples' raw packet/address format is unchanged.
uint64[] block_realtime_ns
uint64[] block_monotonic_ns
uint64 max_age_ns
uint64 producer_dropped_total
