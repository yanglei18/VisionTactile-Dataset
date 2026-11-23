# Actual recorder/writer path counters. Storage return is not an fsync guarantee.
string topic
uint64 writer_received_messages
uint64 writer_received_bytes
uint64 storage_returned_messages
uint64 storage_returned_bytes
uint64 storage_returned_last_monotonic_ns
