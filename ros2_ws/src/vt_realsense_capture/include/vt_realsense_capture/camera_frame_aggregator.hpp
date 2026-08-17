#pragma once

#include "vt_realsense_capture/metadata_parser.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <vt_camera_msgs/msg/camera_frame_timing.hpp>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace vt_realsense_capture
{

enum class StreamKind : std::uint8_t {Color, Depth};

enum class AggregationKind : std::uint8_t
{
  Complete,
  Incomplete,
  Duplicate,
  Invalid,
  DomainMismatch,
};

struct GroupCameraIdentity
{
  std::string name;
  std::string model;
  std::string serial;
};

struct TimingObservation
{
  StreamKind stream;
  builtin_interfaces::msg::Time stamp;
  std::string source_frame_id;
  ParsedMetadata parsed;
  std::uint64_t host_monotonic_raw_ns;
  std::int64_t host_realtime_ns;
  std::uint32_t validity_flags;
};

struct AggregationEvent
{
  AggregationKind kind;
  std::int64_t shared_stamp_ns;
  std::optional<vt_camera_msgs::msg::CameraFrameTiming> timing;
  std::string detail;
};

class CameraFrameAggregator
{
public:
  CameraFrameAggregator(
    GroupCameraIdentity identity,
    std::uint64_t max_wait_ns,
    std::size_t max_newer_stamps);

  std::vector<AggregationEvent> push(TimingObservation observation);
  std::vector<AggregationEvent> flush(std::uint64_t now_host_ns);

private:
  struct PendingKey
  {
    std::optional<TimingObservation> color;
    std::optional<TimingObservation> depth;
    std::uint64_t first_host_arrival_ns;
    bool invalid{false};
    bool duplicate{false};
  };

  static std::int64_t stamp_to_nanoseconds(
    const builtin_interfaces::msg::Time & stamp);
  static bool has_required_validity(const TimingObservation & observation);

  std::vector<AggregationEvent> finalize_ready(std::uint64_t now_host_ns);
  AggregationEvent finalize(std::int64_t stamp_ns, const PendingKey & pending) const;
  void remember_finalized(std::int64_t stamp_ns);
  bool was_finalized(std::int64_t stamp_ns) const;

  GroupCameraIdentity identity_;
  std::uint64_t max_wait_ns_;
  std::size_t max_newer_stamps_;
  std::map<std::int64_t, PendingKey> pending_;
  std::deque<std::int64_t> finalized_order_;
  std::set<std::int64_t> finalized_recent_;
  std::size_t finalized_history_capacity_;
  std::optional<std::int64_t> closed_through_stamp_;
};

}  // namespace vt_realsense_capture
