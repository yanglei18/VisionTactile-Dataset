#include "vt_realsense_capture/camera_frame_aggregator.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace vt_realsense_capture
{
namespace
{

using CameraFrameTiming = vt_camera_msgs::msg::CameraFrameTiming;

constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
constexpr std::size_t kMinimumFinalizedHistory = 64U;
constexpr std::uint32_t kRequiredValidityFlags =
  CameraFrameTiming::VALID_FRAME_NUMBER |
  CameraFrameTiming::VALID_DEVICE_TIMESTAMP |
  CameraFrameTiming::VALID_CLOCK_DOMAIN |
  CameraFrameTiming::VALID_HOST_MONOTONIC |
  CameraFrameTiming::VALID_HOST_REALTIME;

std::uint8_t message_domain(TimestampDomain domain)
{
  switch (domain) {
    case TimestampDomain::HardwareClock:
      return CameraFrameTiming::DOMAIN_HARDWARE_CLOCK;
    case TimestampDomain::SystemTime:
      return CameraFrameTiming::DOMAIN_SYSTEM_TIME;
    case TimestampDomain::GlobalTime:
      return CameraFrameTiming::DOMAIN_GLOBAL_TIME;
    case TimestampDomain::Unknown:
      return CameraFrameTiming::DOMAIN_UNKNOWN;
  }
  return CameraFrameTiming::DOMAIN_UNKNOWN;
}

CameraFrameTiming build_timing(
  const GroupCameraIdentity & identity,
  std::int64_t stamp_ns,
  const TimingObservation & color,
  const TimingObservation & depth)
{
  CameraFrameTiming timing;
  timing.header.stamp = color.stamp;
  timing.header.frame_id = identity.name + "_frame_group";
  timing.camera_name = identity.name;
  timing.camera_model = identity.model;
  timing.serial_number = identity.serial;
  timing.shared_ros_timestamp_ns = stamp_ns;

  timing.color_frame_number = color.parsed.frame_number;
  timing.depth_frame_number = depth.parsed.frame_number;
  timing.color_timestamp_domain = message_domain(color.parsed.clock_domain);
  timing.depth_timestamp_domain = message_domain(depth.parsed.clock_domain);
  timing.color_device_timestamp_ns = color.parsed.device_timestamp_ns;
  timing.depth_device_timestamp_ns = depth.parsed.device_timestamp_ns;
  timing.color_sensor_timestamp_ns = color.parsed.sensor_timestamp_ns.value_or(0LL);
  timing.depth_sensor_timestamp_ns = depth.parsed.sensor_timestamp_ns.value_or(0LL);
  timing.color_backend_timestamp_ns = color.parsed.backend_timestamp_ns.value_or(0LL);
  timing.depth_backend_timestamp_ns = depth.parsed.backend_timestamp_ns.value_or(0LL);
  timing.color_host_monotonic_raw_ns = color.host_monotonic_raw_ns;
  timing.depth_host_monotonic_raw_ns = depth.host_monotonic_raw_ns;
  timing.color_host_realtime_ns = color.host_realtime_ns;
  timing.depth_host_realtime_ns = depth.host_realtime_ns;

  const auto & earlier =
    color.host_monotonic_raw_ns <= depth.host_monotonic_raw_ns ? color : depth;
  timing.group_host_monotonic_raw_ns = earlier.host_monotonic_raw_ns;
  timing.group_host_realtime_ns = earlier.host_realtime_ns;
  timing.host_callback_spread_ns =
    std::max(color.host_monotonic_raw_ns, depth.host_monotonic_raw_ns) -
    std::min(color.host_monotonic_raw_ns, depth.host_monotonic_raw_ns);

  timing.color_validity_flags = color.validity_flags;
  timing.depth_validity_flags = depth.validity_flags;
  timing.group_validity_flags =
    CameraFrameTiming::GROUP_VALID_COMMON_STAMP |
    CameraFrameTiming::GROUP_VALID_IDENTITY |
    CameraFrameTiming::GROUP_VALID_DOMAINS |
    CameraFrameTiming::GROUP_VALID_CALLBACK_CLOCKS |
    CameraFrameTiming::GROUP_VALID_UNIQUE;
  return timing;
}

void order_events(std::vector<AggregationEvent> & events)
{
  std::stable_sort(
    events.begin(), events.end(),
    [](const AggregationEvent & left, const AggregationEvent & right) {
      return left.shared_stamp_ns < right.shared_stamp_ns;
    });
}

}  // namespace

CameraFrameAggregator::CameraFrameAggregator(
  GroupCameraIdentity identity,
  std::uint64_t max_wait_ns,
  std::size_t max_newer_stamps)
: identity_(std::move(identity)),
  max_wait_ns_(max_wait_ns),
  max_newer_stamps_(max_newer_stamps),
  finalized_history_capacity_(
    std::max(kMinimumFinalizedHistory, max_newer_stamps + 1U))
{
}

std::int64_t CameraFrameAggregator::stamp_to_nanoseconds(
  const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(stamp.nanosec);
}

bool CameraFrameAggregator::has_required_validity(
  const TimingObservation & observation)
{
  return (observation.validity_flags & kRequiredValidityFlags) ==
         kRequiredValidityFlags;
}

std::vector<AggregationEvent> CameraFrameAggregator::push(
  TimingObservation observation)
{
  const auto stamp_ns = stamp_to_nanoseconds(observation.stamp);
  const auto now_host_ns = observation.host_monotonic_raw_ns;
  auto pending = pending_.find(stamp_ns);

  if (pending == pending_.end() && was_finalized(stamp_ns)) {
    auto events = finalize_ready(now_host_ns);
    events.push_back(AggregationEvent{
      AggregationKind::Duplicate,
      stamp_ns,
      std::nullopt,
      finalized_recent_.count(stamp_ns) != 0U ?
      "duplicate observation after stamp finalization" :
      "late observation after the closed watermark",
    });
    order_events(events);
    return events;
  }

  if (pending == pending_.end()) {
    pending = pending_.emplace(
      stamp_ns,
      PendingKey{std::nullopt, std::nullopt, now_host_ns, false, false}).first;
  }

  auto & key = pending->second;
  auto & stream = observation.stream == StreamKind::Color ? key.color : key.depth;
  if (stream.has_value()) {
    key.duplicate = true;
  } else {
    if (!has_required_validity(observation)) {
      key.invalid = true;
    }
    stream = std::move(observation);
  }

  return finalize_ready(now_host_ns);
}

std::vector<AggregationEvent> CameraFrameAggregator::flush(
  std::uint64_t now_host_ns)
{
  return finalize_ready(now_host_ns);
}

std::vector<AggregationEvent> CameraFrameAggregator::finalize_ready(
  std::uint64_t now_host_ns)
{
  std::optional<std::int64_t> closure_boundary;
  auto newer_stamps = pending_.size();
  for (const auto & [stamp_ns, pending] : pending_) {
    --newer_stamps;
    const bool old_enough =
      now_host_ns >= pending.first_host_arrival_ns &&
      now_host_ns - pending.first_host_arrival_ns >= max_wait_ns_;
    const bool watermark_reached = newer_stamps >= max_newer_stamps_;
    if (old_enough || watermark_reached) {
      closure_boundary = stamp_ns;
    }
  }

  std::vector<AggregationEvent> events;
  auto pending = pending_.begin();
  while (closure_boundary.has_value() && pending != pending_.end() &&
    pending->first <= *closure_boundary)
  {
    const auto stamp_ns = pending->first;
    events.push_back(finalize(stamp_ns, pending->second));
    remember_finalized(stamp_ns);
    pending = pending_.erase(pending);
  }
  return events;
}

AggregationEvent CameraFrameAggregator::finalize(
  std::int64_t stamp_ns,
  const PendingKey & pending) const
{
  if (pending.duplicate) {
    return {
      AggregationKind::Duplicate,
      stamp_ns,
      std::nullopt,
      "same stream observed more than once before finalization",
    };
  }
  if (pending.invalid) {
    return {
      AggregationKind::Invalid,
      stamp_ns,
      std::nullopt,
      "observation is missing required frame, device, domain, or host validity",
    };
  }
  if (!pending.color.has_value() || !pending.depth.has_value()) {
    return {
      AggregationKind::Incomplete,
      stamp_ns,
      std::nullopt,
      pending.color.has_value() ?
      "missing depth observation" : "missing color observation",
    };
  }

  const auto & color = *pending.color;
  const auto & depth = *pending.depth;
  if (color.parsed.clock_domain == TimestampDomain::Unknown ||
    depth.parsed.clock_domain == TimestampDomain::Unknown ||
    color.parsed.clock_domain != depth.parsed.clock_domain)
  {
    return {
      AggregationKind::DomainMismatch,
      stamp_ns,
      std::nullopt,
      "color and depth timestamp domains must be known and equal",
    };
  }

  return {
    AggregationKind::Complete,
    stamp_ns,
    build_timing(identity_, stamp_ns, color, depth),
    "exact color and depth metadata stamp",
  };
}

void CameraFrameAggregator::remember_finalized(std::int64_t stamp_ns)
{
  if (finalized_recent_.insert(stamp_ns).second) {
    finalized_order_.push_back(stamp_ns);
  }
  if (!closed_through_stamp_.has_value() || stamp_ns > *closed_through_stamp_) {
    closed_through_stamp_ = stamp_ns;
  }

  while (finalized_order_.size() > finalized_history_capacity_) {
    finalized_recent_.erase(finalized_order_.front());
    finalized_order_.pop_front();
  }
}

bool CameraFrameAggregator::was_finalized(std::int64_t stamp_ns) const
{
  return finalized_recent_.count(stamp_ns) != 0U ||
         (closed_through_stamp_.has_value() && stamp_ns <= *closed_through_stamp_);
}

}  // namespace vt_realsense_capture
