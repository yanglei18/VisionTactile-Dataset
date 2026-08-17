#include "vt_realsense_capture/camera_frame_aggregator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using CameraFrameTiming = vt_camera_msgs::msg::CameraFrameTiming;
using vt_realsense_capture::AggregationEvent;
using vt_realsense_capture::AggregationKind;
using vt_realsense_capture::CameraFrameAggregator;
using vt_realsense_capture::GroupCameraIdentity;
using vt_realsense_capture::ParsedMetadata;
using vt_realsense_capture::StreamKind;
using vt_realsense_capture::TimestampDomain;
using vt_realsense_capture::TimingObservation;

constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
constexpr std::uint32_t kRequiredValidityFlags =
  CameraFrameTiming::VALID_FRAME_NUMBER |
  CameraFrameTiming::VALID_DEVICE_TIMESTAMP |
  CameraFrameTiming::VALID_CLOCK_DOMAIN |
  CameraFrameTiming::VALID_HOST_MONOTONIC |
  CameraFrameTiming::VALID_HOST_REALTIME;

GroupCameraIdentity identity()
{
  return {"d405_1", "D405", "260322278433"};
}

builtin_interfaces::msg::Time stamp_from_nanoseconds(std::int64_t stamp_ns)
{
  auto seconds = stamp_ns / kNanosecondsPerSecond;
  auto nanoseconds = stamp_ns % kNanosecondsPerSecond;
  if (nanoseconds < 0) {
    --seconds;
    nanoseconds += kNanosecondsPerSecond;
  }

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(seconds);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds);
  return stamp;
}

TimingObservation observation(
  StreamKind stream,
  std::int64_t stamp_ns,
  std::uint64_t frame_number,
  std::uint64_t host_ns)
{
  return TimingObservation{
    stream,
    stamp_from_nanoseconds(stamp_ns),
    stream == StreamKind::Color ?
    "d405_1_color_optical_frame" : "d405_1_depth_optical_frame",
    ParsedMetadata{
      frame_number,
      TimestampDomain::HardwareClock,
      static_cast<std::int64_t>(frame_number) * 33'333'333LL,
      std::nullopt,
      std::nullopt,
      std::nullopt,
    },
    host_ns,
    static_cast<std::int64_t>(host_ns) + kNanosecondsPerSecond,
    kRequiredValidityFlags,
  };
}

void append(
  std::vector<AggregationEvent> & destination,
  std::vector<AggregationEvent> source)
{
  destination.insert(
    destination.end(),
    std::make_move_iterator(source.begin()),
    std::make_move_iterator(source.end()));
}

std::size_t count_kind(
  const std::vector<AggregationEvent> & events,
  AggregationKind kind)
{
  return static_cast<std::size_t>(std::count_if(
    events.begin(), events.end(),
    [kind](const AggregationEvent & event) {return event.kind == kind;}));
}

std::size_t count_complete(const std::vector<AggregationEvent> & events)
{
  return count_kind(events, AggregationKind::Complete);
}

std::size_t count_incomplete(const std::vector<AggregationEvent> & events)
{
  return count_kind(events, AggregationKind::Incomplete);
}

const AggregationEvent & first_complete(
  const std::vector<AggregationEvent> & events)
{
  const auto complete_count = count_complete(events);
  if (complete_count != 1U) {
    throw std::runtime_error("expected exactly one complete aggregation event");
  }

  const auto complete = std::find_if(
    events.begin(), events.end(),
    [](const AggregationEvent & event) {
      return event.kind == AggregationKind::Complete;
    });
  EXPECT_TRUE(complete->timing.has_value());
  return *complete;
}

const AggregationEvent & only_event(
  const std::vector<AggregationEvent> & events)
{
  if (events.size() != 1U) {
    throw std::runtime_error("expected exactly one aggregation event");
  }
  return events.front();
}

void expect_no_timing(const std::vector<AggregationEvent> & events)
{
  for (const auto & event : events) {
    if (event.kind != AggregationKind::Complete) {
      EXPECT_FALSE(event.timing.has_value());
    }
  }
}

TEST(CameraFrameAggregator, EmitsOneGroupAfterExactStampWatermark)
{
  CameraFrameAggregator aggregate({"d405_1", "D405", "260322278433"},
                                  150000000ULL, 4U);
  EXPECT_TRUE(aggregate.push(
    observation(StreamKind::Color, 1000000000LL, 10U, 2000U)).empty());
  EXPECT_TRUE(aggregate.push(
    observation(StreamKind::Depth, 1000000000LL, 20U, 2600U)).empty());
  for (int index = 1; index <= 3; ++index) {
    const auto stamp = 1000000000LL + index * 33333333LL;
    EXPECT_TRUE(aggregate.push(
      observation(
        StreamKind::Color, stamp, 10U + index, 3000U + index)).empty());
    EXPECT_TRUE(aggregate.push(
      observation(
        StreamKind::Depth, stamp, 20U + index, 3600U + index)).empty());
  }

  constexpr auto fourth_stamp = 1000000000LL + 4 * 33333333LL;
  const auto events = aggregate.push(
    observation(StreamKind::Color, fourth_stamp, 14U, 3004U));
  const auto & group = first_complete(events).timing.value();
  EXPECT_EQ(group.shared_ros_timestamp_ns, 1000000000LL);
  EXPECT_EQ(group.color_frame_number, 10U);
  EXPECT_EQ(group.depth_frame_number, 20U);
  EXPECT_EQ(group.group_host_monotonic_raw_ns, 2000U);
  EXPECT_EQ(group.host_callback_spread_ns, 600U);
  EXPECT_TRUE(aggregate.push(
    observation(StreamKind::Depth, fourth_stamp, 24U, 3604U)).empty());
}

TEST(CameraFrameAggregator, ClosesPendingPrefixThroughHighestReadyStamp)
{
  CameraFrameAggregator aggregate(identity(), 10ULL, 4U);
  aggregate.push(observation(StreamKind::Color, 20LL, 20U, 100U));
  aggregate.push(observation(StreamKind::Depth, 20LL, 30U, 101U));
  aggregate.push(observation(StreamKind::Color, 10LL, 10U, 105U));

  auto closure_events = aggregate.flush(110U);
  EXPECT_EQ(closure_events.size(), 2U);
  if (closure_events.size() == 2U) {
    EXPECT_EQ(closure_events[0].shared_stamp_ns, 10LL);
    EXPECT_EQ(closure_events[0].kind, AggregationKind::Incomplete);
    EXPECT_FALSE(closure_events[0].timing.has_value());
    EXPECT_EQ(closure_events[1].shared_stamp_ns, 20LL);
    EXPECT_EQ(closure_events[1].kind, AggregationKind::Complete);
    EXPECT_TRUE(closure_events[1].timing.has_value());
  }

  auto late_events = aggregate.push(
    observation(StreamKind::Depth, 10LL, 11U, 111U));
  EXPECT_EQ(late_events.size(), 1U);
  if (late_events.size() == 1U) {
    EXPECT_EQ(late_events[0].shared_stamp_ns, 10LL);
    EXPECT_EQ(late_events[0].kind, AggregationKind::Duplicate);
    EXPECT_FALSE(late_events[0].timing.has_value());
  }
  const auto final_events = aggregate.flush(115U);
  EXPECT_TRUE(final_events.empty());

  std::vector<AggregationEvent> all_events;
  append(all_events, std::move(closure_events));
  append(all_events, std::move(late_events));
  append(all_events, final_events);
  EXPECT_EQ(count_complete(all_events), 1U);
  EXPECT_EQ(count_incomplete(all_events), 1U);
  EXPECT_EQ(count_kind(all_events, AggregationKind::Duplicate), 1U);
  ASSERT_EQ(count_complete(all_events), 1U);
  EXPECT_EQ(first_complete(all_events).shared_stamp_ns, 20LL);
  expect_no_timing(all_events);
}

TEST(CameraFrameAggregator, NeverNearestPairsDifferentStamps)
{
  CameraFrameAggregator aggregate(identity(), 150000000ULL, 4U);
  aggregate.push(observation(StreamKind::Color, 1000000000LL, 1U, 100U));
  aggregate.push(observation(StreamKind::Depth, 1000000001LL, 1U, 101U));
  const auto events = aggregate.flush(1000000000ULL);
  EXPECT_EQ(count_complete(events), 0U);
  EXPECT_EQ(count_incomplete(events), 2U);
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].shared_stamp_ns, 1000000000LL);
  EXPECT_EQ(events[1].shared_stamp_ns, 1000000001LL);
  expect_no_timing(events);
}

TEST(CameraFrameAggregator, PreservesAuditFieldsWhenDepthCallbackArrivesFirst)
{
  CameraFrameAggregator aggregate(identity(), 150000000ULL, 4U);
  auto depth = observation(StreamKind::Depth, 1000000042LL, 20U, 2600U);
  depth.parsed.sensor_timestamp_ns = 7001LL;
  depth.parsed.backend_timestamp_ns = 7002LL;
  depth.validity_flags |=
    CameraFrameTiming::VALID_SENSOR_TIMESTAMP |
    CameraFrameTiming::VALID_BACKEND_TIMESTAMP;
  auto color = observation(StreamKind::Color, 1000000042LL, 10U, 2000U);
  color.parsed.sensor_timestamp_ns = 6001LL;
  color.parsed.backend_timestamp_ns = 6002LL;
  color.validity_flags |=
    CameraFrameTiming::VALID_SENSOR_TIMESTAMP |
    CameraFrameTiming::VALID_BACKEND_TIMESTAMP;

  aggregate.push(std::move(depth));
  aggregate.push(std::move(color));
  const auto events = aggregate.flush(150002600ULL);

  ASSERT_EQ(count_complete(events), 1U);
  const auto & group = first_complete(events).timing.value();
  EXPECT_EQ(group.header.stamp, stamp_from_nanoseconds(1000000042LL));
  EXPECT_EQ(group.header.frame_id, "d405_1_frame_group");
  EXPECT_EQ(group.camera_name, "d405_1");
  EXPECT_EQ(group.camera_model, "D405");
  EXPECT_EQ(group.serial_number, "260322278433");
  EXPECT_EQ(group.shared_ros_timestamp_ns, 1000000042LL);
  EXPECT_EQ(group.color_frame_number, 10U);
  EXPECT_EQ(group.depth_frame_number, 20U);
  EXPECT_EQ(
    group.color_timestamp_domain,
    CameraFrameTiming::DOMAIN_HARDWARE_CLOCK);
  EXPECT_EQ(
    group.depth_timestamp_domain,
    CameraFrameTiming::DOMAIN_HARDWARE_CLOCK);
  EXPECT_EQ(group.color_device_timestamp_ns, 333333330LL);
  EXPECT_EQ(group.depth_device_timestamp_ns, 666666660LL);
  EXPECT_EQ(group.color_sensor_timestamp_ns, 6001LL);
  EXPECT_EQ(group.depth_sensor_timestamp_ns, 7001LL);
  EXPECT_EQ(group.color_backend_timestamp_ns, 6002LL);
  EXPECT_EQ(group.depth_backend_timestamp_ns, 7002LL);
  EXPECT_EQ(group.color_host_monotonic_raw_ns, 2000U);
  EXPECT_EQ(group.depth_host_monotonic_raw_ns, 2600U);
  EXPECT_EQ(group.color_host_realtime_ns, 1000002000LL);
  EXPECT_EQ(group.depth_host_realtime_ns, 1000002600LL);
  EXPECT_EQ(group.group_host_monotonic_raw_ns, 2000U);
  EXPECT_EQ(group.group_host_realtime_ns, 1000002000LL);
  EXPECT_EQ(group.host_callback_spread_ns, 600U);
  EXPECT_EQ(
    group.color_validity_flags,
    kRequiredValidityFlags |
    CameraFrameTiming::VALID_SENSOR_TIMESTAMP |
    CameraFrameTiming::VALID_BACKEND_TIMESTAMP);
  EXPECT_EQ(
    group.depth_validity_flags,
    kRequiredValidityFlags |
    CameraFrameTiming::VALID_SENSOR_TIMESTAMP |
    CameraFrameTiming::VALID_BACKEND_TIMESTAMP);
  EXPECT_EQ(
    group.group_validity_flags,
    CameraFrameTiming::GROUP_VALID_COMMON_STAMP |
    CameraFrameTiming::GROUP_VALID_IDENTITY |
    CameraFrameTiming::GROUP_VALID_DOMAINS |
    CameraFrameTiming::GROUP_VALID_CALLBACK_CLOCKS |
    CameraFrameTiming::GROUP_VALID_UNIQUE);
}

TEST(CameraFrameAggregator, RejectsSameStreamDuplicateBeforeWatermark)
{
  CameraFrameAggregator aggregate(identity(), 150000000ULL, 4U);
  EXPECT_TRUE(aggregate.push(
    observation(StreamKind::Color, 1000000000LL, 10U, 100U)).empty());
  EXPECT_TRUE(aggregate.push(
    observation(StreamKind::Color, 1000000000LL, 11U, 101U)).empty());
  EXPECT_TRUE(aggregate.push(
    observation(StreamKind::Depth, 1000000000LL, 20U, 102U)).empty());

  const auto events = aggregate.flush(150000100ULL);

  const auto & event = only_event(events);
  EXPECT_EQ(event.kind, AggregationKind::Duplicate);
  EXPECT_EQ(event.shared_stamp_ns, 1000000000LL);
  EXPECT_FALSE(event.detail.empty());
  EXPECT_EQ(count_complete(events), 0U);
  expect_no_timing(events);
}

TEST(CameraFrameAggregator, DetectsDuplicateAfterEmitWithoutRepublishing)
{
  CameraFrameAggregator aggregate(identity(), 10ULL, 4U);
  std::vector<AggregationEvent> events;
  append(events, aggregate.push(
    observation(StreamKind::Color, 1000000000LL, 10U, 100U)));
  append(events, aggregate.push(
    observation(StreamKind::Depth, 1000000000LL, 20U, 101U)));
  append(events, aggregate.flush(110U));
  append(events, aggregate.push(
    observation(StreamKind::Color, 1000000000LL, 10U, 200U)));

  EXPECT_EQ(count_complete(events), 1U);
  EXPECT_EQ(count_kind(events, AggregationKind::Duplicate), 1U);
  const auto duplicate = std::find_if(
    events.begin(), events.end(),
    [](const AggregationEvent & event) {
      return event.kind == AggregationKind::Duplicate;
    });
  ASSERT_NE(duplicate, events.end());
  EXPECT_EQ(duplicate->shared_stamp_ns, 1000000000LL);
  EXPECT_FALSE(duplicate->timing.has_value());
  EXPECT_FALSE(duplicate->detail.empty());
}

TEST(CameraFrameAggregator, RejectsParseInvalidFallbackObservation)
{
  CameraFrameAggregator aggregate(identity(), 150000000ULL, 4U);
  auto invalid = observation(StreamKind::Color, 1000000000LL, 10U, 100U);
  invalid.parsed = ParsedMetadata{
    0U,
    TimestampDomain::Unknown,
    0LL,
    std::nullopt,
    std::nullopt,
    std::nullopt,
  };
  invalid.validity_flags =
    CameraFrameTiming::VALID_HOST_MONOTONIC |
    CameraFrameTiming::VALID_HOST_REALTIME;
  aggregate.push(std::move(invalid));
  aggregate.push(observation(StreamKind::Depth, 1000000000LL, 20U, 101U));

  const auto events = aggregate.flush(150000100ULL);

  const auto & event = only_event(events);
  EXPECT_EQ(event.kind, AggregationKind::Invalid);
  EXPECT_FALSE(event.detail.empty());
  EXPECT_EQ(count_complete(events), 0U);
  expect_no_timing(events);
}

TEST(CameraFrameAggregator, RejectsKnownTimestampDomainMismatch)
{
  CameraFrameAggregator aggregate(identity(), 150000000ULL, 4U);
  auto depth = observation(StreamKind::Depth, 1000000000LL, 20U, 101U);
  depth.parsed.clock_domain = TimestampDomain::GlobalTime;
  aggregate.push(observation(StreamKind::Color, 1000000000LL, 10U, 100U));
  aggregate.push(std::move(depth));

  const auto events = aggregate.flush(150000100ULL);

  const auto & event = only_event(events);
  EXPECT_EQ(event.kind, AggregationKind::DomainMismatch);
  EXPECT_FALSE(event.detail.empty());
  EXPECT_EQ(count_complete(events), 0U);
  expect_no_timing(events);
}

TEST(CameraFrameAggregator, FinalizesAtExactlyOneHundredFiftyMilliseconds)
{
  CameraFrameAggregator aggregate(identity(), 150000000ULL, 4U);
  aggregate.push(observation(StreamKind::Color, 1000000000LL, 10U, 100U));

  EXPECT_TRUE(aggregate.flush(150000099ULL).empty());
  const auto events = aggregate.flush(150000100ULL);

  const auto & event = only_event(events);
  EXPECT_EQ(event.kind, AggregationKind::Incomplete);
  EXPECT_EQ(event.shared_stamp_ns, 1000000000LL);
  EXPECT_FALSE(event.detail.empty());
  expect_no_timing(events);
}

TEST(CameraFrameAggregator, PublishesACompletedStampExactlyOnce)
{
  CameraFrameAggregator aggregate(identity(), 10ULL, 4U);
  std::vector<AggregationEvent> events;
  append(events, aggregate.push(
    observation(StreamKind::Color, 1000000000LL, 10U, 100U)));
  append(events, aggregate.push(
    observation(StreamKind::Depth, 1000000000LL, 20U, 101U)));
  append(events, aggregate.flush(110U));
  append(events, aggregate.flush(1000U));
  append(events, aggregate.push(
    observation(StreamKind::Depth, 1000000000LL, 20U, 1001U)));
  append(events, aggregate.push(
    observation(StreamKind::Color, 1000000000LL, 10U, 1002U)));
  append(events, aggregate.flush(2000U));

  EXPECT_EQ(count_complete(events), 1U);
  EXPECT_EQ(count_kind(events, AggregationKind::Duplicate), 2U);
  EXPECT_EQ(first_complete(events).shared_stamp_ns, 1000000000LL);
  expect_no_timing(events);
}

}  // namespace
