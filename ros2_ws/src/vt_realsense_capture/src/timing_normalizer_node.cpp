#include "vt_realsense_capture/camera_frame_aggregator.hpp"
#include "vt_realsense_capture/metadata_parser.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realsense2_camera_msgs/msg/metadata.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vt_camera_msgs/msg/camera_frame_timing.hpp>

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <time.h>

namespace vt_realsense_capture
{
namespace
{

using CameraFrameTiming = vt_camera_msgs::msg::CameraFrameTiming;
using Metadata = realsense2_camera_msgs::msg::Metadata;
using Trigger = std_srvs::srv::Trigger;

constexpr std::uint64_t kNanosecondsPerSecond = 1'000'000'000ULL;
constexpr std::int64_t kDefaultMaxWaitNs = 150'000'000LL;
constexpr std::int64_t kDefaultMaxNewerStamps = 4LL;

std::uint64_t monotonic_nanoseconds(const timespec & sample)
{
  return static_cast<std::uint64_t>(sample.tv_sec) * kNanosecondsPerSecond +
         static_cast<std::uint64_t>(sample.tv_nsec);
}

std::int64_t realtime_nanoseconds(const timespec & sample)
{
  return static_cast<std::int64_t>(sample.tv_sec) *
           static_cast<std::int64_t>(kNanosecondsPerSecond) +
         static_cast<std::int64_t>(sample.tv_nsec);
}

std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) *
           static_cast<std::int64_t>(kNanosecondsPerSecond) +
         static_cast<std::int64_t>(stamp.nanosec);
}

std::uint32_t validity_flags(const ParsedMetadata & parsed)
{
  std::uint32_t flags =
    CameraFrameTiming::VALID_FRAME_NUMBER |
    CameraFrameTiming::VALID_DEVICE_TIMESTAMP |
    CameraFrameTiming::VALID_HOST_MONOTONIC |
    CameraFrameTiming::VALID_HOST_REALTIME;
  if (parsed.clock_domain != TimestampDomain::Unknown) {
    flags |= CameraFrameTiming::VALID_CLOCK_DOMAIN;
  }
  if (parsed.sensor_timestamp_ns.has_value()) {
    flags |= CameraFrameTiming::VALID_SENSOR_TIMESTAMP;
  }
  if (parsed.backend_timestamp_ns.has_value()) {
    flags |= CameraFrameTiming::VALID_BACKEND_TIMESTAMP;
  }
  return flags;
}

void validate_identities(
  const std::vector<std::string> & camera_names,
  const std::vector<std::string> & camera_models,
  const std::vector<std::string> & serial_numbers)
{
  if (
    camera_names.empty() || camera_models.empty() || serial_numbers.empty() ||
    camera_names.size() != camera_models.size() ||
    camera_names.size() != serial_numbers.size())
  {
    throw std::invalid_argument(
            "camera_names, camera_models, and serial_numbers must be non-empty "
            "equal-length arrays");
  }

  std::unordered_set<std::string> unique_names;
  std::unordered_set<std::string> unique_serials;
  for (std::size_t index = 0; index < camera_names.size(); ++index) {
    if (
      camera_names[index].empty() || camera_models[index].empty() ||
      serial_numbers[index].empty())
    {
      throw std::invalid_argument("camera identity values must not be empty");
    }
    if (!unique_names.insert(camera_names[index]).second) {
      throw std::invalid_argument("duplicate camera name: " + camera_names[index]);
    }
    if (!unique_serials.insert(serial_numbers[index]).second) {
      throw std::invalid_argument("duplicate camera serial: " + serial_numbers[index]);
    }
  }
}

}  // namespace

class TimingNormalizerNode final : public rclcpp::Node
{
public:
  TimingNormalizerNode()
  : Node("timing_normalizer")
  {
    const auto camera_names = declare_parameter<std::vector<std::string>>(
      "camera_names", std::vector<std::string>{});
    const auto camera_models = declare_parameter<std::vector<std::string>>(
      "camera_models", std::vector<std::string>{});
    rcl_interfaces::msg::ParameterDescriptor serial_descriptor;
    serial_descriptor.dynamic_typing = true;
    declare_parameter(
      "serial_numbers", rclcpp::ParameterValue(std::vector<std::string>{}),
      serial_descriptor);
    const auto serial_parameter = get_parameter("serial_numbers");
    std::vector<std::string> serial_numbers;
    if (serial_parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      serial_numbers = serial_parameter.as_string_array();
    } else if (
      serial_parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
    {
      for (const auto serial : serial_parameter.as_integer_array()) {
        if (serial < 0) {
          throw std::invalid_argument("camera serial numbers must not be negative");
        }
        serial_numbers.push_back(std::to_string(serial));
      }
    } else {
      throw std::invalid_argument(
              "serial_numbers must be a string array or non-negative integer array");
    }

    validate_identities(camera_names, camera_models, serial_numbers);
    const auto max_wait_parameter = declare_parameter<std::int64_t>(
      "max_wait_ns", kDefaultMaxWaitNs);
    const auto max_newer_parameter = declare_parameter<std::int64_t>(
      "max_newer_stamps", kDefaultMaxNewerStamps);
    if (max_wait_parameter <= 0) {
      throw std::invalid_argument("max_wait_ns must be positive");
    }
    if (max_newer_parameter <= 0) {
      throw std::invalid_argument("max_newer_stamps must be positive");
    }
    if (
      static_cast<std::uint64_t>(max_newer_parameter) >
      std::numeric_limits<std::size_t>::max())
    {
      throw std::invalid_argument("max_newer_stamps exceeds platform size limits");
    }
    max_wait_ns_ = static_cast<std::uint64_t>(max_wait_parameter);
    max_newer_stamps_ = static_cast<std::size_t>(max_newer_parameter);

    cameras_.reserve(camera_names.size());
    subscriptions_.reserve(camera_names.size() * 2U);
    for (std::size_t index = 0; index < camera_names.size(); ++index) {
      create_camera(GroupCameraIdentity{
        camera_names[index], camera_models[index], serial_numbers[index]});
    }

    flush_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {expire_pending_groups();});
    flush_service_ = create_service<Trigger>(
      "/timing_normalizer/flush",
      [this](
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response)
      {
        flush_for_shutdown(*response);
      });
  }

private:
  struct CameraState
  {
    CameraState(
      GroupCameraIdentity camera_identity,
      std::uint64_t max_wait_ns,
      std::size_t max_newer_stamps,
      rclcpp::Publisher<CameraFrameTiming>::SharedPtr timing_publisher)
    : identity(std::move(camera_identity)),
      aggregator(identity, max_wait_ns, max_newer_stamps),
      publisher(std::move(timing_publisher))
    {
    }

    GroupCameraIdentity identity;
    CameraFrameAggregator aggregator;
    rclcpp::Publisher<CameraFrameTiming>::SharedPtr publisher;
  };

  void create_camera(GroupCameraIdentity identity)
  {
    auto publisher = create_publisher<CameraFrameTiming>(
      "/" + identity.name + "/frame_timing",
      rclcpp::SensorDataQoS().keep_last(10));
    auto camera = std::make_unique<CameraState>(
      std::move(identity), max_wait_ns_, max_newer_stamps_, std::move(publisher));
    auto * camera_state = camera.get();
    cameras_.push_back(std::move(camera));

    create_metadata_subscription(*camera_state, StreamKind::Color, "color");
    create_metadata_subscription(*camera_state, StreamKind::Depth, "depth");
  }

  void create_metadata_subscription(
    CameraState & camera,
    StreamKind stream,
    const std::string & stream_name)
  {
    const auto topic = "/" + camera.identity.name + "/" + stream_name + "/metadata";
    auto subscription = create_subscription<Metadata>(
      topic,
      rclcpp::SensorDataQoS(),
      [this, camera_state = &camera, stream, stream_name](
        const Metadata::ConstSharedPtr metadata)
      {
        timespec monotonic_sample{};
        timespec realtime_sample{};
        const int monotonic_result =
          ::clock_gettime(CLOCK_MONOTONIC_RAW, &monotonic_sample);
        const int monotonic_error = monotonic_result == 0 ? 0 : errno;
        const int realtime_result = ::clock_gettime(CLOCK_REALTIME, &realtime_sample);
        const int realtime_error = realtime_result == 0 ? 0 : errno;
        const auto stamp_ns = stamp_nanoseconds(metadata->header.stamp);

        if (monotonic_result != 0 || realtime_result != 0) {
          const auto stamp_text = std::to_string(stamp_ns);
          RCLCPP_ERROR(
            get_logger(),
            "Clock sample failed camera=%s stamp_ns=%s stream=%s "
            "CLOCK_MONOTONIC_RAW_errno=%d CLOCK_REALTIME_errno=%d",
            camera_state->identity.name.c_str(), stamp_text.c_str(),
            stream_name.c_str(), monotonic_error, realtime_error);
          return;
        }

        const auto host_monotonic_raw_ns = monotonic_nanoseconds(monotonic_sample);
        const auto host_realtime_ns = realtime_nanoseconds(realtime_sample);
        ParsedMetadata parsed{
          0U, TimestampDomain::Unknown, 0LL,
          std::nullopt, std::nullopt, std::nullopt};
        std::uint32_t flags =
          CameraFrameTiming::VALID_HOST_MONOTONIC |
          CameraFrameTiming::VALID_HOST_REALTIME;
        try {
          parsed = parse_metadata(metadata->json_data);
          flags = validity_flags(parsed);
        } catch (const MetadataError & error) {
          const auto stamp_text = std::to_string(stamp_ns);
          RCLCPP_ERROR(
            get_logger(),
            "Metadata parse failed camera=%s stamp_ns=%s stream=%s: %s",
            camera_state->identity.name.c_str(), stamp_text.c_str(),
            stream_name.c_str(), error.what());
        }

        publish_events(
          *camera_state,
          camera_state->aggregator.push(TimingObservation{
            stream,
            metadata->header.stamp,
            metadata->header.frame_id,
            std::move(parsed),
            host_monotonic_raw_ns,
            host_realtime_ns,
            flags}));
      });
    subscriptions_.push_back(std::move(subscription));
  }

  std::size_t publish_events(
    CameraState & camera,
    std::vector<AggregationEvent> events)
  {
    std::size_t published = 0U;
    for (auto & event : events) {
      const auto stamp_text = std::to_string(event.shared_stamp_ns);
      switch (event.kind) {
        case AggregationKind::Complete:
          if (!event.timing.has_value()) {
            RCLCPP_ERROR(
              get_logger(),
              "Complete timing event missing message camera=%s stamp_ns=%s",
              camera.identity.name.c_str(), stamp_text.c_str());
            break;
          }
          camera.publisher->publish(*event.timing);
          ++published;
          break;
        case AggregationKind::Incomplete:
          RCLCPP_WARN(
            get_logger(), "Incomplete timing group camera=%s stamp_ns=%s: %s",
            camera.identity.name.c_str(), stamp_text.c_str(), event.detail.c_str());
          break;
        case AggregationKind::Duplicate:
          RCLCPP_WARN(
            get_logger(), "Duplicate timing group camera=%s stamp_ns=%s: %s",
            camera.identity.name.c_str(), stamp_text.c_str(), event.detail.c_str());
          break;
        case AggregationKind::Invalid:
          RCLCPP_ERROR(
            get_logger(), "Invalid timing group camera=%s stamp_ns=%s: %s",
            camera.identity.name.c_str(), stamp_text.c_str(), event.detail.c_str());
          break;
        case AggregationKind::DomainMismatch:
          RCLCPP_ERROR(
            get_logger(), "Timing domain mismatch camera=%s stamp_ns=%s: %s",
            camera.identity.name.c_str(), stamp_text.c_str(), event.detail.c_str());
          break;
      }
    }
    return published;
  }

  bool sample_monotonic_now(std::uint64_t & now_ns, int & sample_error) const
  {
    timespec sample{};
    const int result = ::clock_gettime(CLOCK_MONOTONIC_RAW, &sample);
    sample_error = result == 0 ? 0 : errno;
    if (result != 0) {
      return false;
    }
    now_ns = monotonic_nanoseconds(sample);
    return true;
  }

  void expire_pending_groups()
  {
    std::uint64_t now_ns = 0U;
    int sample_error = 0;
    if (!sample_monotonic_now(now_ns, sample_error)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to sample CLOCK_MONOTONIC_RAW for timing flush errno=%d",
        sample_error);
      return;
    }
    for (auto & camera : cameras_) {
      publish_events(*camera, camera->aggregator.flush(now_ns));
    }
  }

  void flush_for_shutdown(Trigger::Response & response)
  {
    std::uint64_t now_ns = 0U;
    int sample_error = 0;
    if (!sample_monotonic_now(now_ns, sample_error)) {
      response.success = false;
      response.message =
        "CLOCK_MONOTONIC_RAW sample failed errno=" + std::to_string(sample_error);
      RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
      return;
    }

    const auto drain_now =
      now_ns > std::numeric_limits<std::uint64_t>::max() - max_wait_ns_ ?
      std::numeric_limits<std::uint64_t>::max() : now_ns + max_wait_ns_;
    std::size_t published = 0U;
    for (auto & camera : cameras_) {
      published += publish_events(*camera, camera->aggregator.flush(drain_now));
    }
    response.success = true;
    response.message =
      "published " + std::to_string(published) + " complete camera timing groups";
  }

  std::uint64_t max_wait_ns_{0U};
  std::size_t max_newer_stamps_{0U};
  std::vector<std::unique_ptr<CameraState>> cameras_;
  std::vector<rclcpp::Subscription<Metadata>::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
  rclcpp::Service<Trigger>::SharedPtr flush_service_;
};

}  // namespace vt_realsense_capture

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<vt_realsense_capture::TimingNormalizerNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("timing_normalizer"), "Timing normalizer failed: %s",
      error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
