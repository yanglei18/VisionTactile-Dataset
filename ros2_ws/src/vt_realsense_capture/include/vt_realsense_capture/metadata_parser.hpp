#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string_view>

namespace vt_realsense_capture
{

enum class TimestampDomain : std::uint8_t
{
  Unknown = 0,
  HardwareClock = 1,
  SystemTime = 2,
  GlobalTime = 3,
};

struct ParsedMetadata
{
  std::uint64_t frame_number;
  TimestampDomain clock_domain;
  std::int64_t device_timestamp_ns;
  std::optional<std::int64_t> sensor_timestamp_ns;
  std::optional<std::int64_t> backend_timestamp_ns;
  std::optional<std::int64_t> hardware_timestamp_ns;
};

class MetadataError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

ParsedMetadata parse_metadata(std::string_view json_text);

}  // namespace vt_realsense_capture
