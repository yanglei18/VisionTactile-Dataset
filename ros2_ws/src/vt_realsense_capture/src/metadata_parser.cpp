#include "vt_realsense_capture/metadata_parser.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace vt_realsense_capture
{
namespace
{

using Json = nlohmann::json;

constexpr std::int64_t kIntegerNanosecondsPerMillisecond = 1'000'000;
constexpr double kFloatingNanosecondsPerMillisecond = 1'000'000.0;
constexpr std::uint64_t kNanosecondsPerMicrosecond = 1'000;
constexpr double kInt64ExclusiveUpperBound = 9'223'372'036'854'775'808.0;

const Json & require_field(const Json & document, std::string_view name)
{
  const auto field = document.find(std::string(name));
  if (field == document.end()) {
    throw MetadataError("missing required metadata field: " + std::string(name));
  }
  return *field;
}

std::uint64_t parse_frame_number(const Json & value)
{
  if (!value.is_number_unsigned()) {
    throw MetadataError("frame_number must be an unsigned integer");
  }
  return value.get<std::uint64_t>();
}

std::string lowercase_ascii(std::string value)
{
  for (char & character : value) {
    if (character >= 'A' && character <= 'Z') {
      character = static_cast<char>(character + ('a' - 'A'));
    }
  }
  return value;
}

TimestampDomain parse_clock_domain(const Json & value)
{
  if (!value.is_string()) {
    throw MetadataError("clock_domain must be a string");
  }

  const auto domain = lowercase_ascii(value.get<std::string>());
  if (domain == "hardware_clock") {
    return TimestampDomain::HardwareClock;
  }
  if (domain == "system_time") {
    return TimestampDomain::SystemTime;
  }
  if (domain == "global_time") {
    return TimestampDomain::GlobalTime;
  }
  if (domain == "unknown") {
    return TimestampDomain::Unknown;
  }
  throw MetadataError("unrecognized clock_domain: " + domain);
}

std::int64_t milliseconds_to_nanoseconds(const Json & value)
{
  if (!value.is_number()) {
    throw MetadataError("frame_timestamp must be a number in milliseconds");
  }

  if (value.is_number_unsigned()) {
    const auto milliseconds = value.get<std::uint64_t>();
    constexpr auto kLargestWholeMillisecond =
      static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) /
      static_cast<std::uint64_t>(kIntegerNanosecondsPerMillisecond);
    if (milliseconds > kLargestWholeMillisecond) {
      throw MetadataError("frame_timestamp overflows nanoseconds");
    }
    return static_cast<std::int64_t>(milliseconds) * kIntegerNanosecondsPerMillisecond;
  }

  if (value.is_number_integer()) {
    const auto milliseconds = value.get<std::int64_t>();
    if (milliseconds < 0) {
      throw MetadataError("frame_timestamp must be non-negative and finite");
    }
    constexpr auto kLargestWholeMillisecond =
      std::numeric_limits<std::int64_t>::max() / kIntegerNanosecondsPerMillisecond;
    if (milliseconds > kLargestWholeMillisecond) {
      throw MetadataError("frame_timestamp overflows nanoseconds");
    }
    return milliseconds * kIntegerNanosecondsPerMillisecond;
  }

  const double milliseconds = value.get<double>();
  if (!std::isfinite(milliseconds) || std::signbit(milliseconds)) {
    throw MetadataError("frame_timestamp must be non-negative and finite");
  }
  const double nanoseconds = milliseconds * kFloatingNanosecondsPerMillisecond;
  if (!std::isfinite(nanoseconds) || nanoseconds >= kInt64ExclusiveUpperBound) {
    throw MetadataError("frame_timestamp overflows nanoseconds");
  }
  return static_cast<std::int64_t>(std::llround(nanoseconds));
}

std::optional<std::int64_t> optional_microseconds_to_nanoseconds(
  const Json & document, std::string_view name)
{
  const auto field = document.find(std::string(name));
  if (field == document.end()) {
    return std::nullopt;
  }
  if (!field->is_number_unsigned()) {
    throw MetadataError(std::string(name) + " must be an unsigned integer in microseconds");
  }

  const auto microseconds = field->get<std::uint64_t>();
  if (microseconds == 0) {
    return std::nullopt;
  }

  constexpr auto kLargestMicrosecond =
    static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) /
    kNanosecondsPerMicrosecond;
  if (microseconds > kLargestMicrosecond) {
    throw MetadataError(std::string(name) + " overflows nanoseconds");
  }
  return static_cast<std::int64_t>(microseconds) *
         static_cast<std::int64_t>(kNanosecondsPerMicrosecond);
}

}  // namespace

ParsedMetadata parse_metadata(std::string_view json_text)
{
  try {
    const auto document = Json::parse(json_text.begin(), json_text.end());
    if (!document.is_object()) {
      throw MetadataError("metadata JSON must be an object");
    }

    return ParsedMetadata{
      parse_frame_number(require_field(document, "frame_number")),
      parse_clock_domain(require_field(document, "clock_domain")),
      milliseconds_to_nanoseconds(require_field(document, "frame_timestamp")),
      optional_microseconds_to_nanoseconds(document, "sensor_timestamp"),
      optional_microseconds_to_nanoseconds(document, "backend_timestamp"),
      optional_microseconds_to_nanoseconds(document, "hw_timestamp"),
    };
  } catch (const MetadataError &) {
    throw;
  } catch (const nlohmann::json::exception & error) {
    throw MetadataError("invalid metadata JSON: " + std::string(error.what()));
  }
}

}  // namespace vt_realsense_capture
