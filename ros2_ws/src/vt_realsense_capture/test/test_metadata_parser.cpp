#include "vt_realsense_capture/metadata_parser.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using vt_realsense_capture::MetadataError;
using vt_realsense_capture::TimestampDomain;
using vt_realsense_capture::parse_metadata;

std::string read_fixture(const std::string & name)
{
  std::ifstream input(std::string(METADATA_FIXTURE_DIR) + "/" + name);
  if (!input) {
    throw std::runtime_error("could not open metadata fixture: " + name);
  }
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

TEST(MetadataParser, ConvertsDocumentedMillisecondsAndMetadataMicroseconds)
{
  const auto parsed = parse_metadata(read_fixture("metadata_hardware_clock.json"));
  EXPECT_EQ(parsed.frame_number, 30u);
  EXPECT_EQ(parsed.clock_domain, TimestampDomain::HardwareClock);
  EXPECT_EQ(parsed.device_timestamp_ns, 3566215572000LL);
  ASSERT_TRUE(parsed.sensor_timestamp_ns.has_value());
  EXPECT_EQ(*parsed.sensor_timestamp_ns, 3566199583000LL);
  EXPECT_FALSE(parsed.backend_timestamp_ns.has_value());
}

TEST(MetadataParser, RejectsMissingRequiredFields)
{
  EXPECT_THROW(parse_metadata(R"({"frame_number":1})"), MetadataError);
}

TEST(MetadataParser, RejectsNegativeOrNonFiniteTime)
{
  EXPECT_THROW(
    parse_metadata(
      R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":-1})"),
    MetadataError);
}

TEST(MetadataParser, RejectsMalformedJsonAndNonObjectDocuments)
{
  EXPECT_THROW(parse_metadata("{"), MetadataError);
  EXPECT_THROW(parse_metadata("[]"), MetadataError);
  EXPECT_THROW(parse_metadata("null"), MetadataError);
}

TEST(MetadataParser, RequiresEveryRequiredField)
{
  const std::vector<std::string> documents{
    R"({"clock_domain":"hardware_clock","frame_timestamp":1})",
    R"({"frame_number":1,"frame_timestamp":1})",
    R"({"frame_number":1,"clock_domain":"hardware_clock"})",
  };

  for (const auto & document : documents) {
    SCOPED_TRACE(document);
    EXPECT_THROW(parse_metadata(document), MetadataError);
  }
}

TEST(MetadataParser, RejectsWrongRequiredFieldTypesWithoutCoercion)
{
  const std::vector<std::string> documents{
    R"({"frame_number":-1,"clock_domain":"hardware_clock","frame_timestamp":1})",
    R"({"frame_number":1.0,"clock_domain":"hardware_clock","frame_timestamp":1})",
    R"({"frame_number":"1","clock_domain":"hardware_clock","frame_timestamp":1})",
    R"({"frame_number":true,"clock_domain":"hardware_clock","frame_timestamp":1})",
    R"({"frame_number":1,"clock_domain":1,"frame_timestamp":1})",
    R"({"frame_number":1,"clock_domain":null,"frame_timestamp":1})",
    R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":"1"})",
    R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":true})",
    R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":null})",
  };

  for (const auto & document : documents) {
    SCOPED_TRACE(document);
    EXPECT_THROW(parse_metadata(document), MetadataError);
  }
}

TEST(MetadataParser, PreservesMaximumUnsignedFrameNumberExactly)
{
  const auto parsed = parse_metadata(
    R"({"frame_number":18446744073709551615,"clock_domain":"hardware_clock","frame_timestamp":1})");
  EXPECT_EQ(parsed.frame_number, std::numeric_limits<std::uint64_t>::max());
}

TEST(MetadataParser, RejectsNonFiniteFrameTimestamp)
{
  EXPECT_THROW(
    parse_metadata(
      R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":1e400})"),
    MetadataError);
  EXPECT_THROW(
    parse_metadata(
      R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":NaN})"),
    MetadataError);
}

TEST(MetadataParser, NormalizesKnownDomainsCaseInsensitively)
{
  const std::vector<std::pair<std::string, TimestampDomain>> cases{
    {"HARDWARE_CLOCK", TimestampDomain::HardwareClock},
    {"System_Time", TimestampDomain::SystemTime},
    {"gLoBaL_tImE", TimestampDomain::GlobalTime},
  };

  for (const auto & [input, expected] : cases) {
    SCOPED_TRACE(input);
    const auto document =
      std::string(R"({"frame_number":1,"clock_domain":")") + input +
      R"(","frame_timestamp":1})";
    EXPECT_EQ(parse_metadata(document).clock_domain, expected);
  }
}

TEST(MetadataParser, MapsOnlyExplicitUnknownDomainToUnknown)
{
  const auto parsed = parse_metadata(
    R"({"frame_number":1,"clock_domain":"UnKnOwN","frame_timestamp":1})");
  EXPECT_EQ(parsed.clock_domain, TimestampDomain::Unknown);

  EXPECT_THROW(
    parse_metadata(
      R"({"frame_number":1,"clock_domain":"device_clock","frame_timestamp":1})"),
    MetadataError);
  EXPECT_THROW(
    parse_metadata(R"({"frame_number":1,"clock_domain":"","frame_timestamp":1})"),
    MetadataError);
}

TEST(MetadataParser, TreatsAbsentAndZeroOptionalTimestampsAsUnavailable)
{
  const auto absent = parse_metadata(
    R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":1})");
  EXPECT_FALSE(absent.hardware_timestamp_ns.has_value());
  EXPECT_FALSE(absent.sensor_timestamp_ns.has_value());
  EXPECT_FALSE(absent.backend_timestamp_ns.has_value());

  const auto zero = parse_metadata(
    R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":1,"hw_timestamp":0,"sensor_timestamp":0,"backend_timestamp":0})");
  EXPECT_FALSE(zero.hardware_timestamp_ns.has_value());
  EXPECT_FALSE(zero.sensor_timestamp_ns.has_value());
  EXPECT_FALSE(zero.backend_timestamp_ns.has_value());
}

TEST(MetadataParser, ConvertsEveryAvailableOptionalTimestampFromMicroseconds)
{
  const auto parsed = parse_metadata(
    R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":1,"hw_timestamp":2,"sensor_timestamp":3,"backend_timestamp":4})");
  ASSERT_TRUE(parsed.hardware_timestamp_ns.has_value());
  ASSERT_TRUE(parsed.sensor_timestamp_ns.has_value());
  ASSERT_TRUE(parsed.backend_timestamp_ns.has_value());
  EXPECT_EQ(*parsed.hardware_timestamp_ns, 2000LL);
  EXPECT_EQ(*parsed.sensor_timestamp_ns, 3000LL);
  EXPECT_EQ(*parsed.backend_timestamp_ns, 4000LL);
}

TEST(MetadataParser, RejectsWrongOptionalTimestampTypesWithoutCoercion)
{
  const std::vector<std::string> fields{
    R"("hw_timestamp":-1)",
    R"("sensor_timestamp":1.5)",
    R"("backend_timestamp":"1")",
    R"("hw_timestamp":true)",
    R"("sensor_timestamp":null)",
  };

  for (const auto & field : fields) {
    SCOPED_TRACE(field);
    const auto document =
      std::string(R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":1,)") +
      field + "}";
    EXPECT_THROW(parse_metadata(document), MetadataError);
  }
}

TEST(MetadataParser, RoundsMillisecondsToNearestNanosecond)
{
  const auto parsed = parse_metadata(
    R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":0.0000015})");
  EXPECT_EQ(parsed.device_timestamp_ns, 2LL);
}

TEST(MetadataParser, PreservesIntegerMillisecondTimestampExactly)
{
  const auto parsed = parse_metadata(
    R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":9223372036854})");
  EXPECT_EQ(parsed.device_timestamp_ns, 9223372036854000000LL);
}

TEST(MetadataParser, RejectsNegativeTimestampThatUnderflowsToZero)
{
  EXPECT_THROW(
    parse_metadata(
      R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":-1e-4000})"),
    MetadataError);
}

TEST(MetadataParser, RejectsExplicitNegativeZeroTimestamp)
{
  EXPECT_THROW(
    parse_metadata(
      R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":-0.0})"),
    MetadataError);
}

TEST(MetadataParser, RejectsMillisecondsThatOverflowNanoseconds)
{
  EXPECT_THROW(
    parse_metadata(
      R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":9223372036854.776})"),
    MetadataError);
}

TEST(MetadataParser, RejectsMicrosecondsThatOverflowNanoseconds)
{
  const std::vector<std::string> keys{
    "hw_timestamp", "sensor_timestamp", "backend_timestamp"};

  for (const auto & key : keys) {
    SCOPED_TRACE(key);
    const auto document =
      std::string(R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":1,")") +
      key + R"(":9223372036854776})";
    EXPECT_THROW(parse_metadata(document), MetadataError);
  }
}

TEST(MetadataParser, AcceptsLargestNonOverflowingMicrosecondTimestamp)
{
  const auto parsed = parse_metadata(
    R"({"frame_number":1,"clock_domain":"hardware_clock","frame_timestamp":1,"sensor_timestamp":9223372036854775})");
  ASSERT_TRUE(parsed.sensor_timestamp_ns.has_value());
  EXPECT_EQ(*parsed.sensor_timestamp_ns, 9223372036854775000LL);
}

}  // namespace
