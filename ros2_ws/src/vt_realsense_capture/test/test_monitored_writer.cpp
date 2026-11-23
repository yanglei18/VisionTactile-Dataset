#include <filesystem>
#include <memory>
#include <unistd.h>
#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <std_msgs/msg/string.hpp>
#include "vt_realsense_capture/monitored_writer.hpp"
#include "vt_realsense_capture/recorder_qos.hpp"

namespace vt = vt_realsense_capture;
namespace fs = std::filesystem;

TEST(RecorderQos, PartialOverridesPreserveOfficialDefaultsAndTransientLocal)
{
  const auto profiles = vt::recording_qos(YAML::Load(
      "/image: {history: keep_last, depth: 64, reliability: reliable, durability: volatile}\n"
      "/extrinsics: {history: keep_last, depth: 1, reliability: reliable, durability: transient_local}\n"));
  const auto & image = profiles.at("/image").get_rmw_qos_profile();
  EXPECT_EQ(image.depth, 64U);
  EXPECT_EQ(image.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(image.liveliness, rmw_qos_profile_default.liveliness);
  EXPECT_EQ(image.liveliness_lease_duration.sec, rmw_qos_profile_default.liveliness_lease_duration.sec);
  EXPECT_EQ(image.liveliness_lease_duration.nsec, rmw_qos_profile_default.liveliness_lease_duration.nsec);
  EXPECT_EQ(image.deadline.sec, rmw_qos_profile_default.deadline.sec);
  EXPECT_EQ(image.deadline.nsec, rmw_qos_profile_default.deadline.nsec);
  EXPECT_EQ(profiles.at("/extrinsics").get_rmw_qos_profile().durability,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  EXPECT_THROW(vt::recording_qos(YAML::Load("/data: {depth: 10, typo: 1}")), std::invalid_argument);
}

class MonitoredWriterTest : public ::testing::Test
{
protected:
  fs::path directory;
  void SetUp() override
  {
    char pattern[] = "/tmp/vt-writer-test-XXXXXX";
    const auto path = ::mkdtemp(pattern);
    ASSERT_NE(path, nullptr);
    directory = path;
  }
  void TearDown() override {fs::remove_all(directory);}

  std::shared_ptr<rclcpp::SerializedMessage> message()
  {
    std_msgs::msg::String value;
    value.data = std::string(4096, 'a');
    auto serialized = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<std_msgs::msg::String> serializer;
    serializer.serialize_message(&value, serialized.get());
    return serialized;
  }
};

TEST_F(MonitoredWriterTest, CachedAndUncachedWritesMatchIndependentMcapReadback)
{
  for (const auto cache : {0ULL, 1024ULL * 1024ULL}) {
    auto metrics = std::make_shared<vt::WriterMetrics>(std::vector<std::string>{"/test/data"});
    rosbag2_cpp::Writer writer(std::make_unique<vt::ObservedWriter>(metrics));
    rosbag2_storage::StorageOptions options;
    options.uri = (directory / ("bag-" + std::to_string(cache))).string();
    options.storage_id = "mcap";
    options.max_cache_size = cache;
    writer.open(options);
    for (int i = 0; i < 100; ++i) {
      writer.write(message(), "/test/data", "std_msgs/msg/String", rclcpp::Time(i + 1));
      if (i == 49) {writer.split_bagfile();}
    }
    writer.close();
    rosbag2_cpp::Reader reader;
    reader.open(options);
    size_t count = 0, bytes = 0;
    while (reader.has_next()) {
      auto record = reader.read_next();
      ++count;
      bytes += record->serialized_data->buffer_length;
    }
    EXPECT_EQ(count, 100U);
    auto snapshot = metrics->snapshot();
    EXPECT_EQ(snapshot["writer_errors"], 0);
    EXPECT_EQ(snapshot["storage_errors"], 0);
    const auto & topic = snapshot["topics"][0];
    EXPECT_EQ(topic["writer_received_messages"], count);
    EXPECT_EQ(topic["storage_returned_messages"], count);
    EXPECT_EQ(topic["writer_received_bytes"], bytes);
    EXPECT_EQ(topic["storage_returned_bytes"], bytes);
    EXPECT_GT(topic["storage_returned_last_monotonic_ns"].get<uint64_t>(), 0U);
  }
}

TEST_F(MonitoredWriterTest, OfficialCacheRejectionIsNotReportedAsStored)
{
  auto metrics = std::make_shared<vt::WriterMetrics>(std::vector<std::string>{"/test/data"});
  rosbag2_cpp::Writer writer(std::make_unique<vt::ObservedWriter>(metrics));
  rosbag2_storage::StorageOptions options;
  options.uri = (directory / "small-cache").string();
  options.storage_id = "mcap";
  options.max_cache_size = 128;  // Each 4 KiB message is intentionally too large.
  writer.open(options);
  for (int i = 0; i < 1000; ++i) {
    writer.write(message(), "/test/data", "std_msgs/msg/String", rclcpp::Time(i + 1));
  }
  writer.close();
  auto topic = metrics->snapshot()["topics"][0];
  EXPECT_EQ(topic["writer_received_messages"], 1000);
  rosbag2_cpp::Reader reader;
  reader.open(options);
  size_t count = 0;
  while (reader.has_next()) {reader.read_next(); ++count;}
  // The official cache admits a message when an empty buffer is available even
  // if it exceeds the byte target. Do not assume all oversized writes are lost.
  EXPECT_LT(count, 1000U);
  EXPECT_EQ(topic["storage_returned_messages"], count);
}

TEST_F(MonitoredWriterTest, FailedWriterCallIsCountedWithoutAStorageSuccess)
{
  auto metrics = std::make_shared<vt::WriterMetrics>(std::vector<std::string>{"/test/data"});
  vt::ObservedWriter writer(metrics);
  auto record = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  record->topic_name = "/test/data";
  auto serialized = message();
  record->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
    serialized, &serialized->get_rcl_serialized_message());
  EXPECT_THROW(writer.write(record), std::runtime_error);
  auto snapshot = metrics->snapshot();
  EXPECT_EQ(snapshot["writer_errors"], 1);
  EXPECT_EQ(snapshot["topics"][0]["writer_received_messages"], 1);
  EXPECT_EQ(snapshot["topics"][0]["storage_returned_messages"], 0);
}
