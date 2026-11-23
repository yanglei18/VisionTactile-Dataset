#pragma once

// Observe the official recorder and storage calls without adding a second DDS
// subscription. No message payload is retained by the metrics collector.
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_factory.hpp>

namespace vt_realsense_capture
{
using Message = std::shared_ptr<const rosbag2_storage::SerializedBagMessage>;
using Storage = rosbag2_storage::storage_interfaces::ReadWriteInterface;

inline uint64_t monotonic_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct TopicCounters
{
  uint64_t writer_messages = 0, writer_bytes = 0;
  uint64_t storage_messages = 0, storage_bytes = 0, storage_last_ns = 0;
};

class WriterMetrics
{
public:
  explicit WriterMetrics(const std::vector<std::string> & topics)
  {
    if (topics.empty() || topics.size() > 512) {
      throw std::invalid_argument("Recorder requires 1..512 explicit topics");
    }
    for (const auto & topic : topics) {
      if (!counters_.emplace(topic, TopicCounters{}).second) {
        throw std::invalid_argument("Duplicate recorder topic");
      }
    }
  }

  void received(const Message & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & count = counters_.at(message->topic_name);
    ++count.writer_messages;
    count.writer_bytes += message->serialized_data->buffer_length;
  }

  void stored(const std::vector<Message> & messages)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = monotonic_ns();
    for (const auto & message : messages) {
      auto & count = counters_.at(message->topic_name);
      ++count.storage_messages;
      count.storage_bytes += message->serialized_data->buffer_length;
      count.storage_last_ns = now;
    }
  }

  void error(bool storage)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++(storage ? storage_errors_ : writer_errors_);
  }

  nlohmann::json snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto topics = nlohmann::json::array();
    for (const auto & [name, count] : counters_) {
      topics.push_back({{"topic", name}, {"writer_received_messages", count.writer_messages},
        {"writer_received_bytes", count.writer_bytes},
        {"storage_returned_messages", count.storage_messages},
        {"storage_returned_bytes", count.storage_bytes},
        {"storage_returned_last_monotonic_ns", count.storage_last_ns}});
    }
    return {{"topics", topics}, {"writer_errors", writer_errors_},
      {"storage_errors", storage_errors_}, {"observed_monotonic_ns", monotonic_ns()}};
  }

private:
  mutable std::mutex mutex_;
  std::map<std::string, TopicCounters> counters_;
  uint64_t writer_errors_ = 0, storage_errors_ = 0;
};

// Delegation preserves the installed MCAP plugin and the official cache. Count
// only normally returning writes; exceptions can include partially written
// batches, which deliberately remain unconfirmed until independent readback.
class ObservedStorage final : public Storage
{
public:
  ObservedStorage(std::shared_ptr<Storage> delegate, std::shared_ptr<WriterMetrics> metrics)
  : delegate_(std::move(delegate)), metrics_(std::move(metrics)) {}

  void open(const rosbag2_storage::StorageOptions & options,
    rosbag2_storage::storage_interfaces::IOFlag flag) override {delegate_->open(options, flag);}
  void write(Message message) override
  {
    try {delegate_->write(message);} catch (...) {metrics_->error(true); throw;}
    metrics_->stored({message});
  }
  void write(const std::vector<Message> & messages) override
  {
    try {delegate_->write(messages);} catch (...) {metrics_->error(true); throw;}
    metrics_->stored(messages);
  }
  void update_metadata(const rosbag2_storage::BagMetadata & metadata) override
  {delegate_->update_metadata(metadata);}
  void create_topic(const rosbag2_storage::TopicMetadata & topic,
    const rosbag2_storage::MessageDefinition & definition) override
  {delegate_->create_topic(topic, definition);}
  void remove_topic(const rosbag2_storage::TopicMetadata & topic) override
  {delegate_->remove_topic(topic);}
  rosbag2_storage::BagMetadata get_metadata() override {return delegate_->get_metadata();}
  std::string get_relative_file_path() const override {return delegate_->get_relative_file_path();}
  uint64_t get_bagfile_size() const override {return delegate_->get_bagfile_size();}
  std::string get_storage_identifier() const override {return delegate_->get_storage_identifier();}
  uint64_t get_minimum_split_file_size() const override
  {return delegate_->get_minimum_split_file_size();}
  bool set_read_order(const rosbag2_storage::ReadOrder & order) override
  {return delegate_->set_read_order(order);}
  bool has_next() override {return delegate_->has_next();}
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override
  {return delegate_->read_next();}
  std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() override
  {return delegate_->get_all_topics_and_types();}
  void get_all_message_definitions(std::vector<rosbag2_storage::MessageDefinition> & definitions) override
  {delegate_->get_all_message_definitions(definitions);}
  void set_filter(const rosbag2_storage::StorageFilter & filter) override {delegate_->set_filter(filter);}
  void reset_filter() override {delegate_->reset_filter();}
  void seek(const rcutils_time_point_value_t & time) override {delegate_->seek(time);}

private:
  std::shared_ptr<Storage> delegate_;
  std::shared_ptr<WriterMetrics> metrics_;
};

class ObservedStorageFactory final : public rosbag2_storage::StorageFactoryInterface
{
public:
  explicit ObservedStorageFactory(std::shared_ptr<WriterMetrics> metrics)
  : metrics_(std::move(metrics)) {}
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface>
  open_read_only(const rosbag2_storage::StorageOptions & options) override
  {return delegate_.open_read_only(options);}
  std::shared_ptr<Storage> open_read_write(const rosbag2_storage::StorageOptions & options) override
  {
    auto storage = delegate_.open_read_write(options);
    if (!storage) {return nullptr;}
    return std::make_shared<ObservedStorage>(std::move(storage), metrics_);
  }
private:
  rosbag2_storage::StorageFactory delegate_;
  std::shared_ptr<WriterMetrics> metrics_;
};

class ObservedWriter final : public rosbag2_cpp::writers::SequentialWriter
{
public:
  explicit ObservedWriter(std::shared_ptr<WriterMetrics> metrics)
  : SequentialWriter(std::make_unique<ObservedStorageFactory>(metrics)), metrics_(std::move(metrics)) {}
  void write(Message message) override
  {
    metrics_->received(message);
    try {SequentialWriter::write(std::move(message));} catch (...) {metrics_->error(false); throw;}
  }
private:
  std::shared_ptr<WriterMetrics> metrics_;
};
}  // namespace vt_realsense_capture
