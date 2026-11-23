// Use the official ROS recorder, QoS negotiation, cache and MCAP plugin. Only
// the writer/storage call boundaries are observed; preview traffic is unrelated.
#include <atomic>
#include <cerrno>
#include <csignal>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <fcntl.h>

#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/qos.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <yaml-cpp/yaml.h>

#include "vt_realsense_capture/monitored_writer.hpp"
#include "vt_realsense_capture/recorder_qos.hpp"

namespace fs = std::filesystem;
namespace vt = vt_realsense_capture;
namespace
{
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}

struct Options
{
  rosbag2_storage::StorageOptions storage;
  rosbag2_transport::RecordOptions record;
  std::string session_id, instance_id, metrics_file, node_name = "vt_rosbag_recorder";
};

uint64_t positive_integer(const std::string & text)
{
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument("Expected an unsigned integer");
  }
  return std::stoull(text);
}

Options parse(int argc, char ** argv)
{
  Options result;
  result.record.rmw_serialization_format = rmw_get_serialization_format();
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto value = [&]() -> std::string {
        if (++i >= argc) {throw std::invalid_argument("Missing value for " + key);}
        return argv[i];
      };
    if (key == "--topics") {
      while (++i < argc) {result.record.topics.emplace_back(argv[i]);}
    } else if (key == "--output") {result.storage.uri = value();
    } else if (key == "--storage") {result.storage.storage_id = value();
    } else if (key == "--storage-config-file") {result.storage.storage_config_uri = value();
    } else if (key == "--qos-profile-overrides-path") {
      result.record.topic_qos_profile_overrides = vt::recording_qos(YAML::LoadFile(value()));
    } else if (key == "--max-bag-duration") {
      result.storage.max_bagfile_duration = positive_integer(value());
    } else if (key == "--max-bag-size") {
      result.storage.max_bagfile_size = positive_integer(value());
    } else if (key == "--max-cache-size") {
      result.storage.max_cache_size = positive_integer(value());
    } else if (key == "--disable-keyboard-controls") {
      result.record.disable_keyboard_controls = true;
    } else if (key == "--include-unpublished-topics") {
      result.record.include_unpublished_topics = true;
    } else if (key == "--node-name") {result.node_name = value();
    } else if (key == "--session-id") {result.session_id = value();
    } else if (key == "--recorder-instance-id") {result.instance_id = value();
    } else if (key == "--metrics-file") {result.metrics_file = value();
    } else {throw std::invalid_argument("Unknown argument: " + key);}
  }
  const fs::path bag(result.storage.uri), metrics(result.metrics_file);
  if (result.session_id.empty() || result.instance_id.empty() || !bag.is_absolute() ||
    !metrics.is_absolute() || metrics.parent_path() != bag.parent_path() ||
    metrics.filename() != "recorder-metrics.json" || fs::exists(metrics) ||
    fs::is_symlink(metrics) || fs::exists(bag) || result.storage.storage_id != "mcap" ||
    result.record.topics.empty() || !result.record.disable_keyboard_controls)
  {
    throw std::invalid_argument("Invalid recorder session, output or metrics binding");
  }
  return result;
}

// Replace a small snapshot atomically. Do not fsync per update: these are
// observations, not a durability certificate. Interrupted writes leave the last
// complete snapshot. No file I/O takes place on the recorder callback path.
void atomic_snapshot(const fs::path & path, const nlohmann::json & data)
{
  const auto temporary = path.string() + ".tmp." + std::to_string(getpid());
  const auto text = data.dump() + "\n";
  const int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (fd < 0) {throw std::runtime_error("Cannot create recorder metrics: " + std::string(strerror(errno)));}
  bool open = true;
  try {
    size_t offset = 0;
    while (offset < text.size()) {
      const auto size = ::write(fd, text.data() + offset, text.size() - offset);
      if (size < 0 && errno == EINTR) {continue;}
      if (size <= 0) {throw std::runtime_error("Cannot write recorder metrics");}
      offset += size;
    }
    const int closed = ::close(fd);
    open = false;
    if (closed != 0 || ::rename(temporary.c_str(), path.c_str()) != 0) {
      throw std::runtime_error("Cannot publish recorder metrics");
    }
  } catch (...) {
    if (open) {::close(fd);}
    ::unlink(temporary.c_str());
    throw;
  }
}

class Reporter
{
public:
  Reporter(const Options & options, std::shared_ptr<vt::WriterMetrics> metrics)
  : options_(options), metrics_(std::move(metrics)) {}
  ~Reporter() {stop();}
  void publish(bool closed)
  {
    auto value = metrics_->snapshot();
    value["schema_version"] = 1;
    value["session_id"] = options_.session_id;
    value["bag_path"] = options_.storage.uri;
    value["recorder_instance_id"] = options_.instance_id;
    value["pid"] = getpid();
    value["closed"] = closed;
    value["storage_write_semantics"] = "storage_plugin_write_returned_not_fsync";
    atomic_snapshot(options_.metrics_file, value);
  }
  void start()
  {
    publish(false);
    thread_ = std::thread([this] {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!condition_.wait_for(lock, std::chrono::milliseconds(500), [this] {return done_;})) {
          lock.unlock();
          try {publish(false);} catch (const std::exception & error) {
            // Missing/stale observations are reported as unavailable downstream.
            std::cerr << "Recorder metrics unavailable: " << error.what() << '\n';
          }
          lock.lock();
        }
      });
  }
  void stop()
  {
    {std::lock_guard<std::mutex> lock(mutex_); done_ = true;}
    condition_.notify_all();
    if (thread_.joinable()) {thread_.join();}
  }
private:
  Options options_;
  std::shared_ptr<vt::WriterMetrics> metrics_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool done_ = false;
  std::thread thread_;
};
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse(argc, argv);
    const auto metrics = std::make_shared<vt::WriterMetrics>(options.record.topics);
    Reporter reporter(options, metrics);
    // Leave the ROS context alive until Recorder::stop has drained the cache.
    rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    auto writer = std::make_shared<rosbag2_cpp::Writer>(std::make_unique<vt::ObservedWriter>(metrics));
    auto recorder = std::make_shared<rosbag2_transport::Recorder>(
      writer, options.storage, options.record, options.node_name);
    reporter.start();
    recorder->record();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(recorder);
    while (!stop_requested && rclcpp::ok()) {
      executor.spin_some(std::chrono::milliseconds(10));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    recorder->stop();
    reporter.stop();
    reporter.publish(true);
    executor.remove_node(recorder);
    recorder.reset();
    writer.reset();
    rclcpp::shutdown();
    const auto final = metrics->snapshot();
    return (final["writer_errors"].get<uint64_t>() ||
            final["storage_errors"].get<uint64_t>()) ? 2 : 0;
  } catch (const std::exception & error) {
    std::cerr << "Monitored recorder failed: " << error.what() << '\n';
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 1;
  }
}
