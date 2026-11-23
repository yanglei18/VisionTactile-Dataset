#pragma once

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <rosbag2_storage/qos.hpp>

namespace vt_realsense_capture
{
// The storage YAML decoder expects a complete metadata profile, whereas the
// ros2 bag CLI accepts partial overrides. Fill omitted policies from the same
// RMW defaults used by rclpy.QoSProfile before calling the official decoder.
inline std::unordered_map<std::string, rclcpp::QoS> recording_qos(const YAML::Node & input)
{
  if (!input.IsMap()) {throw std::invalid_argument("QoS overrides must be a topic mapping");}
  std::unordered_map<std::string, rclcpp::QoS> result;
  for (const auto & entry : input) {
    const auto topic = entry.first.as<std::string>();
    const auto partial = entry.second;
    if (!partial.IsMap() || (!partial["history"] && !partial["depth"])) {
      throw std::invalid_argument("QoS override requires history or depth");
    }
    auto complete = YAML::convert<rclcpp::QoS>::encode(rclcpp::QoS(rmw_qos_profile_default.depth));
    for (const auto & policy : partial) {
      const auto name = policy.first.as<std::string>();
      if (name == "avoid_ros_namespace_conventions") {continue;}
      if (!complete[name]) {throw std::invalid_argument("Unexpected QoS policy: " + name);}
      complete[name] = policy.second;
    }
    rclcpp::QoS qos(rmw_qos_profile_default.depth);
    if (!YAML::convert<rclcpp::QoS>::decode(complete, qos)) {
      throw std::invalid_argument("Invalid recording QoS profile");
    }
    if (partial["avoid_ros_namespace_conventions"]) {
      qos.avoid_ros_namespace_conventions(partial["avoid_ros_namespace_conventions"].as<bool>());
    }
    if (!result.emplace(topic, qos).second) {
      throw std::invalid_argument("Duplicate QoS topic");
    }
  }
  return result;
}
}  // namespace vt_realsense_capture
