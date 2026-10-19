#include <librealsense2/rs.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <regex>
#include <string>

namespace {
constexpr double kMinimumReasonableScaleMeters = 1e-6;
constexpr double kMaximumReasonableScaleMeters = 0.1;
}

int main(int argc, char ** argv)
{
  if (argc != 2 || std::regex_match(argv[1], std::regex("[0-9]+")) == false) {
    std::cerr << "usage: depth_scale_probe <numeric-camera-serial>\n";
    return 64;
  }
  const std::string requested_serial(argv[1]);
  try {
    rs2::context context;
    const auto devices = context.query_devices();
    std::optional<double> measured_scale;
    std::size_t matches = 0;
    for (auto && device : devices) {
      if (!device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
        continue;
      }
      const std::string observed_serial =
        device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      if (observed_serial != requested_serial) {
        continue;
      }
      ++matches;
      const auto depth_sensor = device.first<rs2::depth_sensor>();
      measured_scale = static_cast<double>(depth_sensor.get_depth_scale());
    }
    if (matches != 1 || !measured_scale.has_value()) {
      std::cerr << "camera serial matched " << matches << " devices\n";
      return 3;
    }
    const double scale = *measured_scale;
    if (!std::isfinite(scale) || scale < kMinimumReasonableScaleMeters ||
      scale > kMaximumReasonableScaleMeters)
    {
      std::cerr << "depth scale is invalid or unreasonable\n";
      return 4;
    }
    std::cout << requested_serial << '\t'
              << std::setprecision(std::numeric_limits<double>::max_digits10)
              << scale << '\n';
    return 0;
  } catch (const rs2::error & error) {
    std::cerr << "librealsense error: " << error.what() << '\n';
    return 2;
  } catch (const std::exception & error) {
    std::cerr << "probe error: " << error.what() << '\n';
    return 2;
  }
}
