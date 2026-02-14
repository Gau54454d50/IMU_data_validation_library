#include "imu_validator/imu_validation_test_framework/yaml_platform_profile_provider.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace imu_validator::test_framework
{
namespace
{
std::string trim(const std::string & in)
{
  const std::size_t first = in.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const std::size_t last = in.find_last_not_of(" \t\r\n");
  return in.substr(first, last - first + 1);
}

bool parse_bool(const std::string & value)
{
  return value == "true" || value == "True" || value == "1";
}

void assign_config_value(
  ImuValidationConfig & config,
  const std::string & key,
  const std::string & value)
{
  if (key == "max_abs_angular_velocity_rad_s") {
    config.max_abs_angular_velocity_rad_s = std::stod(value);
  } else if (key == "max_abs_linear_acceleration_m_s2") {
    config.max_abs_linear_acceleration_m_s2 = std::stod(value);
  } else if (key == "validate_orientation_fields") {
    config.validate_orientation_fields = parse_bool(value);
  } else if (key == "require_non_zero_timestamp") {
    config.require_non_zero_timestamp = parse_bool(value);
  } else if (key == "max_future_timestamp_sec") {
    config.max_future_timestamp_sec = std::stod(value);
  } else if (key == "max_past_age_sec") {
    config.max_past_age_sec = std::stod(value);
  } else if (key == "max_covariance_abs_value") {
    config.max_covariance_abs_value = std::stod(value);
  } else if (key == "require_covariance_diagonal_non_negative") {
    config.require_covariance_diagonal_non_negative = parse_bool(value);
  } else if (key == "require_covariance_symmetric") {
    config.require_covariance_symmetric = parse_bool(value);
  } else if (key == "covariance_symmetry_tolerance") {
    config.covariance_symmetry_tolerance = std::stod(value);
  } else if (key == "enable_stationary_drift_check") {
    config.enable_stationary_drift_check = parse_bool(value);
  } else if (key == "stationary_accel_magnitude_m_s2") {
    config.stationary_accel_magnitude_m_s2 = std::stod(value);
  } else if (key == "stationary_accel_tolerance_m_s2") {
    config.stationary_accel_tolerance_m_s2 = std::stod(value);
  } else if (key == "max_stationary_angular_velocity_rad_s") {
    config.max_stationary_angular_velocity_rad_s = std::stod(value);
  } else if (key == "enable_magnetic_anomaly_proxy_check") {
    config.enable_magnetic_anomaly_proxy_check = parse_bool(value);
  } else if (key == "orientation_covariance_magnetic_anomaly_warn_threshold") {
    config.orientation_covariance_magnetic_anomaly_warn_threshold = std::stod(value);
  }
}
}  // namespace

YamlPlatformProfileProvider::YamlPlatformProfileProvider(std::string profiles_directory)
: profiles_directory_(std::move(profiles_directory))
{
}

PlatformProfile YamlPlatformProfileProvider::load_profile(const std::string & profile_id) const
{
  const std::filesystem::path profile_path =
    std::filesystem::path(profiles_directory_) / (profile_id + ".yaml");
  std::ifstream input(profile_path);
  if (!input.is_open()) {
    throw std::runtime_error("Failed to open profile config: " + profile_path.string());
  }

  PlatformProfile profile;
  profile.profile_name = profile_id;

  std::string section;
  std::string line;
  while (std::getline(input, line)) {
    const std::string stripped = trim(line);
    if (stripped.empty() || stripped[0] == '#') {
      continue;
    }

    if (stripped.back() == ':') {
      section = stripped.substr(0, stripped.size() - 1);
      continue;
    }

    const std::size_t delim = stripped.find(':');
    if (delim == std::string::npos) {
      continue;
    }

    const std::string key = trim(stripped.substr(0, delim));
    const std::string value = trim(stripped.substr(delim + 1));

    if (section == "profile" && key == "name") {
      profile.profile_name = value;
    } else if (section == "validator") {
      assign_config_value(profile.validation_config, key, value);
    } else if (section == "envelope") {
      profile.envelope_parameters[key] = std::stod(value);
    }
  }

  return profile;
}
}  // namespace imu_validator::test_framework
