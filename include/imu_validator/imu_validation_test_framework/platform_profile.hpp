#pragma once

#include "imu_validator/imu_validation_config.hpp"

#include <string>
#include <unordered_map>

namespace imu_validator::test_framework
{
struct PlatformProfile
{
  std::string profile_name;
  ImuValidationConfig validation_config;
  std::unordered_map<std::string, double> envelope_parameters;
};

class IPlatformProfileProvider
{
public:
  virtual ~IPlatformProfileProvider() = default;
  virtual PlatformProfile load_profile(const std::string & profile_id) const = 0;
};
}  // namespace imu_validator::test_framework
