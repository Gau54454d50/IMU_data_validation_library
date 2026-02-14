#pragma once

#include "imu_validator/imu_validation_test_framework/platform_profile.hpp"

#include <string>

namespace imu_validator::test_framework
{
class YamlPlatformProfileProvider : public IPlatformProfileProvider
{
public:
  explicit YamlPlatformProfileProvider(std::string profiles_directory);

  PlatformProfile load_profile(const std::string & profile_id) const override;

private:
  std::string profiles_directory_;
};
}  // namespace imu_validator::test_framework
