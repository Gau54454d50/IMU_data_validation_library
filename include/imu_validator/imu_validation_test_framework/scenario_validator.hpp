#pragma once

#include "imu_validator/imu_validator.hpp"
#include "imu_validator/imu_validation_test_framework/platform_profile.hpp"

#include <cstddef>
#include <vector>

namespace imu_validator::test_framework
{
class IScenarioValidator
{
public:
  virtual ~IScenarioValidator() = default;

  virtual std::vector<ValidationResult> validate_message(
    const sensor_msgs::msg::Imu & imu_message,
    const PlatformProfile & platform_profile,
    std::size_t message_index) const = 0;
};

class ConfiguredImuScenarioValidator : public IScenarioValidator
{
public:
  std::vector<ValidationResult> validate_message(
    const sensor_msgs::msg::Imu & imu_message,
    const PlatformProfile & platform_profile,
    std::size_t message_index) const override;
};
}  // namespace imu_validator::test_framework
