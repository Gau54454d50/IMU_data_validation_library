#include "imu_validator/imu_validation_test_framework/scenario_validator.hpp"

namespace imu_validator::test_framework
{
std::vector<ValidationResult> ConfiguredImuScenarioValidator::validate_message(
  const sensor_msgs::msg::Imu & imu_message,
  const PlatformProfile & platform_profile,
  std::size_t) const
{
  ImuValidator runtime_validator{platform_profile.validation_config};
  return runtime_validator.validate(imu_message);
}
}  // namespace imu_validator::test_framework
