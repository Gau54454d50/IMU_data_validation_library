#pragma once

#include "imu_validator/imu_validation_config.hpp"
#include "imu_validator/validation_result.hpp"

#include <optional>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace imu_validator
{
class ImuValidator
{
public:
  explicit ImuValidator(ImuValidationConfig config = ImuValidationConfig{});

  const ImuValidationConfig & config() const noexcept;

  std::vector<ValidationResult> validate(
    const sensor_msgs::msg::Imu & imu_msg,
    const std::optional<builtin_interfaces::msg::Time> & reference_time = std::nullopt) const;

private:
  ImuValidationConfig config_;
};
}  // namespace imu_validator
