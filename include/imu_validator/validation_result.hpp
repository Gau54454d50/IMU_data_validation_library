#pragma once

#include <string>
#include <vector>

namespace imu_validator
{
enum class ValidationStatus
{
  PASS,
  WARN,
  FAIL
};

struct ValidationResult
{
  std::string check_name;
  ValidationStatus status{ValidationStatus::PASS};
  std::vector<double> measured_values;
  std::string expected_range;
  std::string message;
};
}  // namespace imu_validator
