#pragma once

#include "imu_validator/imu_validation_test_framework/imu_data_source.hpp"
#include "imu_validator/imu_validation_test_framework/platform_profile.hpp"
#include "imu_validator/imu_validation_test_framework/scenario_validator.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace imu_validator::test_framework
{
struct MessageValidationResult
{
  std::size_t message_index;
  ValidationStatus aggregate_status;
  std::vector<ValidationResult> checks;
};

struct ScenarioValidationResult
{
  std::string data_source_name;
  std::string profile_name;
  ValidationStatus aggregate_status;
  std::size_t total_messages;
  std::size_t pass_messages;
  std::size_t warn_messages;
  std::size_t fail_messages;
  std::vector<MessageValidationResult> message_results;
};

class ValidationScenarioRunner
{
public:
  ScenarioValidationResult run(
    const IImuDataSource & data_source,
    const PlatformProfile & platform_profile,
    const IScenarioValidator & validator) const;
};
}  // namespace imu_validator::test_framework
