#include "imu_validator/imu_validation_test_framework/validation_scenario_runner.hpp"

#include <utility>

namespace imu_validator::test_framework
{
namespace
{
ValidationStatus combine_status(ValidationStatus lhs, ValidationStatus rhs)
{
  return static_cast<int>(lhs) >= static_cast<int>(rhs) ? lhs : rhs;
}

ValidationStatus aggregate_message_status(const std::vector<ValidationResult> & checks)
{
  ValidationStatus status = ValidationStatus::PASS;
  for (const auto & check : checks) {
    status = combine_status(status, check.status);
  }
  return status;
}
}  // namespace

ScenarioValidationResult ValidationScenarioRunner::run(
  const IImuDataSource & data_source,
  const PlatformProfile & platform_profile,
  const IScenarioValidator & validator) const
{
  ScenarioValidationResult scenario_result;
  scenario_result.data_source_name = data_source.name();
  scenario_result.profile_name = platform_profile.profile_name;
  scenario_result.aggregate_status = ValidationStatus::PASS;
  scenario_result.total_messages = 0;
  scenario_result.pass_messages = 0;
  scenario_result.warn_messages = 0;
  scenario_result.fail_messages = 0;

  auto messages = data_source.load_messages();
  scenario_result.total_messages = messages.size();
  scenario_result.message_results.reserve(messages.size());

  for (std::size_t i = 0; i < messages.size(); ++i) {
    auto checks = validator.validate_message(messages[i], platform_profile, i);
    const auto aggregate_status = aggregate_message_status(checks);

    if (aggregate_status == ValidationStatus::PASS) {
      ++scenario_result.pass_messages;
    } else if (aggregate_status == ValidationStatus::WARN) {
      ++scenario_result.warn_messages;
    } else {
      ++scenario_result.fail_messages;
    }

    scenario_result.aggregate_status =
      combine_status(scenario_result.aggregate_status, aggregate_status);

    scenario_result.message_results.push_back(
      MessageValidationResult{i, aggregate_status, std::move(checks)});
  }

  return scenario_result;
}
}  // namespace imu_validator::test_framework
