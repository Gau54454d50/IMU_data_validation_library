#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "imu_validator/imu_validation_test_framework/platform_profile.hpp"
#include "imu_validator/imu_validation_test_framework/programmatic_imu_data_source.hpp"
#include "imu_validator/imu_validation_test_framework/rosbag_imu_data_source.hpp"
#include "imu_validator/imu_validation_test_framework/scenario_validator.hpp"
#include "imu_validator/imu_validation_test_framework/validation_scenario_runner.hpp"

#if __has_include(<rosbag2_cpp/writer.hpp>)
#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#endif

namespace
{
using imu_validator::ValidationResult;
using imu_validator::ValidationStatus;
using imu_validator::test_framework::IScenarioValidator;
using imu_validator::test_framework::PlatformProfile;
using imu_validator::test_framework::ProgrammaticImuDataSource;
using imu_validator::test_framework::RosbagImuDataSource;
using imu_validator::test_framework::ScenarioValidationResult;
using imu_validator::test_framework::ValidationScenarioRunner;

sensor_msgs::msg::Imu make_sequence_msg(double angular_x, double accel_z, int sec)
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp.sec = sec;
  imu.header.stamp.nanosec = 0;
  imu.orientation.w = 1.0;
  imu.angular_velocity.x = angular_x;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;
  imu.linear_acceleration.x = 0.0;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = accel_z;
  imu.orientation_covariance = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  imu.angular_velocity_covariance = imu.orientation_covariance;
  imu.linear_acceleration_covariance = imu.orientation_covariance;
  return imu;
}

class SequenceDegradationValidator : public IScenarioValidator
{
public:
  std::vector<ValidationResult> validate_message(
    const sensor_msgs::msg::Imu & imu_message,
    const PlatformProfile &,
    std::size_t message_index) const override
  {
    ValidationStatus status = ValidationStatus::PASS;
    std::string message = "Stable signal characteristics.";

    if (message_index >= 2 || std::abs(imu_message.angular_velocity.x) > 1.5) {
      status = ValidationStatus::WARN;
      message = "Noise/drift growth detected.";
    }
    if (message_index >= 4 || std::abs(imu_message.linear_acceleration.z) > 14.0) {
      status = ValidationStatus::FAIL;
      message = "Degraded signal exceeded safe envelope.";
    }

    return {
      ValidationResult{
        "sequence_health",
        status,
        {imu_message.angular_velocity.x, imu_message.linear_acceleration.z},
        "stable -> warn on growth -> fail on severe degradation",
        message}};
  }
};

TEST(TimeSequenceScenarios, RunnerCapturesNoiseGrowthDriftAndDegradation)
{
  ProgrammaticImuDataSource source{"sequence_source"};
  source.set_messages({
    make_sequence_msg(0.1, 9.81, 1),
    make_sequence_msg(0.4, 9.9, 2),
    make_sequence_msg(1.6, 10.4, 3),
    make_sequence_msg(1.8, 11.5, 4),
    make_sequence_msg(2.0, 14.5, 5)});

  PlatformProfile profile;
  profile.profile_name = "sequence_profile";

  SequenceDegradationValidator validator;
  ValidationScenarioRunner runner;

  const ScenarioValidationResult result = runner.run(source, profile, validator);

  EXPECT_EQ(result.total_messages, 5U);
  EXPECT_EQ(result.pass_messages, 2U);
  EXPECT_EQ(result.warn_messages, 2U);
  EXPECT_EQ(result.fail_messages, 1U);
  EXPECT_EQ(result.aggregate_status, ValidationStatus::FAIL);
  ASSERT_EQ(result.message_results.size(), 5U);
  EXPECT_EQ(result.message_results[0].aggregate_status, ValidationStatus::PASS);
  EXPECT_EQ(result.message_results[2].aggregate_status, ValidationStatus::WARN);
  EXPECT_EQ(result.message_results[4].aggregate_status, ValidationStatus::FAIL);
}

TEST(RosbagReplay, SyntheticBagFlowsThroughDataSourceAndScenarioRunner)
{
#if __has_include(<rosbag2_cpp/writer.hpp>)
  const auto unique_suffix = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  const auto temp_root =
    std::filesystem::temp_directory_path() / ("imu_validator_test_bag_" + unique_suffix);
  std::filesystem::create_directories(temp_root);

  const std::string bag_path = (temp_root / "imu_sample.db3").string();
  const std::string imu_topic = "/imu/data";

  {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::Writer writer;
    writer.open(storage_options, converter_options);

    const auto imu_msg = make_sequence_msg(0.3, 9.7, 123);
    writer.write(imu_msg, imu_topic, rclcpp::Time(imu_msg.header.stamp));
  }

  RosbagImuDataSource source{bag_path, imu_topic};
  imu_validator::test_framework::ConfiguredImuScenarioValidator validator;

  PlatformProfile profile;
  profile.profile_name = "rosbag_profile";
  profile.validation_config.max_abs_angular_velocity_rad_s = 5.0;
  profile.validation_config.max_abs_linear_acceleration_m_s2 = 20.0;
  profile.validation_config.enable_stationary_drift_check = false;
  profile.validation_config.enable_magnetic_anomaly_proxy_check = false;

  ValidationScenarioRunner runner;
  const auto scenario = runner.run(source, profile, validator);

  EXPECT_EQ(scenario.data_source_name, "rosbag:" + bag_path + "#" + imu_topic);
  EXPECT_EQ(scenario.profile_name, "rosbag_profile");
  EXPECT_EQ(scenario.total_messages, 1U);
  EXPECT_EQ(scenario.pass_messages, 1U);
  EXPECT_EQ(scenario.warn_messages, 0U);
  EXPECT_EQ(scenario.fail_messages, 0U);
  EXPECT_EQ(scenario.aggregate_status, ValidationStatus::PASS);
  ASSERT_EQ(scenario.message_results.size(), 1U);
  EXPECT_FALSE(scenario.message_results.front().checks.empty());

  std::filesystem::remove_all(temp_root);
#else
  GTEST_SKIP() << "rosbag2_cpp::Writer API unavailable in this build environment.";
#endif
}
}  // namespace
