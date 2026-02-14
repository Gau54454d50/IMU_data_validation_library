#include <gtest/gtest.h>

#include "imu_validator/imu_validation_test_framework/platform_profile.hpp"
#include "imu_validator/imu_validation_test_framework/programmatic_imu_data_source.hpp"
#include "imu_validator/imu_validation_test_framework/scenario_validator.hpp"
#include "imu_validator/imu_validation_test_framework/validation_scenario_runner.hpp"

namespace
{
sensor_msgs::msg::Imu make_imu(double angular_x)
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp.sec = 1;
  imu.header.stamp.nanosec = 10;
  imu.angular_velocity.x = angular_x;
  imu.angular_velocity.y = 0.1;
  imu.angular_velocity.z = -0.1;
  imu.linear_acceleration.x = 1.0;
  imu.linear_acceleration.y = 0.2;
  imu.linear_acceleration.z = 9.7;
  imu.orientation.w = 1.0;

  imu.orientation_covariance = {1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0};
  imu.angular_velocity_covariance = imu.orientation_covariance;
  imu.linear_acceleration_covariance = imu.orientation_covariance;
  return imu;
}
}  // namespace

TEST(ProgrammaticImuDataSource, ReturnsMessagesInInsertionOrder)
{
  imu_validator::test_framework::ProgrammaticImuDataSource source{"test_source"};
  source.add_message(make_imu(0.1));
  source.add_message(make_imu(0.2));

  const auto messages = source.load_messages();
  ASSERT_EQ(messages.size(), 2U);
  EXPECT_DOUBLE_EQ(messages[0].angular_velocity.x, 0.1);
  EXPECT_DOUBLE_EQ(messages[1].angular_velocity.x, 0.2);
  EXPECT_EQ(source.name(), "test_source");
}

TEST(ValidationScenarioRunner, AggregatesPassAndFailCounts)
{
  imu_validator::test_framework::ProgrammaticImuDataSource source{"scenario_source"};
  source.set_messages({make_imu(1.0), make_imu(100.0)});

  imu_validator::test_framework::PlatformProfile profile;
  profile.profile_name = "tight_limits";
  profile.validation_config.max_abs_angular_velocity_rad_s = 10.0;
  profile.validation_config.enable_stationary_drift_check = false;
  profile.validation_config.enable_magnetic_anomaly_proxy_check = false;

  imu_validator::test_framework::ConfiguredImuScenarioValidator validator;
  imu_validator::test_framework::ValidationScenarioRunner runner;

  const auto result = runner.run(source, profile, validator);

  EXPECT_EQ(result.data_source_name, "scenario_source");
  EXPECT_EQ(result.profile_name, "tight_limits");
  EXPECT_EQ(result.total_messages, 2U);
  EXPECT_EQ(result.pass_messages, 1U);
  EXPECT_EQ(result.warn_messages, 0U);
  EXPECT_EQ(result.fail_messages, 1U);
  EXPECT_EQ(result.aggregate_status, imu_validator::ValidationStatus::FAIL);
  ASSERT_EQ(result.message_results.size(), 2U);
  EXPECT_EQ(result.message_results[0].aggregate_status, imu_validator::ValidationStatus::PASS);
  EXPECT_EQ(result.message_results[1].aggregate_status, imu_validator::ValidationStatus::FAIL);
}
