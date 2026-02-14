#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "imu_validator/imu_validation_test_framework/platform_profile.hpp"
#include "imu_validator/imu_validation_test_framework/scenario_validator.hpp"

namespace
{
using imu_validator::ValidationResult;
using imu_validator::ValidationStatus;
using imu_validator::test_framework::ConfiguredImuScenarioValidator;
using imu_validator::test_framework::PlatformProfile;

sensor_msgs::msg::Imu make_nominal_imu_sample()
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp.sec = 42;
  imu.header.stamp.nanosec = 10;

  imu.orientation.x = 0.0;
  imu.orientation.y = 0.0;
  imu.orientation.z = 0.0;
  imu.orientation.w = 1.0;

  imu.angular_velocity.x = 0.4;
  imu.angular_velocity.y = -0.2;
  imu.angular_velocity.z = 0.7;

  imu.linear_acceleration.x = 0.8;
  imu.linear_acceleration.y = -0.1;
  imu.linear_acceleration.z = 9.6;

  const std::array<double, 9> valid_cov = {
    1.0, 0.01, 0.00,
    0.01, 1.2, 0.02,
    0.00, 0.02, 1.5};
  imu.orientation_covariance = valid_cov;
  imu.angular_velocity_covariance = valid_cov;
  imu.linear_acceleration_covariance = valid_cov;
  return imu;
}

PlatformProfile make_profile(
  const std::string & name,
  double max_gyro,
  double max_accel,
  bool require_non_zero_timestamp = true)
{
  PlatformProfile profile;
  profile.profile_name = name;
  profile.validation_config.max_abs_angular_velocity_rad_s = max_gyro;
  profile.validation_config.max_abs_linear_acceleration_m_s2 = max_accel;
  profile.validation_config.require_non_zero_timestamp = require_non_zero_timestamp;
  return profile;
}

const ValidationResult * find_check(
  const std::vector<ValidationResult> & checks,
  const std::string & check_name)
{
  const auto it = std::find_if(
    checks.begin(),
    checks.end(),
    [&](const ValidationResult & check) { return check.check_name == check_name; });
  return it == checks.end() ? nullptr : &(*it);
}
}  // namespace

TEST(ImuProfilesNominalData, CleanSamplePassesAcrossMultipleProfiles)
{
  const auto imu = make_nominal_imu_sample();
  ConfiguredImuScenarioValidator validator;

  const std::vector<PlatformProfile> profiles = {
    make_profile("slow_asv", 2.0, 15.0),
    make_profile("high_speed_asv", 15.0, 40.0),
    make_profile("auv", 5.0, 25.0),
    make_profile("rov", 3.0, 20.0)};

  for (const auto & profile : profiles) {
    SCOPED_TRACE(profile.profile_name);
    const auto checks = validator.validate_message(imu, profile, 0);
    for (const auto & check : checks) {
      EXPECT_NE(check.status, ValidationStatus::FAIL) << check.check_name;
    }
  }
}

TEST(ImuAnomalies, NaNAndInfAndPhysicalSpikesAreDetected)
{
  auto imu = make_nominal_imu_sample();
  imu.angular_velocity.y = std::numeric_limits<double>::quiet_NaN();
  imu.linear_acceleration.z = std::numeric_limits<double>::infinity();
  imu.angular_velocity.x = 999.0;
  imu.linear_acceleration.x = -999.0;

  ConfiguredImuScenarioValidator validator;
  const auto checks = validator.validate_message(imu, make_profile("tight", 10.0, 100.0), 0);

  ASSERT_NE(find_check(checks, "angular_velocity_finite"), nullptr);
  EXPECT_EQ(find_check(checks, "angular_velocity_finite")->status, ValidationStatus::FAIL);
  ASSERT_NE(find_check(checks, "linear_acceleration_finite"), nullptr);
  EXPECT_EQ(find_check(checks, "linear_acceleration_finite")->status, ValidationStatus::FAIL);
  ASSERT_NE(find_check(checks, "angular_velocity_physical_range"), nullptr);
  EXPECT_EQ(find_check(checks, "angular_velocity_physical_range")->status, ValidationStatus::FAIL);
  ASSERT_NE(find_check(checks, "linear_acceleration_physical_range"), nullptr);
  EXPECT_EQ(find_check(checks, "linear_acceleration_physical_range")->status, ValidationStatus::FAIL);
}

TEST(ImuAnomalies, CovarianceMisuseAndMalformedDefaultLikeMessageAreReported)
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp.sec = 0;
  imu.header.stamp.nanosec = 0;
  imu.orientation.w = 1.0;
  imu.angular_velocity_covariance = {
    -0.1, 0.0, 0.0,
    0.0, 0.1, 0.0,
    0.0, 0.0, 0.1};
  imu.linear_acceleration_covariance = {
    1.0, 0.0, 0.0,
    0.2, 1.0, 0.0,
    0.0, 0.0, 1.0};
  imu.orientation_covariance = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};

  ConfiguredImuScenarioValidator validator;
  const auto checks = validator.validate_message(imu, make_profile("strict", 5.0, 30.0), 0);

  ASSERT_NE(find_check(checks, "timestamp_non_zero"), nullptr);
  EXPECT_EQ(find_check(checks, "timestamp_non_zero")->status, ValidationStatus::WARN);
  ASSERT_NE(find_check(checks, "angular_velocity_covariance"), nullptr);
  EXPECT_EQ(find_check(checks, "angular_velocity_covariance")->status, ValidationStatus::FAIL);
  ASSERT_NE(find_check(checks, "linear_acceleration_covariance"), nullptr);
  EXPECT_EQ(find_check(checks, "linear_acceleration_covariance")->status, ValidationStatus::FAIL);
}

TEST(ImuBoundaries, ExactGyroAndAccelerationThresholdsAreAccepted)
{
  auto imu = make_nominal_imu_sample();
  imu.angular_velocity.x = 10.0;
  imu.angular_velocity.y = -10.0;
  imu.angular_velocity.z = 10.0;
  imu.linear_acceleration.x = 100.0;
  imu.linear_acceleration.y = -100.0;
  imu.linear_acceleration.z = 100.0;

  ConfiguredImuScenarioValidator validator;
  const auto checks = validator.validate_message(imu, make_profile("exact_bounds", 10.0, 100.0), 0);

  ASSERT_NE(find_check(checks, "angular_velocity_physical_range"), nullptr);
  EXPECT_EQ(find_check(checks, "angular_velocity_physical_range")->status, ValidationStatus::PASS);
  ASSERT_NE(find_check(checks, "linear_acceleration_physical_range"), nullptr);
  EXPECT_EQ(find_check(checks, "linear_acceleration_physical_range")->status, ValidationStatus::PASS);
}

TEST(ImuBoundaries, TimestampBoundaryWarningsAtReferenceLimits)
{
  auto imu = make_nominal_imu_sample();
  imu.header.stamp.sec = 100;
  imu.header.stamp.nanosec = 0;

  PlatformProfile profile = make_profile("timestamp_limits", 15.0, 100.0);
  profile.validation_config.max_future_timestamp_sec = 0.25;
  profile.validation_config.max_past_age_sec = 10.0;

  imu_validator::ImuValidator validator{profile.validation_config};

  builtin_interfaces::msg::Time reference_time;
  reference_time.sec = 110;
  reference_time.nanosec = 0;
  auto checks = validator.validate(imu, reference_time);
  ASSERT_NE(find_check(checks, "timestamp_sanity_against_reference"), nullptr);
  EXPECT_EQ(find_check(checks, "timestamp_sanity_against_reference")->status, ValidationStatus::PASS);

  reference_time.sec = 100;
  reference_time.nanosec = 0;
  imu.header.stamp.nanosec = 250'000'000;
  checks = validator.validate(imu, reference_time);
  ASSERT_NE(find_check(checks, "timestamp_sanity_against_reference"), nullptr);
  EXPECT_EQ(find_check(checks, "timestamp_sanity_against_reference")->status, ValidationStatus::PASS);

  imu.header.stamp.nanosec = 250'000'001;
  checks = validator.validate(imu, reference_time);
  ASSERT_NE(find_check(checks, "timestamp_sanity_against_reference"), nullptr);
  EXPECT_EQ(find_check(checks, "timestamp_sanity_against_reference")->status, ValidationStatus::WARN);
}


TEST(ImuFieldAnomalyHeuristics, StationaryGyroDriftProxyWarnsWhenRotationIsTooHigh)
{
  auto imu = make_nominal_imu_sample();
  imu.linear_acceleration.x = 0.0;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = 9.81;
  imu.angular_velocity.x = 0.2;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;

  PlatformProfile profile = make_profile("field_proxy", 5.0, 30.0);
  profile.validation_config.max_stationary_angular_velocity_rad_s = 0.08;
  profile.validation_config.stationary_accel_tolerance_m_s2 = 0.5;

  ConfiguredImuScenarioValidator validator;
  const auto checks = validator.validate_message(imu, profile, 0);

  ASSERT_NE(find_check(checks, "stationary_gyro_drift_proxy"), nullptr);
  EXPECT_EQ(find_check(checks, "stationary_gyro_drift_proxy")->status, ValidationStatus::WARN);
}

TEST(ImuFieldAnomalyHeuristics, MagneticAnomalyProxyWarnsOnLargeOrientationCovariance)
{
  auto imu = make_nominal_imu_sample();
  imu.orientation_covariance = {
    3.0, 0.0, 0.0,
    0.0, 4.0, 0.0,
    0.0, 0.0, 5.0};

  PlatformProfile profile = make_profile("magnetic_proxy", 10.0, 40.0);
  profile.validation_config.orientation_covariance_magnetic_anomaly_warn_threshold = 2.5;

  ConfiguredImuScenarioValidator validator;
  const auto checks = validator.validate_message(imu, profile, 0);

  ASSERT_NE(find_check(checks, "orientation_covariance_magnetic_anomaly_proxy"), nullptr);
  EXPECT_EQ(
    find_check(checks, "orientation_covariance_magnetic_anomaly_proxy")->status,
    ValidationStatus::WARN);
}
TEST(ImuCrossPlatformProfiles, SameSamplePassesInAuvButFailsInSlowAsv)
{
  auto imu = make_nominal_imu_sample();
  imu.angular_velocity.x = 3.0;

  ConfiguredImuScenarioValidator validator;
  const auto slow_checks = validator.validate_message(imu, make_profile("slow_asv", 2.5, 30.0), 0);
  const auto auv_checks = validator.validate_message(imu, make_profile("auv", 5.0, 30.0), 0);

  ASSERT_NE(find_check(slow_checks, "angular_velocity_physical_range"), nullptr);
  ASSERT_NE(find_check(auv_checks, "angular_velocity_physical_range"), nullptr);
  EXPECT_EQ(find_check(slow_checks, "angular_velocity_physical_range")->status, ValidationStatus::FAIL);
  EXPECT_EQ(find_check(auv_checks, "angular_velocity_physical_range")->status, ValidationStatus::PASS);
}
