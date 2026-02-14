// Example: validating MPU-6050 style IMU samples with imu_validator
// Note: Thresholds are practical starter values and should be tuned to your platform.

#include "imu_validator/imu_validation_test_framework/programmatic_imu_data_source.hpp"
#include "imu_validator/imu_validation_test_framework/scenario_validator.hpp"
#include "imu_validator/imu_validation_test_framework/validation_scenario_runner.hpp"

#include <iostream>

namespace
{
sensor_msgs::msg::Imu make_mpu6050_like_sample(double gx, double gy, double gz, double az)
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp.sec = 100;
  imu.header.stamp.nanosec = 0;

  // MPU-6050 has 6-axis IMU (no magnetometer). Orientation may be fused externally.
  imu.orientation.w = 1.0;

  imu.angular_velocity.x = gx;
  imu.angular_velocity.y = gy;
  imu.angular_velocity.z = gz;

  imu.linear_acceleration.x = 0.2;
  imu.linear_acceleration.y = -0.1;
  imu.linear_acceleration.z = az;

  imu.orientation_covariance = {
    0.05, 0.0, 0.0,
    0.0, 0.05, 0.0,
    0.0, 0.0, 0.05};
  imu.angular_velocity_covariance = imu.orientation_covariance;
  imu.linear_acceleration_covariance = imu.orientation_covariance;
  return imu;
}
}  // namespace

int main()
{
  using imu_validator::ValidationStatus;
  using imu_validator::test_framework::ConfiguredImuScenarioValidator;
  using imu_validator::test_framework::PlatformProfile;
  using imu_validator::test_framework::ProgrammaticImuDataSource;
  using imu_validator::test_framework::ValidationScenarioRunner;

  ProgrammaticImuDataSource source{"mpu6050_nominal_and_spike"};
  source.set_messages({
    make_mpu6050_like_sample(0.5, -0.2, 0.1, 9.8),
    make_mpu6050_like_sample(1.2, 0.1, -0.2, 9.7),
    make_mpu6050_like_sample(16.0, 0.2, 0.0, 9.6)  // intentional gyro spike
  });

  PlatformProfile profile;
  profile.profile_name = "mpu6050_surface_robot";
  profile.validation_config.max_abs_angular_velocity_rad_s = 10.0;
  profile.validation_config.max_abs_linear_acceleration_m_s2 = 30.0;
  profile.validation_config.max_past_age_sec = 0.5;
  profile.validation_config.max_future_timestamp_sec = 0.05;

  ConfiguredImuScenarioValidator validator;
  ValidationScenarioRunner runner;
  const auto result = runner.run(source, profile, validator);

  std::cout << "Profile: " << result.profile_name << '\n';
  std::cout << "Total=" << result.total_messages
            << " pass=" << result.pass_messages
            << " warn=" << result.warn_messages
            << " fail=" << result.fail_messages << '\n';

  if (result.aggregate_status == ValidationStatus::FAIL) {
    std::cout << "Scenario status: FAIL (expected due to injected spike)\n";
  }

  return 0;
}
