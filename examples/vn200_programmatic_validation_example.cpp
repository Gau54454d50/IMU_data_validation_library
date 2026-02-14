// Example: validating VN-200 style IMU/INS samples with imu_validator
// Note: VN-200 can support higher dynamics; tune thresholds for your vehicle envelope.

#include "imu_validator/imu_validation_test_framework/programmatic_imu_data_source.hpp"
#include "imu_validator/imu_validation_test_framework/scenario_validator.hpp"
#include "imu_validator/imu_validation_test_framework/validation_scenario_runner.hpp"

#include <iostream>

namespace
{
sensor_msgs::msg::Imu make_vn200_like_sample(
  double qx, double qy, double qz, double qw,
  double gx, double gy, double gz,
  double ax, double ay, double az)
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp.sec = 200;
  imu.header.stamp.nanosec = 1000000;

  imu.orientation.x = qx;
  imu.orientation.y = qy;
  imu.orientation.z = qz;
  imu.orientation.w = qw;

  imu.angular_velocity.x = gx;
  imu.angular_velocity.y = gy;
  imu.angular_velocity.z = gz;

  imu.linear_acceleration.x = ax;
  imu.linear_acceleration.y = ay;
  imu.linear_acceleration.z = az;

  imu.orientation_covariance = {
    0.005, 0.0, 0.0,
    0.0, 0.005, 0.0,
    0.0, 0.0, 0.005};
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

  ProgrammaticImuDataSource source{"vn200_flight_segment"};
  source.set_messages({
    make_vn200_like_sample(0.0, 0.0, 0.0, 1.0, 1.2, 0.4, -0.6, 0.3, -0.2, 9.7),
    make_vn200_like_sample(0.01, -0.01, 0.0, 0.9999, 3.0, 1.1, -2.2, 0.8, -0.4, 10.2),
    make_vn200_like_sample(0.0, 0.0, 0.0, 1.0, 25.0, 0.0, 0.0, 0.4, 0.3, 9.6)  // exceeds tighter profile
  });

  PlatformProfile profile;
  profile.profile_name = "vn200_uav";
  profile.validation_config.max_abs_angular_velocity_rad_s = 20.0;
  profile.validation_config.max_abs_linear_acceleration_m_s2 = 45.0;
  profile.validation_config.max_past_age_sec = 0.2;
  profile.validation_config.max_future_timestamp_sec = 0.02;

  ConfiguredImuScenarioValidator validator;
  ValidationScenarioRunner runner;
  const auto result = runner.run(source, profile, validator);

  std::cout << "Profile: " << result.profile_name << '\n';
  std::cout << "Total=" << result.total_messages
            << " pass=" << result.pass_messages
            << " warn=" << result.warn_messages
            << " fail=" << result.fail_messages << '\n';

  for (const auto & msg_result : result.message_results) {
    std::cout << "message_index=" << msg_result.message_index
              << " checks=" << msg_result.checks.size() << '\n';
  }

  std::cout << "Scenario status="
            << (result.aggregate_status == ValidationStatus::PASS ? "PASS" :
                result.aggregate_status == ValidationStatus::WARN ? "WARN" : "FAIL")
            << '\n';

  return 0;
}
