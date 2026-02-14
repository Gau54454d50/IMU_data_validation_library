// Example: wiring a real IMU transport (serial/USB/I2C bridge) into imu_validator.
//
// This demonstrates how a production data source can implement IImuDataSource.
// Replace `read_packets_from_device(...)` and parsing logic with your real device protocol.

#include "imu_validator/imu_validation_test_framework/imu_data_source.hpp"
#include "imu_validator/imu_validation_test_framework/scenario_validator.hpp"
#include "imu_validator/imu_validation_test_framework/validation_scenario_runner.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace
{
struct RawImuPacket
{
  int64_t stamp_ns;
  double gx;
  double gy;
  double gz;
  double ax;
  double ay;
  double az;
};

std::vector<RawImuPacket> read_packets_from_device(
  const std::string & device_uri,
  const std::size_t sample_limit)
{
  // PSEUDOCODE PLACEHOLDER:
  // - Open serial/USB endpoint (e.g. /dev/ttyUSB0) OR talk to an I2C bridge process.
  // - Read raw bytes and decode vendor packet format.
  // - Convert units to SI (rad/s and m/s^2).
  // - Return bounded batch so validation scenario remains deterministic.
  (void)device_uri;
  (void)sample_limit;
  return {};
}

sensor_msgs::msg::Imu to_ros_imu(const RawImuPacket & packet)
{
  sensor_msgs::msg::Imu imu;

  imu.header.stamp.sec = static_cast<int32_t>(packet.stamp_ns / 1000000000LL);
  imu.header.stamp.nanosec = static_cast<uint32_t>(packet.stamp_ns % 1000000000LL);

  // If your device does not provide orientation, keep neutral/unknown policy as appropriate.
  imu.orientation.w = 1.0;

  imu.angular_velocity.x = packet.gx;
  imu.angular_velocity.y = packet.gy;
  imu.angular_velocity.z = packet.gz;

  imu.linear_acceleration.x = packet.ax;
  imu.linear_acceleration.y = packet.ay;
  imu.linear_acceleration.z = packet.az;

  imu.orientation_covariance = {
    0.05, 0.0, 0.0,
    0.0, 0.05, 0.0,
    0.0, 0.0, 0.05};
  imu.angular_velocity_covariance = imu.orientation_covariance;
  imu.linear_acceleration_covariance = imu.orientation_covariance;

  return imu;
}

class LiveSerialImuDataSource : public imu_validator::test_framework::IImuDataSource
{
public:
  LiveSerialImuDataSource(std::string device_uri, std::size_t sample_limit)
  : device_uri_(std::move(device_uri)), sample_limit_(sample_limit)
  {
  }

  std::string name() const override
  {
    return "live_serial:" + device_uri_;
  }

  std::vector<sensor_msgs::msg::Imu> load_messages() const override
  {
    const auto packets = read_packets_from_device(device_uri_, sample_limit_);
    std::vector<sensor_msgs::msg::Imu> out;
    out.reserve(packets.size());
    for (const auto & packet : packets) {
      out.push_back(to_ros_imu(packet));
    }
    return out;
  }

private:
  std::string device_uri_;
  std::size_t sample_limit_;
};
}  // namespace

int main()
{
  using imu_validator::test_framework::ConfiguredImuScenarioValidator;
  using imu_validator::test_framework::PlatformProfile;
  using imu_validator::test_framework::ValidationScenarioRunner;

  // Example sources:
  //   serial: /dev/ttyUSB0, /dev/ttyACM0
  //   usb bridge: tcp://127.0.0.1:5000
  //   i2c bridge: i2c://bus-1/0x68
  LiveSerialImuDataSource source{"/dev/ttyUSB0", 500};

  PlatformProfile profile;
  profile.profile_name = "hardware_session";
  profile.validation_config.max_abs_angular_velocity_rad_s = 20.0;
  profile.validation_config.max_abs_linear_acceleration_m_s2 = 40.0;
  profile.validation_config.max_past_age_sec = 0.2;
  profile.validation_config.max_future_timestamp_sec = 0.02;

  ConfiguredImuScenarioValidator validator;
  ValidationScenarioRunner runner;

  const auto result = runner.run(source, profile, validator);

  // In production you would persist/export ScenarioValidationResult here.
  if (result.total_messages == 0U) {
    throw std::runtime_error(
            "No IMU packets were read from device. Verify cable/power/permissions/protocol.");
  }

  return 0;
}
