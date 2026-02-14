#pragma once

#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>

namespace imu_validator::test_framework
{
class IImuDataSource
{
public:
  virtual ~IImuDataSource() = default;

  virtual std::string name() const = 0;
  virtual std::vector<sensor_msgs::msg::Imu> load_messages() const = 0;
};

using ImuDataSourcePtr = std::shared_ptr<IImuDataSource>;
}  // namespace imu_validator::test_framework
