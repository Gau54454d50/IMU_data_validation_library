#pragma once

#include "imu_validator/imu_validation_test_framework/imu_data_source.hpp"

#include <string>
#include <vector>

namespace imu_validator::test_framework
{
class ProgrammaticImuDataSource : public IImuDataSource
{
public:
  explicit ProgrammaticImuDataSource(std::string source_name = "programmatic_imu_source");

  void add_message(const sensor_msgs::msg::Imu & message);
  void set_messages(const std::vector<sensor_msgs::msg::Imu> & messages);

  std::string name() const override;
  std::vector<sensor_msgs::msg::Imu> load_messages() const override;

private:
  std::string source_name_;
  std::vector<sensor_msgs::msg::Imu> messages_;
};
}  // namespace imu_validator::test_framework
