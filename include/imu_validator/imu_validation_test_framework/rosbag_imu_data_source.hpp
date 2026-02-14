#pragma once

#include "imu_validator/imu_validation_test_framework/imu_data_source.hpp"

#include <string>

namespace imu_validator::test_framework
{
class RosbagImuDataSource : public IImuDataSource
{
public:
  RosbagImuDataSource(
    std::string bag_path,
    std::string imu_topic = "/imu/data",
    std::string storage_id = "");

  std::string name() const override;
  std::vector<sensor_msgs::msg::Imu> load_messages() const override;

private:
  std::string infer_storage_id() const;

  std::string bag_path_;
  std::string imu_topic_;
  std::string storage_id_;
};
}  // namespace imu_validator::test_framework
