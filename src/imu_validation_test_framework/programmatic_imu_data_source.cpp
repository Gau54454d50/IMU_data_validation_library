#include "imu_validator/imu_validation_test_framework/programmatic_imu_data_source.hpp"

#include <utility>

namespace imu_validator::test_framework
{
ProgrammaticImuDataSource::ProgrammaticImuDataSource(std::string source_name)
: source_name_(std::move(source_name))
{
}

void ProgrammaticImuDataSource::add_message(const sensor_msgs::msg::Imu & message)
{
  messages_.push_back(message);
}

void ProgrammaticImuDataSource::set_messages(const std::vector<sensor_msgs::msg::Imu> & messages)
{
  messages_ = messages;
}

std::string ProgrammaticImuDataSource::name() const
{
  return source_name_;
}

std::vector<sensor_msgs::msg::Imu> ProgrammaticImuDataSource::load_messages() const
{
  return messages_;
}
}  // namespace imu_validator::test_framework
