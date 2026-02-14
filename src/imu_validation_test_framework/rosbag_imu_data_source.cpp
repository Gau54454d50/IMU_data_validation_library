#include "imu_validator/imu_validation_test_framework/rosbag_imu_data_source.hpp"

#include <filesystem>
#include <stdexcept>
#include <utility>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>

namespace imu_validator::test_framework
{
RosbagImuDataSource::RosbagImuDataSource(
  std::string bag_path,
  std::string imu_topic,
  std::string storage_id)
: bag_path_(std::move(bag_path)),
  imu_topic_(std::move(imu_topic)),
  storage_id_(std::move(storage_id))
{
}

std::string RosbagImuDataSource::name() const
{
  return "rosbag:" + bag_path_ + "#" + imu_topic_;
}

std::string RosbagImuDataSource::infer_storage_id() const
{
  if (!storage_id_.empty()) {
    return storage_id_;
  }

  const std::filesystem::path bag{bag_path_};
  const std::string extension = bag.extension().string();
  if (extension == ".db3") {
    return "sqlite3";
  }
  if (extension == ".mcap") {
    return "mcap";
  }

  return "";
}

std::vector<sensor_msgs::msg::Imu> RosbagImuDataSource::load_messages() const
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path_;
  storage_options.storage_id = infer_storage_id();

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::Reader reader;
  reader.open(storage_options, converter_options);

  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
  std::vector<sensor_msgs::msg::Imu> messages;

  while (reader.has_next()) {
    auto serialized_message = reader.read_next();
    if (serialized_message->topic_name != imu_topic_) {
      continue;
    }

    rclcpp::SerializedMessage rclcpp_serialized{*serialized_message->serialized_data};
    sensor_msgs::msg::Imu imu;
    imu_serialization.deserialize_message(&rclcpp_serialized, &imu);
    messages.push_back(std::move(imu));
  }

  if (messages.empty()) {
    throw std::runtime_error(
            "No IMU messages were loaded from bag path '" + bag_path_ +
            "' and topic '" + imu_topic_ + "'.");
  }

  return messages;
}
}  // namespace imu_validator::test_framework
