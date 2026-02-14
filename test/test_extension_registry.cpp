#include <gtest/gtest.h>

#include "imu_validator/imu_validation_test_framework/extension_registry.hpp"
#include "imu_validator/imu_validation_test_framework/imu_data_source.hpp"
#include "imu_validator/imu_validation_test_framework/programmatic_imu_data_source.hpp"

#include <stdexcept>

TEST(ExtensionRegistry, CreatesRegisteredExtension)
{
  imu_validator::test_framework::ExtensionRegistry<imu_validator::test_framework::IImuDataSource>
    registry;

  registry.register_factory(
    "programmatic",
    []() {
      return std::make_unique<imu_validator::test_framework::ProgrammaticImuDataSource>(
        "from_registry");
    });

  auto source = registry.create("programmatic");
  ASSERT_NE(source, nullptr);
  EXPECT_EQ(source->name(), "from_registry");
}

TEST(ExtensionRegistry, ThrowsForUnknownExtension)
{
  imu_validator::test_framework::ExtensionRegistry<imu_validator::test_framework::IImuDataSource>
    registry;

  EXPECT_THROW(static_cast<void>(registry.create("missing")), std::invalid_argument);
}
