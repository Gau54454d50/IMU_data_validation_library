#include <gtest/gtest.h>

#include "imu_validator/imu_validation_test_framework/yaml_platform_profile_provider.hpp"

#include <stdexcept>

TEST(YamlPlatformProfileProvider, LoadsAsvProfile)
{
  imu_validator::test_framework::YamlPlatformProfileProvider provider{TEST_PLATFORM_PROFILE_DIR};

  const auto profile = provider.load_profile("asv");

  EXPECT_EQ(profile.profile_name, "asv");
  EXPECT_DOUBLE_EQ(profile.validation_config.max_abs_angular_velocity_rad_s, 3.0);
  EXPECT_DOUBLE_EQ(profile.validation_config.max_abs_linear_acceleration_m_s2, 20.0);
  EXPECT_TRUE(profile.validation_config.validate_orientation_fields);
  EXPECT_TRUE(profile.validation_config.enable_stationary_drift_check);
  EXPECT_DOUBLE_EQ(profile.validation_config.max_stationary_angular_velocity_rad_s, 0.08);
  EXPECT_TRUE(profile.validation_config.enable_magnetic_anomaly_proxy_check);
  ASSERT_TRUE(profile.envelope_parameters.find("max_pitch_deg") != profile.envelope_parameters.end());
  EXPECT_DOUBLE_EQ(profile.envelope_parameters.at("max_pitch_deg"), 20.0);
}

TEST(YamlPlatformProfileProvider, ThrowsForMissingProfile)
{
  imu_validator::test_framework::YamlPlatformProfileProvider provider{TEST_PLATFORM_PROFILE_DIR};
  EXPECT_THROW(provider.load_profile("does_not_exist"), std::runtime_error);
}
