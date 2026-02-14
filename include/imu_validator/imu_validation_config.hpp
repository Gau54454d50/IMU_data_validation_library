#pragma once

#include <cstddef>

namespace imu_validator
{
struct ImuValidationConfig
{
  // Physical limits.
  double max_abs_angular_velocity_rad_s{50.0};
  double max_abs_linear_acceleration_m_s2{100.0};

  // Timestamp sanity checks.
  bool require_non_zero_timestamp{true};
  double max_future_timestamp_sec{0.25};
  double max_past_age_sec{10.0};

  // Covariance checks.
  bool require_covariance_diagonal_non_negative{true};
  bool require_covariance_symmetric{true};
  double covariance_symmetry_tolerance{1e-6};
  double max_covariance_abs_value{1e6};

  // Field-focused anomaly heuristics.
  bool enable_stationary_drift_check{true};
  double stationary_accel_magnitude_m_s2{9.80665};
  double stationary_accel_tolerance_m_s2{0.75};
  double max_stationary_angular_velocity_rad_s{0.08};

  bool enable_magnetic_anomaly_proxy_check{true};
  double orientation_covariance_magnetic_anomaly_warn_threshold{2.5};

  // Misc guards.
  bool validate_orientation_fields{true};
};
}  // namespace imu_validator
