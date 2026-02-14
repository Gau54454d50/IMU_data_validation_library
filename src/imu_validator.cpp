#include "imu_validator/imu_validator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <string>

namespace imu_validator
{
namespace
{
constexpr std::size_t kCovRows = 3;
constexpr std::size_t kCovCols = 3;
constexpr std::size_t kCovSize = kCovRows * kCovCols;

bool is_finite(double value) noexcept
{
  return std::isfinite(value);
}

void append_result(
  std::vector<ValidationResult> & results,
  std::string check_name,
  ValidationStatus status,
  std::vector<double> measured,
  std::string expected,
  std::string message)
{
  results.push_back(ValidationResult{
    std::move(check_name),
    status,
    std::move(measured),
    std::move(expected),
    std::move(message)});
}

double to_seconds(const builtin_interfaces::msg::Time & stamp) noexcept
{
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) / 1'000'000'000.0;
}

ValidationStatus worst(ValidationStatus a, ValidationStatus b) noexcept
{
  return (static_cast<int>(a) >= static_cast<int>(b)) ? a : b;
}

void validate_vector3_finite(
  const std::string & check_name,
  double x,
  double y,
  double z,
  std::vector<ValidationResult> & out)
{
  const bool ok = is_finite(x) && is_finite(y) && is_finite(z);
  append_result(
    out,
    check_name,
    ok ? ValidationStatus::PASS : ValidationStatus::FAIL,
    {x, y, z},
    "all values finite",
    ok ? "All components are finite." : "One or more components are NaN/Inf.");
}

void validate_covariance(
  const std::array<double, kCovSize> & covariance,
  const std::string & name,
  const ImuValidationConfig & config,
  std::vector<ValidationResult> & out)
{
  if (covariance[0] == -1.0) {
    append_result(
      out,
      name + "_availability",
      ValidationStatus::WARN,
      {covariance[0]},
      "covariance[0] == -1 indicates unavailable measurement (ROS2 convention)",
      "Measurement unavailable by convention; covariance checks skipped.");
    return;
  }

  ValidationStatus aggregate_status = ValidationStatus::PASS;
  std::string aggregate_message{"Covariance matrix is valid."};

  for (std::size_t i = 0; i < covariance.size(); ++i) {
    if (!is_finite(covariance[i])) {
      aggregate_status = ValidationStatus::FAIL;
      aggregate_message = "Covariance contains non-finite value(s).";
      break;
    }
    if (std::fabs(covariance[i]) > config.max_covariance_abs_value) {
      aggregate_status = worst(aggregate_status, ValidationStatus::FAIL);
      aggregate_message = "Covariance has value outside configured absolute bound.";
    }
  }

  if (aggregate_status != ValidationStatus::FAIL &&
      config.require_covariance_diagonal_non_negative) {
    for (std::size_t axis = 0; axis < kCovRows; ++axis) {
      const std::size_t idx = axis * kCovRows + axis;
      if (covariance[idx] < 0.0) {
        aggregate_status = ValidationStatus::FAIL;
        aggregate_message = "Covariance diagonal contains negative variance.";
        break;
      }
    }
  }

  if (aggregate_status != ValidationStatus::FAIL && config.require_covariance_symmetric) {
    for (std::size_t r = 0; r < kCovRows; ++r) {
      for (std::size_t c = r + 1; c < kCovCols; ++c) {
        const double a = covariance[r * kCovCols + c];
        const double b = covariance[c * kCovCols + r];
        if (std::fabs(a - b) > config.covariance_symmetry_tolerance) {
          aggregate_status = ValidationStatus::FAIL;
          aggregate_message = "Covariance matrix is not symmetric within tolerance.";
          r = kCovRows;
          break;
        }
      }
    }
  }

  append_result(
    out,
    name,
    aggregate_status,
    std::vector<double>(covariance.begin(), covariance.end()),
    "finite matrix, |value| <= max_covariance_abs_value, diagonal >= 0, symmetric",
    aggregate_message);
}

double vector_norm3(double x, double y, double z)
{
  return std::sqrt((x * x) + (y * y) + (z * z));
}

}  // namespace

ImuValidator::ImuValidator(ImuValidationConfig config)
: config_(std::move(config))
{
}

const ImuValidationConfig & ImuValidator::config() const noexcept
{
  return config_;
}

std::vector<ValidationResult> ImuValidator::validate(
  const sensor_msgs::msg::Imu & imu_msg,
  const std::optional<builtin_interfaces::msg::Time> & reference_time) const
{
  std::vector<ValidationResult> results;
  results.reserve(12);

  validate_vector3_finite(
    "angular_velocity_finite",
    imu_msg.angular_velocity.x,
    imu_msg.angular_velocity.y,
    imu_msg.angular_velocity.z,
    results);

  validate_vector3_finite(
    "linear_acceleration_finite",
    imu_msg.linear_acceleration.x,
    imu_msg.linear_acceleration.y,
    imu_msg.linear_acceleration.z,
    results);

  if (config_.validate_orientation_fields) {
    const bool orientation_finite =
      is_finite(imu_msg.orientation.x) && is_finite(imu_msg.orientation.y) &&
      is_finite(imu_msg.orientation.z) && is_finite(imu_msg.orientation.w);
    append_result(
      results,
      "orientation_finite",
      orientation_finite ? ValidationStatus::PASS : ValidationStatus::FAIL,
      {imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w},
      "all values finite",
      orientation_finite ? "Orientation values are finite."
                         : "Orientation contains NaN/Inf values.");
  }

  auto check_physical_limit = [this, &results](
                                const std::string & name,
                                double x,
                                double y,
                                double z,
                                double max_abs,
                                const std::string & units) {
    const double max_component =
      std::max({std::fabs(x), std::fabs(y), std::fabs(z)});
    const bool ok = max_component <= max_abs;

    std::ostringstream expected;
    expected << "abs(component) <= " << max_abs << " " << units;

    append_result(
      results,
      name,
      ok ? ValidationStatus::PASS : ValidationStatus::FAIL,
      {x, y, z},
      expected.str(),
      ok ? "Within configured physical bounds." : "Physical bound exceeded.");
  };

  check_physical_limit(
    "angular_velocity_physical_range",
    imu_msg.angular_velocity.x,
    imu_msg.angular_velocity.y,
    imu_msg.angular_velocity.z,
    config_.max_abs_angular_velocity_rad_s,
    "rad/s");

  check_physical_limit(
    "linear_acceleration_physical_range",
    imu_msg.linear_acceleration.x,
    imu_msg.linear_acceleration.y,
    imu_msg.linear_acceleration.z,
    config_.max_abs_linear_acceleration_m_s2,
    "m/s^2");

  if (config_.enable_stationary_drift_check) {
    const double accel_norm = vector_norm3(
      imu_msg.linear_acceleration.x,
      imu_msg.linear_acceleration.y,
      imu_msg.linear_acceleration.z);
    const double gyro_norm = vector_norm3(
      imu_msg.angular_velocity.x,
      imu_msg.angular_velocity.y,
      imu_msg.angular_velocity.z);

    const bool near_stationary_accel =
      std::fabs(accel_norm - config_.stationary_accel_magnitude_m_s2) <=
      config_.stationary_accel_tolerance_m_s2;
    const bool excessive_stationary_rotation =
      near_stationary_accel && (gyro_norm > config_.max_stationary_angular_velocity_rad_s);

    std::ostringstream expected;
    expected << "if |accel_norm - " << config_.stationary_accel_magnitude_m_s2
             << "| <= " << config_.stationary_accel_tolerance_m_s2
             << ", then gyro_norm <= " << config_.max_stationary_angular_velocity_rad_s
             << " rad/s";

    append_result(
      results,
      "stationary_gyro_drift_proxy",
      excessive_stationary_rotation ? ValidationStatus::WARN : ValidationStatus::PASS,
      {accel_norm, gyro_norm},
      expected.str(),
      excessive_stationary_rotation
        ? "Potential gyro drift: angular rate too high while acceleration appears stationary."
        : "No stationary gyro drift signature detected.");
  }

  const auto & stamp = imu_msg.header.stamp;
  if (stamp.nanosec >= 1'000'000'000U) {
    append_result(
      results,
      "timestamp_nanosec_range",
      ValidationStatus::FAIL,
      {static_cast<double>(stamp.sec), static_cast<double>(stamp.nanosec)},
      "0 <= nanosec < 1e9",
      "nanosec field is out of range.");
  } else {
    append_result(
      results,
      "timestamp_nanosec_range",
      ValidationStatus::PASS,
      {static_cast<double>(stamp.sec), static_cast<double>(stamp.nanosec)},
      "0 <= nanosec < 1e9",
      "nanosec field is in valid range.");
  }

  if (config_.require_non_zero_timestamp) {
    const bool non_zero = (stamp.sec != 0) || (stamp.nanosec != 0);
    append_result(
      results,
      "timestamp_non_zero",
      non_zero ? ValidationStatus::PASS : ValidationStatus::WARN,
      {static_cast<double>(stamp.sec), static_cast<double>(stamp.nanosec)},
      "timestamp should be non-zero",
      non_zero ? "Timestamp is non-zero." : "Timestamp is zero (possibly unsynchronized source).");
  }

  if (reference_time.has_value()) {
    const double message_time_sec = to_seconds(stamp);
    const double ref_time_sec = to_seconds(reference_time.value());
    const double delta = message_time_sec - ref_time_sec;

    const bool future_ok = delta <= config_.max_future_timestamp_sec;
    const bool past_ok = delta >= -config_.max_past_age_sec;
    const ValidationStatus status = (future_ok && past_ok) ? ValidationStatus::PASS
                                                            : ValidationStatus::WARN;

    std::ostringstream expected;
    expected << "-" << config_.max_past_age_sec << " <= (msg_time - ref_time) <= "
             << config_.max_future_timestamp_sec << " sec";

    append_result(
      results,
      "timestamp_sanity_against_reference",
      status,
      {delta},
      expected.str(),
      (future_ok && past_ok) ? "Timestamp is within configured age/skew limits."
                             : "Timestamp is too old or too far in the future.");
  }

  validate_covariance(
    imu_msg.orientation_covariance,
    "orientation_covariance",
    config_,
    results);

  if (config_.enable_magnetic_anomaly_proxy_check) {
    const auto & orientation_cov = imu_msg.orientation_covariance;
    if (orientation_cov[0] == -1.0) {
      append_result(
        results,
        "orientation_covariance_magnetic_anomaly_proxy",
        ValidationStatus::WARN,
        {orientation_cov[0]},
        "orientation covariance available for anomaly proxy",
        "Orientation covariance unavailable, cannot evaluate magnetic anomaly proxy.");
    } else {
      const double max_diag = std::max(
        {orientation_cov[0], orientation_cov[4], orientation_cov[8]});
      const bool suspicious =
        is_finite(max_diag) &&
        max_diag > config_.orientation_covariance_magnetic_anomaly_warn_threshold;

      std::ostringstream expected;
      expected << "max(orientation_cov_diag) <= "
               << config_.orientation_covariance_magnetic_anomaly_warn_threshold;

      append_result(
        results,
        "orientation_covariance_magnetic_anomaly_proxy",
        suspicious ? ValidationStatus::WARN : ValidationStatus::PASS,
        {orientation_cov[0], orientation_cov[4], orientation_cov[8]},
        expected.str(),
        suspicious
          ? "Orientation covariance unusually high; possible magnetic disturbance in fused heading."
          : "Orientation covariance does not indicate magnetic disturbance.");
    }
  }

  validate_covariance(
    imu_msg.angular_velocity_covariance,
    "angular_velocity_covariance",
    config_,
    results);
  validate_covariance(
    imu_msg.linear_acceleration_covariance,
    "linear_acceleration_covariance",
    config_,
    results);

  return results;
}

}  // namespace imu_validator
