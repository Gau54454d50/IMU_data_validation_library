# Submission for assignment 
## topic : IMU Data Validation: Validator, Testing Framework & Test Cases
# imu_validator

ROS2-compatible C++17 library for validating `sensor_msgs::msg::Imu` messages and running reusable validation scenarios with structured per-check outputs.

## Project structure and component responsibilities

```text
imu_validator/
├── include/imu_validator/
│   ├── imu_validator.hpp                              # Validator core API
│   ├── imu_validation_config.hpp                      # Threshold/check configuration model
│   ├── validation_result.hpp                          # Structured single-check result model
│   └── imu_validation_test_framework/
│       ├── imu_data_source.hpp                        # Data source extension interface
│       ├── platform_profile.hpp                       # Platform profile model/provider interface
│       ├── scenario_validator.hpp                     # Scenario validator extension interface
│       ├── validation_scenario_runner.hpp             # Scenario runner + aggregate result model
│       ├── extension_registry.hpp                     # Generic extension registration helper
│       ├── programmatic_imu_data_source.hpp           # In-memory test data source
│       ├── rosbag_imu_data_source.hpp                 # ROS bag replay data source
│       └── yaml_platform_profile_provider.hpp         # YAML profile loader
├── src/
│   ├── imu_validator.cpp                              # Validator core check implementations
│   └── imu_validation_test_framework/                 # Data source/profile/runner implementations
├── config/platform_profiles/                          # Platform-specific threshold profiles
├── test/                                              # Unit tests (core framework behavior)
├── tests/                                             # Scenario/integration-style tests
└── scripts/verify_build.sh                            # Dependency/bootstrap helper
```

### Responsibilities by subsystem

- **Validator core (`ImuValidator`)**
  - Applies configured validation checks to one IMU message.
  - Produces `std::vector<ValidationResult>` with check name, status, measured values, expected range, and message.
- **Data sources (`IImuDataSource`)**
  - Provide message batches from different origins.
  - Built-ins: `ProgrammaticImuDataSource` and `RosbagImuDataSource`.
- **Scenario runner (`ValidationScenarioRunner`)**
  - Iterates messages from a source.
  - Uses an `IScenarioValidator` implementation for per-message checks.
  - Aggregates message-level outcomes into scenario-level counts and status.
- **Profiles (`PlatformProfile`, `YamlPlatformProfileProvider`)**
  - Encapsulate validator thresholds and envelope values by platform (ASV/AUV/high_speed_ASV/HEAUV/XLAUV/etc.).
  - Enable tuning behavior without changing C++ code.
- **Tests (`test/`, `tests/`)**
  - Validate extension points, runner aggregation behavior, profile loading, and anomaly/time-order/replay behavior.

## Build prerequisites

This package is intended for a ROS2 workspace build and requires:

- **ROS2** (Jazzy used in repository scripts/CI)
- **CMake >= 3.10**
- **C++17 compiler** (GCC/Clang)
- **GoogleTest** via `ament_cmake_gtest`
- ROS2 dependencies declared in `CMakeLists.txt` / `package.xml`, including:
  - `ament_cmake`
  - `rclcpp`
  - `sensor_msgs`
  - `rosbag2_cpp`
  - `rosbag2_storage`

## Build commands (CLI)

### Option A: ROS2 workspace build (recommended)

From your ROS2 workspace root (the directory containing `src/imu_validator`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select imu_validator --cmake-args -DBUILD_TESTING=ON
```

### Option B: CMake configure/build directly

From this repository root:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S imu_validator -B build -DBUILD_TESTING=ON
cmake --build build -j
```

### Optional helper bootstrap

```bash
bash imu_validator/scripts/verify_build.sh
```

## Exact commands to run all tests from CLI

### In a ROS2 workspace (colcon)

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select imu_validator --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --all --verbose
```

### Direct CTest from CMake build directory

```bash
source /opt/ros/jazzy/setup.bash
cmake -S imu_validator -B build -DBUILD_TESTING=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Current test executables configured in `CMakeLists.txt`:

- `test_programmatic_source_and_runner`
- `test_yaml_profile_provider`
- `test_extension_registry`
- `test_imu_validation_profiles_and_anomalies`
- `test_time_sequence_and_rosbag_replay`

## Defining and editing platform profiles and thresholds

Profiles live in:

- `imu_validator/config/platform_profiles/*.yaml`

Expected YAML sections:

- `profile`: metadata (`name`)
- `validator`: maps directly to `ImuValidationConfig` fields
- `envelope`: free-form scenario/platform values (`std::unordered_map<std::string, double>`)

  each PlatformProfile has two main parts , validation_config which is used directly by ImuValidator checks (PASS/WARN/FAIL logic),envelope_parameters are a free-form map of extra       platform limits/metadata you can use in scenario-level/custom logic. 

So envelope is basically:

“extra numeric parameters about vehicle operating envelope” (e.g., max pitch/roll, nominal speed).
### Supported `validator` keys

- `max_abs_angular_velocity_rad_s`
- `max_abs_linear_acceleration_m_s2`
- `validate_orientation_fields`
- `require_non_zero_timestamp`
- `max_future_timestamp_sec`
- `max_past_age_sec`
- `max_covariance_abs_value`
- `require_covariance_diagonal_non_negative`
- `require_covariance_symmetric`
- `covariance_symmetry_tolerance`
- `enable_stationary_drift_check`
- `stationary_accel_magnitude_m_s2`
- `stationary_accel_tolerance_m_s2`
- `max_stationary_angular_velocity_rad_s`
- `enable_magnetic_anomaly_proxy_check`
- `orientation_covariance_magnetic_anomaly_warn_threshold`

### Example profile

```yaml
profile:
  name: asv

validator:
  max_abs_angular_velocity_rad_s: 8.0
  max_abs_linear_acceleration_m_s2: 25.0
  validate_orientation_fields: true
  require_non_zero_timestamp: true
  max_future_timestamp_sec: 0.02
  max_past_age_sec: 0.25
  max_covariance_abs_value: 5000.0
  require_covariance_diagonal_non_negative: true
  require_covariance_symmetric: true
  covariance_symmetry_tolerance: 0.000001

envelope:
  max_pitch_deg: 20.0
  max_roll_deg: 25.0
  nominal_speed_m_s: 6.0
```

### Adding a new profile

1. Add a new file under `config/platform_profiles/<profile_id>.yaml`.
2. Keep the same section structure (`profile`, `validator`, `envelope`).
3. Load it with `YamlPlatformProfileProvider::load_profile("<profile_id>")`.

## Extending the system

### Add a new validation check

1. Add new config knobs (if needed) in `ImuValidationConfig`.
2. Implement the new check in `ImuValidator::validate(...)`.
3. Return a `ValidationResult` with a stable `check_name`, proper status, measured values, and expected range.
4. Add/extend tests under `test/` or `tests/`.

### Add a new scenario validator

1. Implement `IScenarioValidator`:
   - `validate_message(const sensor_msgs::msg::Imu&, const PlatformProfile&, std::size_t)`
2. Optionally compose `ImuValidator` with custom scenario logic.
3. Execute with `ValidationScenarioRunner::run(...)`.
4. Register using `ExtensionRegistry<IScenarioValidator, ...>` if you want plugin-like instantiation.

### Add a new data source

1. Implement `IImuDataSource`:
   - `name() const`
   - `load_messages() const`
2. Return ordered `sensor_msgs::msg::Imu` samples suitable for replay/validation.
3. Plug into `ValidationScenarioRunner` directly, or register with `ExtensionRegistry<IImuDataSource, ...>`.
4. Add tests for ordering, boundary conditions, and malformed input handling.


### Field-focused checks included by default

- `stationary_gyro_drift_proxy`: warns when acceleration magnitude is near stationary gravity but gyro norm exceeds configured stationary threshold.
- `orientation_covariance_magnetic_anomaly_proxy`: warns when orientation covariance diagonal exceeds a configured threshold, useful as a proxy for magnetic disturbance in fused orientation pipelines.

> Note: `sensor_msgs::msg::Imu` does not contain raw magnetometer vectors. Direct magnetic anomaly detection requires magnetometer data (`sensor_msgs::msg::MagneticField`) or fused-state diagnostics. The provided check is intentionally labeled as a proxy.

## Example device-oriented scenarios

Additional practical examples for common target IMU classes are available in:

- `docs/examples/mpu6050_programmatic_validation_example.cpp`
- `docs/examples/vn200_programmatic_validation_example.cpp`
- `docs/examples/live_serial_imu_data_source_example.cpp`

See the wiki sections **"Device-oriented examples (MPU-6050 and VN-200)"** and **"Interfacing real physical IMUs (serial / USB / I2C)"** for usage notes.

## Wiki and practical test playbook

For a wiki-style, operator-focused guide (including known IMU test patterns and a catalog of implemented tests), see:

- `docs/IMU_VALIDATOR_WIKI.md`

## Structured validation output format (example)

`ValidationScenarioRunner::run(...)` returns a `ScenarioValidationResult` that can be serialized to JSON/YAML by callers. Example JSON-like output:

```json
{
  "data_source_name": "bag:imu_flight_01",
  "profile_name": "high_speed_asv",
  "aggregate_status": "WARN",
  "total_messages": 3,
  "pass_messages": 2,
  "warn_messages": 1,
  "fail_messages": 0,
  "message_results": [
    {
      "message_index": 0,
      "aggregate_status": "PASS",
      "checks": [
        {
          "check_name": "angular_velocity_limit",
          "status": "PASS",
          "measured_values": [1.2, -0.4, 0.1],
          "expected_range": "abs(value) <= 20.0 rad/s",
          "message": "Angular velocity within configured limit"
        }
      ]
    },
    {
      "message_index": 1,
      "aggregate_status": "WARN",
      "checks": [
        {
          "check_name": "timestamp_age",
          "status": "WARN",
          "measured_values": [0.14],
          "expected_range": "age <= 0.10 s",
          "message": "Timestamp is older than profile target"
        }
      ]
    }
  ]
}
```

This structure mirrors framework models:

- `ValidationResult`
- `MessageValidationResult`
- `ScenarioValidationResult`
