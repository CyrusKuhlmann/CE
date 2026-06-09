#pragma once

#include <cstdint>
#include <vector>

#include "Eigen/Dense"

// ─────────────────────────────────────────────────────────────────────────────
// Port map — change these constants to match your robot's wiring.
// Smart-port numbers are 1-21. ADI ports use 'A'-'H'.
// Negative motor ports reverse the motor direction.
// ─────────────────────────────────────────────────────────────────────────────
namespace Ports {

// ── Drivetrain ───────────────────────────────────────────────────────────────
// Add or remove ports here to change the number of motors per side.
// Negative ports reverse the motor's direction.
inline const std::vector<std::int8_t> DRIVE_LEFT = {1, 20};
inline const std::vector<std::int8_t> DRIVE_RIGHT = {-15, -16};

// ── Localization sensors ─────────────────────────────────────────────────────
constexpr std::int8_t IMU = 11;
constexpr std::int8_t LATERAL_ROT = 13;
constexpr std::int8_t FORWARD_ROT = NULL;
constexpr std::int8_t DIST_FRONT = NULL;
constexpr std::int8_t DIST_BACK = NULL;
constexpr std::int8_t DIST_LEFT = NULL;
constexpr std::int8_t DIST_RIGHT = NULL;
constexpr std::int8_t VISION = NULL;

// ── Lift ─────────────────────────────────────────────────────────────────────
constexpr std::int8_t LIFT_MOTOR = NULL;

}  // namespace Ports

namespace Robot {
constexpr double WHEEL_DIAMETER = 4.0;
constexpr double TRACKING_WHEEL_DIAMETER = 2.0;
constexpr double TRACK_WIDTH = 9.9;
constexpr double MAX_SPEED = 200.0;

constexpr double DRIVE_GEAR_RATIO = 1.0;

// Empirical calibration for the motor-encoder forward distance. A clean
// direct-drive 4in setup should compute correctly with this at 1.0, but a test
// move that physically travelled ~11in reported 28.13in of encoder distance, so
// the reading is scaled by the measured ratio. To re-tune: push the robot a
// known distance, read the on-screen "Forward" value, and set this to
// actual / reported. Update whenever the drivetrain wheels or cartridges
// change.
constexpr double FORWARD_ENCODER_SCALE = 11.0 / 28.13;  // ≈ 0.391

constexpr double fwd_wheel_right_offset = 0.0;
constexpr double lat_wheel_fwd_offset = -9.5;

Eigen::Vector2d front_sensor_offset{0.0, 0.0};
Eigen::Vector2d right_sensor_offset{0.0, 0.0};
Eigen::Vector2d back_sensor_offset{0.0, 0.0};
Eigen::Vector2d left_sensor_offset{0.0, 0.0};

enum class SensorSide { FRONT = 0, RIGHT = 1, BACK = 2, LEFT = 3 };
}  // namespace Robot