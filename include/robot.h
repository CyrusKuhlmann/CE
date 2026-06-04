#pragma once

#include "Eigen/Dense"

namespace Robot {
constexpr double WHEEL_DIAMETER = 4.0;
constexpr double TRACKING_WHEEL_DIAMETER = 2.0;
constexpr double TRACK_WIDTH = 12.0;
constexpr double MAX_SPEED = 200.0;

constexpr double fwd_wheel_right_offset = 0.0;
constexpr double lat_wheel_fwd_offset = 0.0;

Eigen::Vector2d front_sensor_offset{0.0, 0.0};
Eigen::Vector2d right_sensor_offset{0.0, 0.0};
Eigen::Vector2d back_sensor_offset{0.0, 0.0};
Eigen::Vector2d left_sensor_offset{0.0, 0.0};

enum class SensorSide { FRONT = 0, RIGHT = 1, BACK = 2, LEFT = 3 };
}  // namespace Robot