#pragma once

#include "Eigen/Dense"
#include "units/units.hpp"

using namespace units;

namespace Robot {
constexpr QLength WHEEL_DIAMETER = 4.0_in;

double fwd_wheel_right_offset = 0.0;
double lat_wheel_fwd_offset = 0.0;

Eigen::Vector2d front_sensor_offset{0.0, 0.0};
Eigen::Vector2d right_sensor_offset{0.0, 0.0};
Eigen::Vector2d back_sensor_offset{0.0, 0.0};
Eigen::Vector2d left_sensor_offset{0.0, 0.0};

enum class SensorSide { FRONT = 0, RIGHT = 1, BACK = 2, LEFT = 3 };
}  // namespace Robot