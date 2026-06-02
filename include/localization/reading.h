#pragma once

#include "robot.h"
#include "units/units.hpp"

using namespace units;

struct DistanceSensorReading {
  Robot::SensorSide side;
  QLength distance;
  double confidence;
};

struct VisionSensorReading {
  Eigen::Vector2d globalPosition;
  QLength distance;
  Angle heading;
  double confidence;
};