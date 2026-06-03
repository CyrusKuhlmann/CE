#pragma once

#include "robot.h"

struct DistanceSensorReading {
  Robot::SensorSide side;
  double distance;
  double confidence;
};

struct VisionSensorReading {
  Eigen::Vector2d globalPosition;
  double distance;
  double heading;
  double confidence;
};