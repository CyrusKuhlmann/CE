#pragma once

#include "Eigen/Dense"
#include "readings.h"
#include "units/units.hpp"

using namespace units;

class ILocalizer {
 public:
  virtual ~ILocalizer() = default;
  virtual void predict(double delta_fwd, double delta_lat,
                       double delta_theta) = 0;
  virtual void updateDistance(const DistanceSensorReading& measurement) = 0;
  virtual void updateVision(const VisionSensorReading& measurement) = 0;

  virtual Eigen::Vector3d getState() = 0;
  virtual void setState(const Eigen::Vector3d& newState) = 0;
};
