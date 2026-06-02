#pragma once

#include "Eigen/Dense"
#include "reading.h"
#include "robot.h"
#include "units/units.hpp"

using namespace units;

namespace UKFConfig {
constexpr double alpha = 1e-3;
constexpr double beta = 2.0;
constexpr double kappa = 0.0;

constexpr QLength sigma_proc_x = 0.1_in;
constexpr QLength sigma_proc_y = 0.1_in;
constexpr Angle sigma_proc_theta = 0.02_rad;

constexpr QLength sigma_dist = 1.0_in;

constexpr int N = 3;  // state dimension

constexpr double mahal_gate_sigma = 3.0;
};  // namespace UKFConfig

class UKF {
 public:
  UKF();
  void predict(QLength delta_fwd, QLength delta_lat, Angle delta_theta);
  void updateDistance(const DistanceSensorReading& measurement);
  void updateVision(const VisionSensorReading& measurement);

  Eigen::Vector3d getState();
  void setState(const Eigen::Vector3d& newState);
  Eigen::Matrix3d getCovariance();

 private:
  Eigen::Vector3d state;  // [x, y, theta]
  Eigen::Matrix3d covariance;
};