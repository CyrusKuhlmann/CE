#pragma once

#include "Eigen/Dense"
#include "localizer.h"
#include "odometry.h"
#include "readings.h"
#include "robot.h"

namespace UKFConfig {
constexpr double alpha = 1e-3;
constexpr double beta = 2.0;
constexpr double kappa = 0.0;

constexpr double sigmaX = 0.1;
constexpr double sigmaY = 0.1;
constexpr double sigmaTheta = 0.02;

constexpr double sigmaMove = 1.0;

constexpr int N = 3;  // state dimension

constexpr double mahal_gate_sigma = 3.0;
};  // namespace UKFConfig

class UKF : public ILocalizer {
 public:
  UKF();
  void predict(double delta_fwd, double delta_lat, double delta_theta) override;
  void updateDistance(const DistanceSensorReading& measurement) override;
  void updateVision(const VisionSensorReading& measurement) override;

  Eigen::Vector3d getState() override;
  void setState(const Eigen::Vector3d& newState) override;

 private:
  Eigen::Vector3d state;  // [x, y, theta]
  Eigen::Matrix3d covariance;
  Odometry odometry;
};