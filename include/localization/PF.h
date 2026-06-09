#pragma once

/*
 * PF - stub localizer.
 *
 * The Monte-Carlo Particle Filter implementation has been removed. This class
 * satisfies the ILocalizer interface using the same arc-integration odometry
 * model as Odometry so the Localization subsystem compiles and runs unchanged.
 * updateDistance() and updateVision() are intentional no-ops.
 */

#include <cmath>

#include "Eigen/Dense"
#include "localizer.h"
#include "robot.h"

class PF : public ILocalizer {
 public:
  PF() = default;

  void predict(double deltaFwd, double deltaLat, double deltaTheta) override {
    const double dFwd = deltaFwd + Robot::fwd_wheel_right_offset * deltaTheta;
    const double dLat = deltaLat - Robot::lat_wheel_fwd_offset * deltaTheta;
    const double th0 = odomTheta_;

    double dx, dy;
    if (std::abs(deltaTheta) < 1e-9) {
      dx = dFwd * std::sin(th0) + dLat * std::cos(th0);
      dy = dFwd * std::cos(th0) - dLat * std::sin(th0);
    } else {
      const double inv = 1.0 / deltaTheta;
      dx = inv * (dFwd * (std::cos(th0) - std::cos(th0 + deltaTheta)) +
                  dLat * (std::sin(th0 + deltaTheta) - std::sin(th0)));
      dy = inv * (dFwd * (std::sin(th0 + deltaTheta) - std::sin(th0)) -
                  dLat * (std::cos(th0) - std::cos(th0 + deltaTheta)));
    }

    state_(0) += dx;
    state_(1) += dy;
    odomTheta_ += deltaTheta;
    odomTheta_ = std::atan2(std::sin(odomTheta_), std::cos(odomTheta_));
    state_(2) = odomTheta_;
  }

  void updateDistance(const DistanceSensorReading&) override {}
  void updateVision(const VisionSensorReading&) override {}

  Eigen::Vector3d getState() override { return state_; }
  void setState(const Eigen::Vector3d& newState) override {
    state_ = newState;
    odomTheta_ = newState(2);
  }

 private:
  Eigen::Vector3d state_ = Eigen::Vector3d::Zero();
  double odomTheta_ = 0.0;
};