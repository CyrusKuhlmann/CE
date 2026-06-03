#pragma once

#include "Eigen/Dense"
#include "localizer.h"
#include "robot.h"
#include "subsystems/drivetrain.h"

class Odometry : public ILocalizer {
 public:
  Odometry() {};
  void predict(double deltaFwd, double deltaLat, double deltaTheta) override {
    // Remove motion caused by wheel offsets
    const double dFwd = deltaFwd + Robot::fwd_wheel_right_offset * deltaTheta;
    const double dLat = deltaLat - Robot::lat_wheel_fwd_offset * deltaTheta;

    const double theta0 = odomTheta;

    Eigen::Vector2d deltaGlobal;

    if (std::abs(deltaTheta) < 1e-9) {
      deltaGlobal.x() = dLat * std::cos(theta0) + dFwd * std::sin(theta0);

      deltaGlobal.y() = -dLat * std::sin(theta0) + dFwd * std::cos(theta0);

    } else {
      const double deltaThetaSin = std::sin(deltaTheta);
      const double deltaThetaCos = std::cos(deltaTheta);

      Eigen::Vector2d deltaPose{dLat, dFwd};

      Eigen::Matrix2d R;
      R << std::cos(theta0), std::sin(theta0), -std::sin(theta0),
          std::cos(theta0);

      Eigen::Matrix2d V;
      V << deltaThetaSin / deltaTheta, (1.0 - deltaThetaCos) / deltaTheta,
          -(1.0 - deltaThetaCos) / deltaTheta, deltaThetaSin / deltaTheta;

      deltaGlobal = R * V * deltaPose;
    }

    pose.x() += deltaGlobal.x();
    pose.y() += deltaGlobal.y();

    odomTheta += deltaTheta;

    odomTheta = std::atan2(std::sin(odomTheta), std::cos(odomTheta));

    pose.z() = odomTheta;
  }

  void updateDistance(const DistanceSensorReading& measurement) override {}
  void updateVision(const VisionSensorReading& measurement) override {}

  Eigen::Vector3d getState() override { return pose; }
  void setState(const Eigen::Vector3d& newState) override { pose = newState; }

 private:
  Eigen::Vector3d pose = Eigen::Vector3d::Zero();
  double odomTheta = 0.0;
};