#pragma once

#include "Eigen/Dense"
#include "command/runCommand.h"
#include "command/subsystem.h"
#include "config.h"

class Localization : public Subsystem {
 public:
  Localization(const std::int8_t imuPort, const std::int8_t lateralRotPort,
               const std::int8_t forwardRotPort,
               const std::int8_t frontDistPort, const std::int8_t backDistPort,
               const std::int8_t leftDistPort, const std::int8_t rightDistPort,
               const std::int8_t visionPort)
      : imu(imuPort),
        lateralRot(lateralRotPort),
        forwardRot(forwardRotPort),
        frontDist(frontDistPort),
        backDist(backDistPort),
        leftDist(leftDistPort),
        rightDist(rightDistPort),
        vision(visionPort) {}

  void periodic() override {
    if (CONFIG::USE_UKF) {
    } else {
    }
  };
  double getX();
  double getY();
  double getHeading();
  Eigen::Vector3d getPose();

  ~Localization() override = default;

 private:
  pros::IMU imu;
  pros::Rotation lateralRot;
  pros::Rotation forwardRot;
  pros::Distance frontDist;
  pros::Distance backDist;
  pros::Distance leftDist;
  pros::Distance rightDist;
  pros::AIVision vision;
};