#pragma once

#include <cmath>
#include <memory>

#include "Eigen/Dense"
#include "command/runCommand.h"
#include "command/subsystem.h"
#include "config.h"
#include "localization/localizer.h"
#include "localization/odometry.h"
#include "localization/ukf.h"
#include "robot.h"

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
        vision(visionPort) {
    if (CONFIG::USE_UKF) {
      // TODO: UKF is not yet implemented; fall back to odometry for now.
      localizer = std::make_unique<Odometry>();
    } else {
      localizer = std::make_unique<Odometry>();
    }
  }

  // Reads the tracking sensors and feeds the localizer once per scheduler
  // frame.
  void periodic() override {
    const double fwd = ticksToInches(forwardRot.get_position());
    const double lat = ticksToInches(lateralRot.get_position());
    const double theta = imu.get_rotation() * M_PI / 180.0;

    if (firstRun) {
      prevFwd = fwd;
      prevLat = lat;
      prevTheta = theta;
      firstRun = false;
      return;
    }

    const double deltaFwd = fwd - prevFwd;
    const double deltaLat = lat - prevLat;
    const double deltaTheta = theta - prevTheta;

    prevFwd = fwd;
    prevLat = lat;
    prevTheta = theta;

    position += deltaFwd;

    localizer->predict(deltaFwd, deltaLat, deltaTheta);
  }

  double getX() { return localizer->getState().x(); }
  double getY() { return localizer->getState().y(); }
  double getTheta() { return localizer->getState().z(); }
  Eigen::Vector3d getPose() { return localizer->getState(); }

  // Total forward distance travelled, in inches.
  double getPosition() const { return position; }

  void setPose(const Eigen::Vector3d& pose) { localizer->setState(pose); }

  ILocalizer* getLocalizer() { return localizer.get(); }

  ~Localization() override = default;

 private:
  static double ticksToInches(std::int32_t centidegrees) {
    return (centidegrees / 100.0) / 360.0 *
           (M_PI * Robot::TRACKING_WHEEL_DIAMETER);
  }

  pros::IMU imu;
  pros::Rotation lateralRot;
  pros::Rotation forwardRot;
  pros::Distance frontDist;
  pros::Distance backDist;
  pros::Distance leftDist;
  pros::Distance rightDist;
  pros::AIVision vision;

  std::unique_ptr<ILocalizer> localizer;

  bool firstRun = true;
  double prevFwd = 0.0;
  double prevLat = 0.0;
  double prevTheta = 0.0;
  double position = 0.0;
};