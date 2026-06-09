#pragma once

#include <cmath>
#include <memory>

#include "Eigen/Dense"
#include "command/runCommand.h"
#include "command/subsystem.h"
#include "config.h"
#include "localization/PF.h"
#include "localization/localizer.h"
#include "localization/odometry.h"
#include "localization/ukf.h"
#include "robot.h"
#include "subsystems/drivetrain.h"

class Localization : public Subsystem {
 public:
  Localization(const std::int8_t imuPort, const std::int8_t lateralRotPort,
               const std::int8_t forwardRotPort,
               const std::int8_t frontDistPort, const std::int8_t backDistPort,
               const std::int8_t leftDistPort, const std::int8_t rightDistPort,
               const std::int8_t visionPort, Drivetrain* drive = nullptr)
      : imu(imuPort),
        lateralRot(lateralRotPort),
        forwardRot(forwardRotPort),
        frontDist(frontDistPort),
        backDist(backDistPort),
        leftDist(leftDistPort),
        rightDist(rightDistPort),
        vision(visionPort),
        drivetrain(drive) {
    switch (CONFIG::LOCALIZER) {
      case CONFIG::LocalizerType::PARTICLE_FILTER:
        localizer = std::make_unique<PF>();
        break;
      case CONFIG::LocalizerType::UKF:
        localizer = std::make_unique<UKF>();
        break;
      case CONFIG::LocalizerType::ODOMETRY:
      default:
        localizer = std::make_unique<Odometry>();
        break;
    }
  }

  // Reads the tracking sensors and feeds the localizer once per scheduler
  // frame.
  void periodic() override {
    const double fwd = readForward();
    const double lat = ticksToInches(lateralRot.get_position());
    const double theta = imu.get_rotation() * M_PI / 180.0;

    // IMU returns PROS_ERR_F (inf) while calibrating; skip until it's valid.
    if (!std::isfinite(theta)) return;

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

    // Fuse the wall-distance sensors for any filter that consumes them. Plain
    // odometry ignores these updates, so only read the sensors when used.
    if (CONFIG::LOCALIZER != CONFIG::LocalizerType::ODOMETRY) {
      feedDistance(frontDist, Robot::SensorSide::FRONT);
      feedDistance(backDist, Robot::SensorSide::BACK);
      feedDistance(leftDist, Robot::SensorSide::LEFT);
      feedDistance(rightDist, Robot::SensorSide::RIGHT);
    }
  }

  double getX() { return localizer->getState().x(); }
  double getY() { return localizer->getState().y(); }
  double getTheta() { return localizer->getState().z(); }
  Eigen::Vector3d getPose() { return localizer->getState(); }

  // Total forward distance travelled, in inches.
  double getPosition() const { return position; }

  void setPose(const Eigen::Vector3d& pose) { localizer->setState(pose); }

  ILocalizer* getLocalizer() { return localizer.get(); }

  pros::IMU& getIMU() { return imu; }
  pros::Rotation& getLateralRot() { return lateralRot; }
  pros::Rotation& getForwardRot() { return forwardRot; }
  pros::Distance& getFrontDist() { return frontDist; }
  pros::Distance& getBackDist() { return backDist; }
  pros::Distance& getLeftDist() { return leftDist; }
  pros::Distance& getRightDist() { return rightDist; }
  pros::AIVision& getVision() { return vision; }

  ~Localization() override = default;

 private:
  static double ticksToInches(std::int32_t centidegrees) {
    return (centidegrees / 100.0) / 360.0 *
           (M_PI * Robot::TRACKING_WHEEL_DIAMETER);
  }

  // Forward distance in inches, sourced either from the dedicated forward
  // tracking-wheel rotation sensor or, when USE_MOTOR_FORWARD_ENCODER is set
  // and a drivetrain is available, the average of the drive motor encoders.
  double readForward() {
    if (CONFIG::USE_MOTOR_FORWARD_ENCODER && drivetrain != nullptr) {
      return drivetrain->getForwardEncoderInches();
    }
    return ticksToInches(forwardRot.get_position());
  }

  // Reads one Distance sensor (millimetres) and feeds it to the localizer as an
  // inches-valued reading. Invalid readings are skipped; the localizer applies
  // confidence weighting and innovation gating internally.
  void feedDistance(pros::Distance& sensor, Robot::SensorSide side) {
    const std::int32_t mm = sensor.get();
    if (mm == PROS_ERR || mm <= 0) return;
    const std::int32_t conf = sensor.get_confidence();
    const DistanceSensorReading reading{
        side, mm / 25.4, conf == PROS_ERR ? 0.0 : static_cast<double>(conf)};
    localizer->updateDistance(reading);
  }

  pros::IMU imu;
  pros::Rotation lateralRot;
  pros::Rotation forwardRot;
  pros::Distance frontDist;
  pros::Distance backDist;
  pros::Distance leftDist;
  pros::Distance rightDist;
  pros::AIVision vision;

  Drivetrain* drivetrain;

  std::unique_ptr<ILocalizer> localizer;

  bool firstRun = true;
  double prevFwd = 0.0;
  double prevLat = 0.0;
  double prevTheta = 0.0;
  double position = 0.0;
};