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

struct DriveVelocities {
  double linearVelocity = 0.0;
  double angularVelocity = 0.0;
};

class Drivetrain : public Subsystem {
 public:
  explicit Drivetrain(const std::initializer_list<std::int8_t> leftPorts,
                      const std::initializer_list<std::int8_t> rightPorts,
                      const std::int8_t forwardTrackerPort,
                      const std::int8_t lateralTrackerPort,
                      const std::int8_t imuPort)
      : leftMotors(leftPorts),
        rightMotors(rightPorts),
        forwardTracker(forwardTrackerPort),
        lateralTracker(lateralTrackerPort),
        imu(imuPort) {
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
    const double fwd = ticksToInches(forwardTracker.get_position());
    const double lat = ticksToInches(lateralTracker.get_position());
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

  void setPct(double leftPct, double rightPct) {
    this->leftMotors.move_voltage(leftPct * 120);
    this->rightMotors.move_voltage(rightPct * 120);
  }

  // Pose accessors backed by the active localizer, for use in commands.
  double getX() { return localizer->getState().x(); }
  double getY() { return localizer->getState().y(); }
  double getTheta() { return localizer->getState().z(); }
  Eigen::Vector3d getPose() { return localizer->getState(); }

  // Cumulative forward distance travelled, in inches.
  double getPosition() const { return position; }

  void setPose(const Eigen::Vector3d& pose) { localizer->setState(pose); }

  ILocalizer* getLocalizer() { return localizer.get(); }

  RunCommand* setPctCommand(double leftPct, double rightPct) {
    return new RunCommand(
        [this, leftPct, rightPct] { this->setPct(leftPct, rightPct); }, {this});
  }

  RunCommand* tank(pros::Controller& controller) {
    return new RunCommand(
        [this, controller]() mutable {
          this->setPct(
              controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 1.270,
              controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 1.270);
        },
        {this});
  }

  ~Drivetrain() override = default;

 private:
  // Rotation sensors report position in centidegrees.
  static double ticksToInches(std::int32_t centidegrees) {
    return (centidegrees / 100.0) / 360.0 *
           (M_PI * Robot::TRACKING_WHEEL_DIAMETER);
  }

  pros::MotorGroup leftMotors;
  pros::MotorGroup rightMotors;

  pros::Rotation forwardTracker;
  pros::Rotation lateralTracker;
  pros::IMU imu;

  std::unique_ptr<ILocalizer> localizer;

  bool firstRun = true;
  double prevFwd = 0.0;
  double prevLat = 0.0;
  double prevTheta = 0.0;
  double position = 0.0;
};