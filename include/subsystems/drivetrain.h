#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "command/runCommand.h"
#include "command/subsystem.h"
#include "config.h"
#include "robot.h"

struct DriveVelocities {
  double linearVelocity = 0.0;
  double angularVelocity = 0.0;
};

class Drivetrain : public Subsystem {
 public:
  explicit Drivetrain(const std::vector<std::int8_t>& leftPorts,
                      const std::vector<std::int8_t>& rightPorts)
      : leftMotors(leftPorts), rightMotors(rightPorts) {}

  void periodic() override {}

  void setPct(double leftPct, double rightPct) {
    this->leftMotors.move_voltage(leftPct * 120);
    this->rightMotors.move_voltage(rightPct * 120);
  }

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

  double getForwardEncoderInches() const {
    double sumDeg = 0.0;
    int count = 0;
    for (const double deg : leftMotors.get_position_all()) {
      if (std::isfinite(deg)) {
        sumDeg += deg;
        ++count;
      }
    }
    for (const double deg : rightMotors.get_position_all()) {
      if (std::isfinite(deg)) {
        sumDeg += deg;
        ++count;
      }
    }
    if (count == 0) return lastForwardInches;

    const double avgDeg = sumDeg / count;
    lastForwardInches =
        (avgDeg / 360.0) *
        (M_PI * Robot::WHEEL_DIAMETER * Robot::DRIVE_GEAR_RATIO) *
        Robot::FORWARD_ENCODER_SCALE;
    return lastForwardInches;
  }

  ~Drivetrain() override = default;

 private:
  pros::MotorGroup leftMotors;
  pros::MotorGroup rightMotors;
  mutable double lastForwardInches = 0.0;
};