#pragma once

#include "command/runCommand.h"
#include "command/subsystem.h"

class Drivetrain : public Subsystem {
 public:
  explicit Drivetrain(const std::initializer_list<std::int8_t> leftPorts,
                      const std::initializer_list<std::int8_t> rightPorts)
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

  ~Drivetrain() override = default;

 private:
  pros::MotorGroup leftMotors;
  pros::MotorGroup rightMotors;
};