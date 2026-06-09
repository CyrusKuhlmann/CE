#pragma once

#include "command/command.h"
#include "config.h"
#include "controllers/feedback/pid.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/localization.h"

/**
 * Turn on point code
 */
class Rotate : public Command {
 private:
  Drivetrain* drivetrain;
  Localization* localization;
  PID pid{PIDConstants::turnPID};
  double static_voltage;
  bool finish{true};

  double angle = 0.0;

 public:
  Rotate(Drivetrain* drivetrain, Localization* localization, const double angle,
         const double static_voltage = 0.0, const bool finish = true)
      : drivetrain(drivetrain),
        localization(localization),
        angle(angle),
        static_voltage(static_voltage),
        finish(finish) {}

  void initialize() override {
    this->pid = PID(PIDConstants::turnPID);
    this->pid.reset();
    this->pid.setTarget(angle);
    this->pid.setTurnPid(true);
  }

  void execute() override {
    const auto output =
        std::ranges::clamp(pid.update(localization->getTheta()), -1.0, 1.0);
    drivetrain->setPct(static_voltage - output, static_voltage + output);
  }

  bool isFinished() override {
    return finish && std::abs(localization->getTheta() - pid.getTarget()) < 2.0;
  }

  std::vector<Subsystem*> getRequirements() override { return {drivetrain}; }

  ~Rotate() override = default;
};