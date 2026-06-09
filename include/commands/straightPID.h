#pragma once

#include <cmath>

#include "command/command.h"
#include "controllers/feedback/pid.h"
#include "pros/rtos.hpp"
#include "subsystems/Drivetrain.h"
#include "subsystems/localization.h"

/**
 * @brief Drives the robot straight for a target distance using two PID loops.
 *
 * @details A lateral (distance) PID drives the robot the requested distance
 * forward, while an angular (heading) PID holds the robot's heading constant so
 * that the motion stays straight. The angular correction is applied
 * differentially on top of the shared lateral output.
 */
class StraightPID : public Command {
 private:
  Drivetrain* drivetrain;
  Localization* localization;

  double distance = 0.0;
  double targetDistance = 0.0;
  double targetAngle = 0.0;

  static constexpr double DISTANCE_TOLERANCE = 0.5;

  double timeout = 0.0;

  double startTime = 0.0;
  double startDistance = 0.0;

  PID distancePID;
  PID anglePID;

 public:
  /**
   * @brief Construct a StraightPID command.
   *
   * @param drivetrain Drivetrain subsystem to drive.
   * @param localization Localization subsystem providing distance and heading.
   * @param distance Target distance to travel forward, in inches.
   * @param timeout Maximum run time in seconds. 0 disables the timeout.
   */
  StraightPID(Drivetrain* drivetrain, Localization* localization,
              double distance, double timeout = 0.0)
      : drivetrain(drivetrain),
        localization(localization),
        distance(distance),
        timeout(timeout) {}

  void initialize() override {
    startTime = pros::millis();
    startDistance = localization->getPosition();
    targetDistance = startDistance + distance;

    targetAngle = localization->getTheta();

    distancePID = PIDConstants::distancePID;
    distancePID.setTurnPid(false);
    distancePID.setTarget(targetDistance);
    distancePID.reset();

    anglePID = PIDConstants::turnPID;
    anglePID.setTurnPid(true);
    anglePID.setTarget(targetAngle);
    anglePID.reset();
  }

  void execute() override {
    const double currentDistance = localization->getPosition();
    const double currentAngle = localization->getTheta();

    const double lateralVoltage = distancePID.update(currentDistance);
    const double angularVoltage = anglePID.update(currentAngle);

    const double leftVoltage = lateralVoltage - angularVoltage;
    const double rightVoltage = lateralVoltage + angularVoltage;

    drivetrain->setPct(leftVoltage, rightVoltage);
  }

  void end(bool interrupted) override { drivetrain->setPct(0.0, 0.0); }

  bool isFinished() override {
    if (timeout > 0.0 && (pros::millis() - startTime) >= timeout * 1000.0) {
      return true;
    }

    const double distanceError = targetDistance - localization->getPosition();
    return std::abs(distanceError) <= DISTANCE_TOLERANCE;
  }

  std::vector<Subsystem*> getRequirements() override { return {drivetrain}; }
};
