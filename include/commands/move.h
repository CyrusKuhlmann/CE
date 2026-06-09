#pragma once

#include "command/command.h"
#include "controllers/feedback/pid.h"
#include "controllers/feedforward/feedforward.h"
#include "motionProfiling/MotionProfile1D.h"
#include "robot.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/localization.h"

class Move : public Command {
 private:
  Drivetrain* drivetrain;
  Localization* localization;
  MotionProfile1D profile;

  double startTime = 0.0;
  double targetAngle = 0.0;

  double distance = 0.0;
  double curvature = 0.0;

  double startDistance = 0.0;

  bool useTurnPID = true;

  PID turnPID;
  PID distancePID;

 public:
  Move(Drivetrain* drivetrain, Localization* localization, double distance,
       double curvature, const MotionProfileConstraints& constraints,
       double targetAngle, bool useTurnPID = true,
       const double initialVelocity = 0.0, const double finalVelocity = 0.0)
      : drivetrain(drivetrain),
        localization(localization),
        profile(distance, initialVelocity, finalVelocity, constraints),
        distance(distance),
        curvature(curvature),
        targetAngle(targetAngle),
        useTurnPID(useTurnPID) {
    this->useTurnPID = useTurnPID;
    turnPID.setTurnPid(true);
  };

  [[nodiscard]] double getVelocityMultiplier() const {
    if (curvature == 0.0) {
      return 1.0;
    } else {
      return 1.0 / (1.0 + std::abs(curvature * 0.5) * Robot::TRACK_WIDTH);
    }
  };

  void initialize() override {
    startTime = pros::millis();
    startDistance = localization->getPosition();
    double adjustedVelocity = this->getVelocityMultiplier() *
                              this->profile.getProfileConstraints().maxVelocity;

    this->profile.setProfileConstraints(
        {adjustedVelocity,
         this->profile.getProfileConstraints().maxAcceleration});

    this->profile.generateProfile();

    turnPID = PIDConstants::turnPID;
    turnPID.setTarget(targetAngle);
    turnPID.setTurnPid(true);

    turnPID.reset();

    distancePID = PIDConstants::distancePID;
    distancePID.setTurnPid(false);
    distancePID.reset();
  }

  void execute() override {
    const double duration = (pros::millis() - startTime) / 1000.0;

    const double acceleration = profile.getAccelerationAtTime(duration);
    const double velocity = profile.getVelocityAtTime(duration);
    const double targetDistance = profile.getDistanceAtTime(duration);
    const double currentDistance = localization->getPosition() - startDistance;

    distancePID.setTarget(targetDistance);

    const auto [linearFF, angularFF] =
        FeedforwardConstants::defaultFeedforward.calculate(
            velocity, acceleration, curvature * velocity,
            curvature * acceleration);

    const double wheelVoltage = distancePID.update(currentDistance) + linearFF;

    const double leftCurvatureAdjustment =
        (2.0 + curvature * Robot::TRACK_WIDTH) * 0.5;
    const double rightCurvatureAdjustment =
        (2.0 - curvature * Robot::TRACK_WIDTH) * 0.5;

    double leftVoltage = wheelVoltage * leftCurvatureAdjustment;
    double rightVoltage = wheelVoltage * rightCurvatureAdjustment;

    if (useTurnPID) {
      const double offset = targetDistance * curvature;
      double angleWithOffset = targetAngle + offset;
      turnPID.setTarget(angleWithOffset);

      const double turnVoltage = turnPID.update(localization->getTheta());

      leftVoltage -= turnVoltage + angularFF;
      rightVoltage += turnVoltage + angularFF;
    }

    drivetrain->setPct(leftVoltage, rightVoltage);
  }

  void end(bool interrupted) override {}

  bool isFinished() override {
    return (pros::millis() - startTime) >= profile.getTotalTime() * 1000.0;
  };

  std::vector<Subsystem*> getRequirements() override { return {drivetrain}; }
};