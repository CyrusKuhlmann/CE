#pragma once

#include "command/runCommand.h"
#include "command/subsystem.h"

class Lift : public Subsystem {
 public:
  Lift();
  void set(double power);
  void stop();
  double getPosition();
  void resetPosition();

 private:
  pros::Motor liftMotor;
};