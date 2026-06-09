#pragma once

#include "command/command.h"
#include "subsystems/drivetrain.h"
#include "units/units.hpp"

using namespace units;

class Ramsete : public Command {
 public:
 private:
  Drivetrain* drivetrain;

  float zeta;
  float beta;

  double startTime;

  double lastLeft = 0.0;
  double lastRight = 0.0;

  DriveVelocities lastVelocities{0.0, 0.0};
};