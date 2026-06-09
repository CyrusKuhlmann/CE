#pragma once

#include "command/includes.h"
#include "commands/move.h"
#include "subsystems/drivetrain.h"
#include "subsystems/localization.h"

namespace Cyrus {

static Command* Skills() {
  return new Sequence({
      new Move(drivetrain, localization, 20.0, 0.05, {100.0, 50.0}, 0.0, true,
               0.0, 0.0),
  });
}
static Command* Left() { return new Sequence({}); }
static Command* Right() { return new Sequence({}); }
static Command* SoloAWP() { return new Sequence({}); }

}  // namespace Cyrus