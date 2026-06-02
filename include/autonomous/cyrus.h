#pragma once

#include "command/includes.h"

namespace Cyrus {

static Command* Skills() { return new Sequence({}); }
static Command* Left() { return new Sequence({}); }
static Command* Right() { return new Sequence({}); }
static Command* SoloAWP() { return new Sequence({}); }

}  // namespace Cyrus