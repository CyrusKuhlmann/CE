#pragma once

#include "autons.h"
#include "command/command.h"
#include "config.h"
#include "cyrus.h"

class AutonomousCommands {
 public:
  static Command* getAuton() {
    switch (CONFIG::AUTON) {
      case CYRUS_SKILLS:
        return Cyrus::Skills();
      case CYRUS_LEFT:
        return Cyrus::Left();
      case CYRUS_RIGHT:
        return Cyrus::Right();
      case CYRUS_SOLO_AWP:
        return Cyrus::SoloAWP();
      default:
        return new InstantCommand(
            []() { std::cout << "No auton" << std::endl; }, {});
    }
  }
};