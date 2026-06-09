#pragma once

#include "autonomous/autons.h"
#include "game.h"

namespace CONFIG {

// Which localization filter the robot runs. ODOMETRY is plain dead-reckoning;
// UKF and PARTICLE_FILTER additionally fuse the wall-distance sensors.
enum class LocalizerType { ODOMETRY, UKF, PARTICLE_FILTER };

// Forward-distance encoding source for localization.
//   false -> use the dedicated forward tracking-wheel rotation sensor.
//   true  -> use the average of the drivetrain motor encoders as the forward
//            tracking wheel (no dedicated forward rotation sensor required).
constexpr bool USE_MOTOR_FORWARD_ENCODER = true;

constexpr Alliance ALLIANCE = RED;
constexpr Auton AUTON = Auton::CYRUS_SKILLS;
constexpr LocalizerType LOCALIZER = LocalizerType::ODOMETRY;

}  // namespace CONFIG
