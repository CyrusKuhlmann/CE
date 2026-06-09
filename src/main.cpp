#include "main.h"

CommandController primary(pros::E_CONTROLLER_MASTER);

Drivetrain* drivetrain;
Localization* localization;
Command* autonomousCommand;

[[noreturn]] void screen_update_loop() {
  while (true) {
    auto start_time = pros::millis();
    pros::lcd::print(1, "Forward: %.2f", drivetrain->getForwardEncoderInches());
    // x, y, theta
    pros::lcd::print(2, "X: %.2f Y: %.2f T: %.2f", localization->getX(),
                     localization->getY(), localization->getTheta());

    pros::c::task_delay_until(&start_time, 10);
  }
}

/**
 * @brief This function runs the update scheduler at each frame with a
 * consistent schedule
 *
 * @warning This function or alternative similar to it must be running to ensure
 * the \refitem CommandScheduler is run
 */
[[noreturn]] void update_loop() {
  // Loop forever
  while (true) {
    // Store the start time
    auto start_time = pros::millis();

    // Run the command scheduler
    // This might be an expensive(Time wise) computation
    CommandScheduler::run();

    // Use delay until if this computation ends up being expensive, keeping loop
    // time in check
    pros::c::task_delay_until(&start_time, 10);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();

  drivetrain = new Drivetrain(Ports::DRIVE_LEFT, Ports::DRIVE_RIGHT);
  CommandScheduler::registerSubsystem(drivetrain, drivetrain->tank(primary));

  localization =
      new Localization(Ports::IMU, Ports::LATERAL_ROT, Ports::FORWARD_ROT,
                       Ports::DIST_FRONT, Ports::DIST_BACK, Ports::DIST_LEFT,
                       Ports::DIST_RIGHT, Ports::VISION, drivetrain);

  CommandScheduler::registerSubsystem(localization,
                                      new RunCommand([] {}, {localization}));

  autonomousCommand = AutonomousCommands::getAuton();

  pros::Task commandSchedulerTask(update_loop);
  localization->getLateralRot().reset_position();
  localization->getForwardRot().reset_position();
  localization->getIMU().reset(true);
  pros::delay(500);

  // The screen task copies Eigen Vector3d poses by value and does multiple
  // %f LVGL prints per frame; the default 8KB stack overflows (manifesting as
  // nan readings and a crashed task), so give it double the default depth.
  pros::Task screenUpdateTask(screen_update_loop, TASK_PRIORITY_DEFAULT,
                              TASK_STACK_DEPTH_DEFAULT * 10, "Screen Update");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous
 * selector on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own
 * task with the default priority and stack size whenever the robot is enabled
 * via the Field Management System or the VEX Competition Switch in the
 * autonomous mode. Alternatively, this function may be called in initialize
 * or opcontrol for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start
 * it from where it left off.
 */
void autonomous() { CommandScheduler::schedule(autonomousCommand); }

/**
 * Runs the operator control code. This function will be started in its own
 * task with the default priority and stack size whenever the robot is enabled
 * via the Field Management System or the VEX Competition Switch in the
 * operator control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart
 * the task, not resume it from where it left off.
 */
void opcontrol() { autonomousCommand->cancel(); }