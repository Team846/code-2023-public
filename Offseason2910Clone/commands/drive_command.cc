#include "Offseason2910Clone/commands/drive_command.h"

#include <utility>

#include "Offseason2910Clone/subsystems/drivetrain.h"
#include "Offseason2910Clone/subsystems/swerve_module.h"
#include "transitionLib846/math.h"
#include "transitionLib846/wpilib/time.h"
#include "transitionLib846/xbox.h"


DriveCommand::DriveCommand(RobotContainer& container)
    : driver_(container.driver_),
      drivetrain_(container.drivetrain_),
      limelight_(container.limelight_),
      roller_(container.roller_) {
  AddRequirements({&drivetrain_});
  SetName("drive_command");
}

double DriveCommand::JoystickOutput(double ji) {
  ji = ji * abs(pow(ji, 1));  // ji = ji * abs(pow(ji,
                              // driver_.translation_exponent_.value() - 1));
  return ji;
}

void DriveCommand::Execute() {
  DrivetrainTarget drivetrain_target;

  // -----BUTTON MAPPINGS-----

  // Left bumper   | robot centric translation
  // Right trigger | vision aiming
  // Right bumper  | precision drive
  // RB + RT       | gp shelf pickup
  // Dpad Right    | Lock Wheels

  bool is_robot_centric = false;
  bool is_slow_drive =
      !driver_.readings().right_trigger && driver_.readings().right_bumper;

  bool is_auto_aim =
      driver_.readings().right_trigger && !driver_.readings().right_bumper;

  if (driver_.readings().b_button) {
    if (loop_b_unheld) {
      toggle_align_to_shelf = !toggle_align_to_shelf;
    }
    loop_b_unheld = false;
  } else {
    loop_b_unheld = true;
  }

  if (toggle_align_to_shelf) {
    if (roller_.Initialized()) {
      if (roller_.readings().has_piece) {
        toggle_align_to_shelf = false;
      }
    }
  }

  // -----TRANSLATION CONTROL-----

  double translate_x = transitionLib846::HorizontalDeadband(
      JoystickOutput(driver_.readings().left_stick_x),
      pow(driver_.translation_deadband_.value(), 2), 1, 1, 1);
  double translate_y = transitionLib846::HorizontalDeadband(
      JoystickOutput(driver_.readings().left_stick_y),
      pow(driver_.translation_deadband_.value(), 2), 1,
      driver_.translation_exponent_.value(), 1);

  drivetrain_target.v_x = translate_x * drivetrain_.max_speed_.value();
  drivetrain_target.v_y = translate_y * drivetrain_.max_speed_.value();

  if (driver_.readings().right_bumper && !driver_.readings().right_trigger) {
    is_slow_drive = true;
  }

  // Slow down translation if slow mode is active
  // if (is_slow_drive) {
  //   translate_x = transitionLib846::HorizontalDeadband(
  //       driver_.readings().left_stick_x *
  //       abs(driver_.readings().left_stick_x),
  //       driver_.translation_deadband_.value(), 1,
  //       driver_.translation_exponent_.value(), 1);
  //   translate_y = transitionLib846::HorizontalDeadband(
  //       driver_.readings().left_stick_y *
  //       abs(driver_.readings().left_stick_y),
  //       driver_.translation_deadband_.value(), 1,
  //       driver_.translation_exponent_.value(), 1);
  //   drivetrain_target.v_x = translate_x * drivetrain_.max_speed_.value() *
  //                           drivetrain_.slow_mode_percent_.value();
  //   drivetrain_target.v_y = translate_y * drivetrain_.max_speed_.value() *
  //                           drivetrain_.slow_mode_percent_.value();
  // }

  // Robot vs field oriented translation
  drivetrain_target.translation_reference =
      is_robot_centric ? DrivetrainTranslationReference::kRobot
                       : DrivetrainTranslationReference::kField;

  drivetrain_target.control = kOpenLoop;

  // -----STEER CONTROL-----

  double steer_x = transitionLib846::HorizontalDeadband(
      driver_.readings().right_stick_x * abs(driver_.readings().right_stick_x),
      driver_.steer_deadband_.value(), 1, driver_.steer_exponent_.value(), 1);

  // If there's any manual steer input, prioritize that over vision aiming
  if (is_auto_aim && limelight_.Initialized()) {
    drivetrain_target.rotation = DrivetrainRotationPosition(180_deg);
    if (limelight_.readings().target_exists &&
        units::math::abs(limelight_.readings().retro_distance) >
            drivetrain_.align_tolerance_.value()) {
      // Auto align (Cone)
      drivetrain_target.control = kClosedLoop;
      drivetrain_target.rotation = DrivetrainRotationPosition(180_deg);

      units::inch_t offset =
          limelight_.readings().retro_distance *
              units::math::sin(drivetrain_.readings().pose.bearing +
                               limelight_.readings().tx) -
          limelight_.subsystem()->camera_offset;

      drivetrain_target.v_x = units::feet_per_second_t(
          (units::foot_t(offset) * drivetrain_.align_gains_p_.value()).value());

      // if (!limelight_.readings().is_low) {
      //   double retro_dist_adjusted =
      //   units::foot_t((units::math::abs(limelight_.readings().retro_distance)
      //   - (1_in + limelight_.subsystem()->high_offset))).value(); if
      //   (abs(retro_dist_adjusted) >
      //   units::foot_t(drivetrain_.align_tolerance_.value()).value()*8) {
      //     drivetrain_target.v_y =
      //     -units::feet_per_second_t(retro_dist_adjusted) *
      //     drivetrain_.align_gains_p_.value() * 0.3;
      //   }
      // } else {
      //   double retro_dist_adjusted =
      //   units::foot_t((units::math::abs(limelight_.readings().retro_distance)
      //   - (1_in + limelight_.subsystem()->low_offset))).value(); if
      //   (abs(retro_dist_adjusted) >
      //   units::foot_t(drivetrain_.align_tolerance_.value()).value()*8) {
      //     drivetrain_target.v_y =
      //     -units::feet_per_second_t(retro_dist_adjusted) *
      //     drivetrain_.align_gains_p_.value() * 0.3;
      //   }
      // }

      // Wrap the value to a max of 4_fps
      if (drivetrain_target.v_x > 4_fps) {
        drivetrain_target.v_x = 4_fps;
      } else if (drivetrain_target.v_x < -4_fps) {
        drivetrain_target.v_x = -4_fps;
      }
    }
  } else if (steer_x != 0) {
    // Manual steer
    auto target = steer_x * drivetrain_.max_omega();

    // Slow down steering if slow mode is active
    // if (is_slow_drive) {
    //   target *= drivetrain_.slow_omega_percent_.value();
    // }

    drivetrain_target.rotation = DrivetrainRotationVelocity(target);
  } else {
    // Hold position
    drivetrain_target.rotation = DrivetrainRotationVelocity(0_deg_per_s);
  }

  if (is_auto_aim) {
    // if (roller_.Initialized()) {
    //   if (roller_.readings().has_piece) {
    //     drivetrain_target.rotation = DrivetrainRotationPosition(0_deg);
    //   } else {
    //     drivetrain_target.rotation = DrivetrainRotationPosition(180_deg);
    //   }
    // } else {
    //   drivetrain_target.rotation = DrivetrainRotationPosition(0_deg);
    // }
    drivetrain_target.rotation = DrivetrainRotationPosition(0_deg);
  } else if (is_slow_drive) {
    drivetrain_target.rotation = DrivetrainRotationPosition(180_deg);
  }  // For shelf/scoring position position

  if (driver_.readings().left_bumper)
    drivetrain_target = drivetrain_.LockModulesTarget();

  drivetrain_.SetTarget(drivetrain_target);
}

bool DriveCommand::IsFinished() { return false; }