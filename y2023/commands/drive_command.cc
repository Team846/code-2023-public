#include "y2023/commands/drive_command.h"

#include <utility>

#include "frc846/math.h"
#include "y2023/subsystems/drivetrain.h"
#include "y2023/subsystems/swerve_module.h"

DriveCommand::DriveCommand(RobotContainer& container)
    : driver_(container.driver_),
      drivetrain_(container.drivetrain_),
      limelight_(container.limelight_) {
  AddRequirements({&drivetrain_});
  SetName("drive_command");
}

void DriveCommand::Execute() {
  DrivetrainTarget drivetrain_target;

  // -----BUTTON MAPPINGS-----

  // Left bumper   | robot centric translation
  // Right trigger | vision aiming
  // Right bumper  | precision drive

  bool is_robot_centric = false;
  bool is_slow_drive = driver_.readings().right_bumper;
  bool is_auto_aim = driver_.readings().right_trigger;





  // -----TRANSLATION CONTROL-----

  double translate_x = frc846::HorizontalDeadband(
      driver_.readings().left_stick_x, driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);
  double translate_y = frc846::HorizontalDeadband(
      driver_.readings().left_stick_y, driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);

  drivetrain_target.v_x = translate_x * drivetrain_.max_speed_.value();
  drivetrain_target.v_y = translate_y * drivetrain_.max_speed_.value();

  // Slow down translation if slow mode is active
  if (is_slow_drive) {
    translate_x = frc846::HorizontalDeadband(
      driver_.readings().left_stick_x*abs(driver_.readings().left_stick_x), driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);
    translate_y = frc846::HorizontalDeadband(
      driver_.readings().left_stick_y*abs(driver_.readings().left_stick_y), driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);
    drivetrain_target.v_x = translate_x * drivetrain_.max_speed_.value() * drivetrain_.slow_mode_percent_.value();
    drivetrain_target.v_y = translate_y * drivetrain_.max_speed_.value() * drivetrain_.slow_mode_percent_.value();
  }

  // Robot vs field oriented translation
  drivetrain_target.translation_reference =
      is_robot_centric ? DrivetrainTranslationReference::kRobot
                       : DrivetrainTranslationReference::kField;

  drivetrain_target.control = kOpenLoop;

  // -----STEER CONTROL-----

  double steer_x = frc846::HorizontalDeadband(
      driver_.readings().right_stick_x, driver_.steer_deadband_.value(), 1,
      driver_.steer_exponent_.value(), 1);

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
    if (is_slow_drive) {
      target *= drivetrain_.slow_omega_percent_.value();
    }

    drivetrain_target.rotation = DrivetrainRotationVelocity(target);
  } else {
    // Hold position
    drivetrain_target.rotation = DrivetrainRotationVelocity(0_deg_per_s);
  }

  drivetrain_.SetTarget(drivetrain_target);
}

bool DriveCommand::IsFinished() { return false; }