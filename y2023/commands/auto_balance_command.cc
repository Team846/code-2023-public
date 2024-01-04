#include "y2023_off/commands/auto_balance_command.h"

#include "frcRev846/wpilib/time.h"
#include "y2023_off/subsystems/drivetrain.h"
#include "y2023_off/subsystems/swerve_module.h"
#include <frc/Timer.h>
#include <frc/DataLogManager.h>

AutoBalanceCommand::AutoBalanceCommand(RobotContainer& container)
    : drivetrain_(container.drivetrain_) {
  AddRequirements({&drivetrain_});
  SetName("auto_balance_command");
  start_time_ = frcRev846::wpilib::CurrentFPGATime();
}

AutoBalanceCommand::AutoBalanceCommand(RobotContainer& container, units::second_t start_time)
    : drivetrain_(container.drivetrain_) {
  AddRequirements({&drivetrain_});
  SetName("auto_balance_command");
  start_time_ = start_time;
}

void AutoBalanceCommand::Execute() {
  DrivetrainTarget drivetrain_target;
  auto max_speed = drivetrain_.balance_max_speed_.value();

  auto tilt = drivetrain_.readings().tilt;
  units::degree_t p_error = tilt;
  if (drivetrain_.readings().pose.bearing > 90_deg) p_error *= -1;
  units::degrees_per_second_t d_error = drivetrain_.readings().tilt_omega;
  auto target_speed = units::feet_per_second_t(
      drivetrain_.balance_gains_p_.value() * p_error.to<double>()  +
      drivetrain_.balance_gains_d_.value() * d_error.to<double>());
  if (abs(p_error.to<double>())>13){
    target_speed=units::feet_per_second_t(drivetrain_.balance_max_speed_.value()*p_error.to<double>()/abs(p_error.to<double>()));
  }
  drivetrain_target.control = kClosedLoop;
  drivetrain_target.translation_reference = kField;
  drivetrain_target.rotation = DrivetrainRotationVelocity(0_deg_per_s);
  drivetrain_target.v_y = target_speed;
  if (units::math::abs(drivetrain_target.v_y) > max_speed) {
    if (drivetrain_target.v_y > 0_fps) {
      drivetrain_target.v_y = max_speed;
    } else {
      drivetrain_target.v_y = -max_speed;
    }
  }
      
  if (units::math::abs(tilt) < drivetrain_.angle_threshold_.value()) {
    if (loops_since > 4) { 
      is_balanced_ = true;
      drivetrain_.SetTarget(drivetrain_.LockModulesTarget());

      logger->sendLog("final_balance_angle", drivetrain_.readings().tilt.to<double>());
      logger->sendLog("finished balancing", units::math::abs(frcRev846::wpilib::CurrentFPGATime()-start_time_));
      
    } else {
      loops_since += 1;
    }
  } else {
    drivetrain_.SetTarget(drivetrain_target);
    loops_since = 0;
  }
}

bool AutoBalanceCommand::IsFinished() { return is_balanced_; }