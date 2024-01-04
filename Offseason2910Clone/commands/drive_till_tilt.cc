#include "Offseason2910Clone/commands/drive_till_tilt.h"

#include <frc/Timer.h>

#include "Offseason2910Clone/subsystems/drivetrain.h"
#include "transitionLib846/named.h"
#include "transitionLib846/wpilib/time.h"

DriveTillTilt::DriveTillTilt(RobotContainer& container, units::degree_t tilt_,
                             bool greater, units::feet_per_second_t spd)
    : drivetrain_(container.drivetrain_),
      greater_(greater),
      ftilt(tilt_),
      fspd(spd) {
  AddRequirements({&drivetrain_});
  SetName("drive_till_tilt_command");
}

void DriveTillTilt::Execute() {
  DrivetrainTarget drivetrain_target;
  auto max_speed = drivetrain_.balance_max_speed_.value();

  auto tilt = drivetrain_.readings().tilt;

  drivetrain_target.control = kClosedLoop;

  drivetrain_target.v_y = fspd;

  drivetrain_.SetTarget(drivetrain_target);

  if (greater_) {
    is_done_ = (tilt >= ftilt);
  } else {
    is_done_ = (tilt <= ftilt);
  }
}

bool DriveTillTilt::IsFinished() { return is_done_; }