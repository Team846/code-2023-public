#ifndef Offseason2910Clone_COMMANDS_DRIVE_TILL_TILT_COMMAND_H_
#define Offseason2910Clone_COMMANDS_DRIVE_TILL_TILT_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "Offseason2910Clone/subsystems/drivetrain.h"
#include "Offseason2910Clone/subsystems/robot_container.h"
#include "transitionLib846/logger.h"

class DriveTillTilt
    : public frc2::CommandHelper<frc2::CommandBase, DriveTillTilt> {
 public:
  DriveTillTilt(RobotContainer& container, units::degree_t tilt_, bool greater,
                units::feet_per_second_t spd);

  void Execute() override;

  bool IsFinished() override;

 private:
  transitionLib846::Logger* logger =
      new transitionLib846::Logger("drive_till_tilt", false, true);

  DrivetrainSubsystem& drivetrain_;

  bool is_done_ = false;

  bool greater_;

  units::degree_t ftilt = 0_deg;

  units::feet_per_second_t fspd;
};

#endif  // Offseason2910Clone_COMMANDS_DRIVE_TILL_TILT_COMMAND_H_