#ifndef Y2023_COMMANDS_RETRACT_COMMAND_H_
#define Y2023_COMMANDS_RETRACT_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/subsystem.h"
#include "y2023/subsystems/robot_container.h"
#include "y2023/subsystems/telescope.h"

class RetractCommand
    : public frc2::CommandHelper<frc2::CommandBase, RetractCommand> {
 public:
  RetractCommand(RobotContainer& container,
                         const frc846::Pref<units::degree_t>& pivot_target, const frc846::Pref<units::turn_t>& telescope_target);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalTelescopeSubsystem& telescope_;
  OptionalPivotSubsystem& pivot_;

  const frc846::Pref<units::degree_t>& pivot_target_;
  const frc846::Pref<units::turn_t>& telescope_target_;
};

#endif  // Y2023_COMMANDS_RETRACT_COMMAND_H_