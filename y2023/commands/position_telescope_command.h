#ifndef Y2023_COMMANDS_POSITION_TELESCOPE_COMMAND_H_
#define Y2023_COMMANDS_POSITION_TELESCOPE_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/subsystem.h"
#include "y2023/subsystems/robot_container.h"
#include "y2023/subsystems/telescope.h"

class PositionTelescopeCommand
    : public frc2::CommandHelper<frc2::CommandBase, PositionTelescopeCommand> {
 public:
  PositionTelescopeCommand(RobotContainer& container,
                         const frc846::Pref<units::turn_t>& target);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalTelescopeSubsystem& telescope_;

  const frc846::Pref<units::turn_t>& target_;
};

#endif  // Y2023_COMMANDS_POSITION_TELESCOPE_COMMAND_H_