#ifndef Y2023_COMMANDS_PLACE_COMMAND_COMMAND_H_
#define Y2023_COMMANDS_PLACE_COMMAND_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/subsystem.h"
#include "y2023/subsystems/pivot.h"
#include "y2023/subsystems/robot_container.h"

class PlaceCommand
    : public frc2::CommandHelper<frc2::CommandBase, PlaceCommand> {
 public:
  PlaceCommand(RobotContainer& container);

  void Initialize() override;

  void End(bool interrupted) override;

 private:
  OptionalPivotSubsystem& pivot_;
};

#endif  // Y2023_COMMANDS_PLACE_COMMAND_H_