#ifndef Y2023_COMMANDS_LOAD_CONVEYOR_COMMAND_H_
#define Y2023_COMMANDS_LOAD_CONVEYOR_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "y2023/subsystems/conveyor.h"
#include "y2023/subsystems/robot_container.h"

class LoadConveyorCommand
    : public frc2::CommandHelper<frc2::CommandBase, LoadConveyorCommand> {
 public:
  LoadConveyorCommand(RobotContainer& container,
                      const frc846::Pref<bool>& should_run);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalConveyorSubsystem& conveyor_;
  OptionalGripperSubsystem& gripper_;

  const frc846::Pref<bool>& should_run_;
};

#endif  // Y2023_COMMANDS_LOAD_CONVEYOR_COMMAND_H_