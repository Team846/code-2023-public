#ifndef Y2023_COMMANDS_PICKUP_CONVEYOR_COMMAND_H_
#define Y2023_COMMANDS_PICKUP_CONVEYOR_COMMAND_H_

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/subsystem.h"
#include "y2023/subsystems/robot_container.h"
#include "y2023/subsystems/telescope.h"

class PickupConveyorCommand
    : public frc2::CommandHelper<frc2::CommandBase, PickupConveyorCommand> {
 public:
  PickupConveyorCommand(RobotContainer& container,
                        const frc846::Pref<bool>& should_run);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalGripperSubsystem& gripper_;

  const frc846::Pref<bool>& should_auto_pickup_;
};

#endif  // Y2023_COMMANDS_PICKUP_CONVEYOR_COMMAND_H_