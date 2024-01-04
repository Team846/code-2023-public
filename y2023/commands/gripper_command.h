#ifndef Y2023_COMMANDS_GRIPPER_COMMAND_H_
#define Y2023_COMMANDS_GRIPPER_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "y2023/subsystems/gripper.h"
#include "y2023/subsystems/robot_container.h"

class GripperCommand
    : public frc2::CommandHelper<frc2::CommandBase, GripperCommand> {
 public:
  GripperCommand(RobotContainer& container,
                 const frc846::Pref<double>& gripper_speed);

  void Initialize() override;

  void End(bool interrupted) override;

 private:
  OptionalGripperSubsystem& gripper_;
  const frc846::Pref<double>& gripper_speed_;
};
#endif  // Y2023_COMMANDS_GRIPPER_COMMAND_H_
