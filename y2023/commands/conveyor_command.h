#ifndef Y2023_COMMANDS_CONVEYOR_COMMAND_H_
#define Y2023_COMMANDS_CONVEYOR_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "y2023/subsystems/conveyor.h"
#include "y2023/subsystems/robot_container.h"

class ConveyorCommand
    : public frc2::CommandHelper<frc2::CommandBase, ConveyorCommand> {
 public:
  ConveyorCommand(RobotContainer& container,
                  const frc846::Pref<double>& belt_speed,
                  const frc846::Pref<double>& roller_speed);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalConveyorSubsystem& conveyor_;

  const frc846::Pref<double>& belt_speed_;
  const frc846::Pref<double>& roller_speed_;
};

#endif  // Y2023_COMMANDS_CONVEYOR_COMMAND_H_