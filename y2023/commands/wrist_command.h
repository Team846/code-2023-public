#ifndef Y2023_COMMANDS_WRIST_COMMAND_H_
#define Y2023_COMMANDS_WRIST_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "y2023/subsystems/robot_container.h"
#include "y2023/subsystems/wrist.h"

class WristCommand
    : public frc2::CommandHelper<frc2::CommandBase, WristCommand> {
 public:
  WristCommand(RobotContainer& container,
               const frc846::Pref<double>& speed);

  void Initialize() override;

  void End(bool interrupted) override;

 private:
  OptionalWristSubsystem& wrist_;
  const frc846::Pref<double>& speed_;
};
#endif  // Y2023_COMMANDS_WRIST_COMMAND_H_
