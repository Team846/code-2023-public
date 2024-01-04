#ifndef Y2023_COMMANDS_POSITION_PIVOT_COMMAND_H_
#define Y2023_COMMANDS_POSITION_PIVOT_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/subsystem.h"
#include "y2023/subsystems/robot_container.h"
#include "y2023/subsystems/pivot.h"

class PositionPivotCommand
    : public frc2::CommandHelper<frc2::CommandBase, PositionPivotCommand> {
 public:
  PositionPivotCommand(RobotContainer& container,
                         const frc846::Pref<units::degree_t>& target);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalPivotSubsystem& pivot_;

  const frc846::Pref<units::degree_t>& target_;
};

#endif  // Y2023_COMMANDS_POSITION_PIVOT_COMMAND_H_