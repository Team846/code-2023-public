#ifndef Y2023_COMMANDS_EXTEND_COMMAND_H_
#define Y2023_COMMANDS_EXTEND_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "Offseason2910Clone/subsystems/pivot.h"
#include "Offseason2910Clone/subsystems/robot_container.h"
#include "transitionLib846/logger.h"
#include "transitionLib846/subsystem.h"


class ExtendCommand
    : public frc2::CommandHelper<frc2::CommandBase, ExtendCommand> {
 public:
  ExtendCommand(RobotContainer& container,
                const transitionLib846::Pref<units::degree_t>& pivot_target,
                const transitionLib846::Pref<units::inch_t>& telescope_target,
                const transitionLib846::Pref<units::degree_t>& wrist_target);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalPivotSubsystem& pivot_;
  OptionalTelescopeSubsystem& telescope_;
  OptionalWristSubsystem& wrist_;

  const transitionLib846::Pref<units::degree_t>& pivot_target_;
  const transitionLib846::Pref<units::inch_t>& telescope_target_;
  const transitionLib846::Pref<units::degree_t>& wrist_target_;
};

#endif  // Y2023_COMMANDS_EXTEND_COMMAND_H_