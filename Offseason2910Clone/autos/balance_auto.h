#ifndef Offseason2910Clone_COMMANDS_BALANCE_AUTO_H_
#define Offseason2910Clone_COMMANDS_BALANCE_AUTO_H_

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Offseason2910Clone/subsystems/robot_container.h"
#include "transitionLib846/math.h"
#include "transitionLib846/trajectory_generator.h"


class BalanceAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, BalanceAuto> {
 public:
  BalanceAuto(RobotContainer& container, bool should_flip_);

  bool should_flip_;
};

#endif  // Offseason2910Clone_COMMANDS_BALANCE_AUTO_H_