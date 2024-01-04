#ifndef Y2023_COMMANDS_BASE_PICKUP_COMMAND_H_
#define Y2023_COMMANDS_BASE_PICKUP_COMMAND_H_

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/subsystem.h"
#include "y2023/subsystems/robot_container.h"
#include "y2023/subsystems/telescope.h"

class BasePickupCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 BasePickupCommand> {
 public:
  BasePickupCommand(RobotContainer& container);

};

#endif  // Y2023_COMMANDS_BASE_PICKUP_COMMAND_H_