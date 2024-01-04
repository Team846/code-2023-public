#ifndef Y2023_COMMANDS_AUTO_MID_ONE_PIECE_PICKUP_DOCK_AUTO_COMMAND_H_
#define Y2023_COMMANDS_AUTO_MID_ONE_PIECE_PICKUP_DOCK_AUTO_COMMAND_H_

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/math.h"
#include "frc846/trajectory_generator.h"
#include "y2023/subsystems/robot_container.h"

class MidOnePiecePickupDockAutoCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 MidOnePiecePickupDockAutoCommand> {
 public:
  MidOnePiecePickupDockAutoCommand(RobotContainer& container,
                                   bool should_flip_);

  bool should_flip_;
};

#endif  // Y2023_COMMANDS_AUTO_MID_ONE_PIECE_PICKUP_DOCK_AUTO_COMMAND_H_