#ifndef Y2023_COMMANDS_ALLIANCE_TWO_PIECE_AUTO_COMMAND_H_
#define Y2023_COMMANDS_ALLIANCE_TWO_PIECE_AUTO_COMMAND_H_

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/math.h"
#include "frc846/trajectory_generator.h"
#include "y2023/subsystems/robot_container.h"

class AllianceTwoPieceAutoCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AllianceTwoPieceAutoCommand> {
 public:
  AllianceTwoPieceAutoCommand(RobotContainer& container,
                                     bool should_flip_);

  bool should_flip_;
};

#endif  // Y2023_COMMANDS_ALLIANCE_TWO_PIECE_AUTO_COMMAND_H_