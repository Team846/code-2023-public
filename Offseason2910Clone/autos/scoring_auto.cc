#include "Offseason2910Clone/autos/scoring_auto.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Offseason2910Clone/commands/extend_command.h"
#include "Offseason2910Clone/commands/follow_trajectory_command.h"
#include "Offseason2910Clone/commands/roller_command.h"
#include "Offseason2910Clone/field.h"
#include "Offseason2910Clone/subsystems/drivetrain.h"
#include "Offseason2910Clone/subsystems/robot_container.h"
#include "Offseason2910Clone/subsystems/swerve_module.h"
#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"
#include "transitionLib846/math.h"


ScoringAuto::ScoringAuto(RobotContainer& container, bool should_flip)
    : should_flip_(should_flip) {
  SetName("ScoringAutoCommand" +
          std::string(should_flip_ ? " flip" : " unflip"));
  AddCommands(
      frc2::InstantCommand{[&, flip = should_flip_] {
        auto pose_ = field::points::kStartScoringTable(flip);
        container.drivetrain_.SetPoint(pose_.point);
        container.drivetrain_.SetBearing(pose_.bearing);
      }},

      ExtendCommand{container, container.pivot_.subsystem()->cone_high_,
                    container.telescope_.subsystem()->cone_high_position_,
                    container.wrist_.subsystem()->cone_high_position_},
      frc2::InstantCommand{[&] {
        transitionLib846::Named(container.drivetrain_, "scoring").Debug("tt2");
      }},
      frc2::ParallelDeadlineGroup{frc2::WaitCommand{0.5_s},
                                  RollerCommand{container, false, true}},

      frc2::ParallelDeadlineGroup{
          FollowTrajectoryCommand{
              container,
              {{field::points::kIntermediaryPoint(should_flip), 11_fps},
               {field::points::kIntakeCubeOne(should_flip), 11_fps},
               {field::points::kIntakeCubeOnePlus(should_flip), 4_fps}}},
          ExtendCommand{container,
                        container.pivot_.subsystem()->cube_intake_low_,
                        container.telescope_.subsystem()->cube_intake_position_,
                        container.wrist_.subsystem()->cube_intake_position_},
          RollerCommand{container, true, false}},
      frc2::ParallelDeadlineGroup{
          FollowTrajectoryCommand{
              container,
              {{field::points::kIntermediaryPoint(should_flip), 0_fps},
               {field::points::kScoreCube(should_flip), 0_fps}}},
          ExtendCommand{container, container.pivot_.subsystem()->cube_high_,
                        container.telescope_.subsystem()->cube_high_position_,
                        container.wrist_.subsystem()->cube_place_position_}},
      frc2::ParallelDeadlineGroup{frc2::WaitCommand{0.5_s},
                                  RollerCommand{container, false, false}},
      frc2::ParallelDeadlineGroup{
          FollowTrajectoryCommand{
              container,
              {{field::points::kIntermediaryPoint(should_flip), 0_fps},
               {field::points::kIntakeCubeTwo(should_flip), 0_fps},
               {field::points::kIntakeCubeTwoPlus(should_flip), 4_fps}}},
          ExtendCommand{container,
                        container.pivot_.subsystem()->cube_intake_low_,
                        container.telescope_.subsystem()->cube_intake_position_,
                        container.wrist_.subsystem()->cube_intake_position_},
          RollerCommand{container, true, false}},
      frc2::ParallelDeadlineGroup{
          FollowTrajectoryCommand{
              container,
              {{field::points::kIntermediaryPoint(should_flip), 0_fps},
               {field::points::kScoreCube(should_flip), 0_fps}}},
          ExtendCommand{container, container.pivot_.subsystem()->cube_mid_,
                        container.telescope_.subsystem()->cube_mid_position_,
                        container.wrist_.subsystem()->cube_place_position_}},
      frc2::ParallelDeadlineGroup{frc2::WaitCommand{0.5_s},
                                  RollerCommand{container, false, false}});
}