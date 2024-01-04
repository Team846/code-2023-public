#include "y2023/autos/alliance_two_piece_dock_auto_command.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitUntilCommand.h>

#include "frc2/command/WaitCommand.h"
#include "frc846/math.h"
#include "y2023/commands/auto_balance_command.h"
#include "y2023/commands/auto_extend_command.h"
#include "y2023/commands/conveyor_command.h"
#include "y2023/commands/extend_command.h"
#include "y2023/commands/follow_trajectory_command.h"
#include "y2023/commands/gripper_command.h"
#include "y2023/commands/intake_command.h"
#include "y2023/commands/load_conveyor_command.h"
#include "y2023/commands/pickup_conveyor_command.h"
#include "y2023/commands/place_command.h"
#include "y2023/commands/position_telescope_command.h"
#include "y2023/commands/retract_command.h"
#include "y2023/field.h"
#include "y2023/subsystems/intake.h"
#include "y2023/subsystems/robot_container.h"

AllianceTwoPieceDockAutoCommand::AllianceTwoPieceDockAutoCommand(
    RobotContainer& container, bool should_flip)
    : should_flip_(should_flip) {
  SetName("AllianceTwoPieceDockAutoCommand" +
          std::string(should_flip_ ? " red" : " blue"));
  AddCommands(
      frc2::ParallelDeadlineGroup{
          // Timeout command after 14.5 seconds
          frc2::WaitCommand{14.5_s},
          frc2::SequentialCommandGroup{
              frc2::InstantCommand{[&, flip = should_flip_] {
                auto pose_ = field::points::kStartAllianceWall(flip);
                container.drivetrain_.SetPoint(pose_.point);
                container.drivetrain_.SetBearing(pose_.bearing);
              }},

              // Retract arm in case it's fallen
              frc2::ParallelRaceGroup{
                  frc2::WaitCommand{0.3_s},
                  PositionTelescopeCommand{container,
                                           container.telescope_.subsystem()
                                               ->cube_conveyor_position_}},

              // Extend arm
              frc2::ParallelRaceGroup{
                  frc2::WaitCommand{2_s},
                  AutoExtendCommand{
                      container,
                      container.pivot_.subsystem()->cone_high_position_,
                      container.telescope_.subsystem()->cone_high_position_,
                      container.pivot_.subsystem()->cone_min_mid_}},

              // Align to cone node
              FollowTrajectoryCommand{
                  container,
                  {
                      {field::points::kScoreCone4(should_flip_), 0_fps},
                  }},

              // Place Game Piece
              frc2::ParallelDeadlineGroup{frc2::WaitCommand{0.75_s},
                                          PlaceCommand{container}},

              // Drive and Intake
              frc2::ParallelDeadlineGroup{
                  FollowTrajectoryCommand{
                      container,
                      {{field::points::kIntermediaryAlliance(should_flip_),
                        9_fps},
                       {field::points::kIntake4(should_flip_), 0_fps}},
                  },
                  // Run gripper out
                  frc2::ParallelDeadlineGroup{
                      frc2::WaitCommand{1_s},
                      GripperCommand{container, container.gripper_.subsystem()
                                                    ->place_speed_}},
                  RetractCommand{
                      container,
                      container.pivot_.subsystem()->cube_conveyor_position_,
                      container.telescope_.subsystem()
                          ->cube_conveyor_position_},
                  IntakeCommand{
                      container, kDeploy,
                      container.intake_.subsystem()->cube_roller_speed_}},

              // Deploy for game piece to stay in the robot
              frc2::ParallelDeadlineGroup{
                  frc2::WaitCommand{1.1_s},
                  IntakeCommand{
                      container, kDeploy,
                      container.intake_.subsystem()->cube_roller_speed_},
                  LoadConveyorCommand{
                      container, container.conveyor_.subsystem()->should_run_},
                  PickupConveyorCommand{
                      container,
                      container.gripper_.subsystem()->should_auto_pickup_}},

              // Hold stow till game piece in the robot
              frc2::ParallelDeadlineGroup{
                  frc2::ParallelRaceGroup{frc2::WaitUntilCommand([&container] {
                                            return container.gripper_
                                                .subsystem()
                                                ->readings()
                                                .has_piece;
                                          }),
                                          frc2::WaitCommand{2_s}},
                  IntakeCommand{
                      container, kStow,
                      container.intake_.subsystem()->cube_roller_speed_},
                  LoadConveyorCommand{
                      container, container.conveyor_.subsystem()->should_run_},
                  PickupConveyorCommand{
                      container,
                      container.gripper_.subsystem()->should_auto_pickup_}},

              // Stow intake, extend arm, and drive to target
              frc2::ParallelDeadlineGroup{
                  FollowTrajectoryCommand{
                      container,
                      {{field::points::kIntermediaryAlliance(should_flip_),
                        6_fps},
                       {field::points::kScoreCube4(should_flip_), 0_fps}},

                  },
                  frc2::SequentialCommandGroup{
                      frc2::ParallelRaceGroup{
                          frc2::WaitUntilCommand([&container] {
                            return container.gripper_.subsystem()
                                ->readings()
                                .has_piece;
                          }),
                          frc2::WaitCommand{1.7_s}},
                      ExtendCommand{
                          container,
                          container.pivot_.subsystem()->cube_high_position_,
                          container.telescope_.subsystem()->cube_high_position_,
                          container.pivot_.subsystem()->cube_min_high_},
                  },

                  IntakeCommand{
                      container, kStow,
                      container.intake_.subsystem()->zero_roller_speed_}},

              // Place cube
              frc2::ParallelDeadlineGroup{
                  frc2::WaitCommand{0.2_s},
                  GripperCommand{container,
                                 container.gripper_.subsystem()->place_speed_}},
              frc2::ParallelDeadlineGroup{
                  frc2::SequentialCommandGroup{
                      FollowTrajectoryCommand{
                          container,
                          {{field::points::kBalanceIntermediaryAlliance(
                                should_flip_),
                            9_fps},
                           {field::points::kBalanceBackwardAlliance(
                                should_flip_),
                            6_fps}}},
                      AutoBalanceCommand{container}},
                  frc2::SequentialCommandGroup{
                      frc2::ParallelDeadlineGroup{
                          frc2::WaitCommand{0.5_s},
                          GripperCommand{
                              container,
                              container.gripper_.subsystem()->place_speed_},
                      },
                      RetractCommand{
                          container,
                          container.pivot_.subsystem()->cube_conveyor_position_,
                          container.telescope_.subsystem()
                              ->cube_conveyor_position_},
                  }}}},

      // Lock Wheels
      frc2::InstantCommand{[&container] {
        container.drivetrain_.SetTarget(
            {0_fps, 0_fps, kField,
             container.drivetrain_.readings().pose.bearing + 1_deg,
             kClosedLoop});
      }});
  ;
}