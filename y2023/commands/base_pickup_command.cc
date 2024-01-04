#include "y2023/commands/base_pickup_command.h"

#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>

#include "frc846/pref.h"
#include "units/math.h"
#include "y2023/commands/conveyor_command.h"
#include "y2023/commands/gripper_command.h"
#include "y2023/commands/position_pivot_command.h"
#include "y2023/commands/position_telescope_command.h"
#include "y2023/commands/retract_command.h"
#include "y2023/subsystems/robot_container.h"

BasePickupCommand::BasePickupCommand(RobotContainer& container) {
  SetName("Base_Pickup_Command");
  AddCommands(
      frc2::ParallelDeadlineGroup{
          frc2::ParallelDeadlineGroup{
              frc2::WaitUntilCommand([&container] {
                return units::math::abs(
                           container.pivot_.subsystem()->readings().position -
                           container.pivot_.subsystem()
                               ->retract_position_.value()) < 1_deg;
              }),
              RetractCommand{
                  container, container.pivot_.subsystem()->retract_position_,
                  container.telescope_.subsystem()->cone_conveyor_position_},
          },
          ConveyorCommand{
              container, container.conveyor_.subsystem()->belt_speed_,
              container.conveyor_.subsystem()->default_roller_speed_},
          frc2::InstantCommand{[&] { container.gripper_.SetTargetZero(); }}},
      frc2::ParallelDeadlineGroup{
          frc2::WaitCommand(1_s),
          ConveyorCommand{
              container, container.conveyor_.subsystem()->belt_speed_,
              container.conveyor_.subsystem()->default_roller_speed_},
          frc2::InstantCommand{[&] { container.gripper_.SetTargetZero(); }}},

      frc2::ParallelDeadlineGroup{
          frc2::WaitUntilCommand([&container] {
            return units::math::abs(
                       container.telescope_.subsystem()->readings().position -
                       container.telescope_.subsystem()
                           ->retract_position_.value()) < 0.1_tr;
          }),
          PositionTelescopeCommand{
              container, container.telescope_.subsystem()->retract_position_},
          frc2::InstantCommand{[&] { container.conveyor_.SetTargetZero(); }},
          frc2::InstantCommand{[&] { container.gripper_.SetTargetZero(); }}},
      frc2::ParallelDeadlineGroup{
          frc2::WaitUntilCommand([&container] {
            return units::math::abs(
                       container.pivot_.subsystem()->readings().position -
                       container.pivot_.subsystem()
                           ->cone_conveyor_position_.value()) < 1_deg;
          }),
          PositionPivotCommand{
              container, container.pivot_.subsystem()->cone_conveyor_position_},
          frc2::InstantCommand{[&] { container.conveyor_.SetTargetZero(); }},
          frc2::InstantCommand{[&] { container.gripper_.SetTargetZero(); }}},
      frc2::ParallelDeadlineGroup{PositionTelescopeCommand{
          container,
          container.telescope_.subsystem()->cone_conveyor_position_}},
      frc2::ParallelDeadlineGroup{
          frc2::WaitUntilCommand([&container] {
            return container.gripper_.subsystem()->readings().has_piece;
          }),
          frc2::InstantCommand{[&] { container.conveyor_.SetTargetZero(); }},
          GripperCommand{container,
                         container.gripper_.subsystem()->grab_speed_}});
};