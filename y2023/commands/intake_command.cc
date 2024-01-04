#include "y2023/commands/intake_command.h"

#include "y2023/subsystems/intake.h"

IntakeCommand::IntakeCommand(RobotContainer& container,
                             IntakeState intake_state,
                             const frc846::Pref<double>& roller_speed)
    : intake_(container.intake_),
      pivot_(container.pivot_),
      intake_state_(intake_state),
      roller_speed_(roller_speed) {
  AddRequirements({&intake_});
  SetName("intake_command");
}

void IntakeCommand::Execute() {
  if (intake_.Initialized() && pivot_.Initialized()) {
    if (intake_state_ == kDeploy && (pivot_.subsystem()->readings().position -
                         pivot_.subsystem()->retract_position_.value()) >
            pivot_.subsystem()->retract_tolerance_.value()) {
      // Retract pivot before intake deployed
      intake_.SetTarget({kStow, 0});
    } else {
      intake_.SetTarget({intake_state_, roller_speed_.value()});
    }
  } else if (intake_.Initialized()) {
    intake_.SetTarget({intake_state_, roller_speed_.value()});
  }
}

void IntakeCommand::End(bool interrupted) {
  (void)interrupted;
  if (intake_.Initialized()) {
    intake_.SetTargetZero();
  }
}