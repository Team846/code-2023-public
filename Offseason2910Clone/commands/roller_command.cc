#include "Offseason2910Clone/commands/roller_command.h"

#include "transitionLib846/named.h"

RollerCommand::RollerCommand(RobotContainer& container, const bool grab,
                             const bool cone)
    : roller_(container.roller_), grab_(grab), cone_(cone) {
  AddRequirements({&roller_});
  SetName("roller_command");
}

void RollerCommand::Initialize() {
  if (roller_.Initialized()) {
    roller_.subsystem()->ToggleCubeMode(!cone_);

    if (grab_)
      roller_.subsystem()->GrabTarget();
    else
      roller_.subsystem()->ReleaseTarget();
  }
  return;
}

void RollerCommand::End(bool interrupted) { (void)interrupted; }