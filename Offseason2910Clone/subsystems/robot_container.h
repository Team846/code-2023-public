#ifndef transitionLib846_SUBSYSTEMS_ROBOT_CONTAINER_H_
#define transitionLib846_SUBSYSTEMS_ROBOT_CONTAINER_H_

#include "Offseason2910Clone/subsystems/driver.h"
#include "Offseason2910Clone/subsystems/drivetrain.h"
#include "Offseason2910Clone/subsystems/leds.h"
#include "Offseason2910Clone/subsystems/limelight.h"
#include "Offseason2910Clone/subsystems/operator.h"
#include "Offseason2910Clone/subsystems/pivot.h"
#include "Offseason2910Clone/subsystems/roller.h"
#include "Offseason2910Clone/subsystems/telescope.h"
#include "Offseason2910Clone/subsystems/wrist.h"
#include "transitionLib846/pref.h"


class RobotContainer : public transitionLib846::Named {
 public:
  RobotContainer() : transitionLib846::Named{"robot_container"} {}

 private:
  transitionLib846::Pref<bool> init_leds_{*this, "init_leds", false};
  transitionLib846::Pref<bool> init_limelight_{*this, "init_limelight ", false};
  transitionLib846::Pref<bool> init_roller_{*this, "init_roller ", true};
  transitionLib846::Pref<bool> init_pivot_{*this, "init_pivot ", false};
  transitionLib846::Pref<bool> init_wrist_{*this, "init_wrist ", false};
  transitionLib846::Pref<bool> init_telescope_{*this, "init_telescope ", false};

 public:
  DriverSubsystem driver_;
  OperatorSubsystem operator_;
  DrivetrainSubsystem drivetrain_;
  OptionalLEDsSubsystem leds_{init_leds_.value(), "leds"};
  OptionalLimelightSubsystem limelight_{init_limelight_.value(), "limelight"};
  OptionalRollerSubsystem roller_{init_roller_.value(), "roller"};
  OptionalPivotSubsystem pivot_{init_pivot_.value(), "pivot"};
  OptionalWristSubsystem wrist_{init_wrist_.value(), "wrist"};
  OptionalTelescopeSubsystem telescope_{init_telescope_.value(), "telescope"};

  std::vector<transitionLib846::SubsystemBase*> all_subsystems_{
      &driver_, &operator_, &drivetrain_, &limelight_, &leds_,
      &roller_, &pivot_,    &wrist_,      &telescope_};
};

#endif  // transitionLib846_SUBSYSTEMS_ROBOT_CONTAINER_H_