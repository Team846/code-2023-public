#ifndef FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_
#define FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_

#include "frc846/pref.h"
#include "y2023/subsystems/conveyor.h"
#include "y2023/subsystems/driver.h"
#include "y2023/subsystems/drivetrain.h"
#include "y2023/subsystems/gripper.h"
#include "y2023/subsystems/intake.h"
#include "y2023/subsystems/leds.h"
#include "y2023/subsystems/limelight.h"
#include "y2023/subsystems/operator.h"
#include "y2023/subsystems/pivot.h"
#include "y2023/subsystems/telescope.h"
#include "y2023/subsystems/wrist.h"

class RobotContainer : public frc846::Named {
 public:
  RobotContainer() : frc846::Named{"robot_container"} {}

 private:
  frc846::Pref<bool> init_pivot_{*this, "init_pivot", true};
  frc846::Pref<bool> init_telescope_{*this, "init_telescope", true};
  frc846::Pref<bool> init_conveyor_{*this, "init_conveyor", true};
  frc846::Pref<bool> init_gripper_{*this, "init_gripper", true};
  frc846::Pref<bool> init_wrist_{*this, "init_wrist", false};
  frc846::Pref<bool> init_intake_{*this, "init_intake", true};
  frc846::Pref<bool> init_leds_{*this, "init_leds", true};
  frc846::Pref<bool> init_limelight_{*this, "init_limelight ", true};

 public:
  DriverSubsystem driver_;
  OperatorSubsystem operator_;
  DrivetrainSubsystem drivetrain_;
  OptionalPivotSubsystem pivot_{init_pivot_.value(), "pivot"};
  OptionalTelescopeSubsystem telescope_{init_telescope_.value(), "telescope"};
  OptionalIntakeSubsystem intake_{init_intake_.value(), "intake"};
  OptionalConveyorSubsystem conveyor_{init_conveyor_.value(), "conveyor"};
  OptionalGripperSubsystem gripper_{init_gripper_.value(), "gripper"};
  OptionalWristSubsystem wrist_{init_wrist_.value(), "wrist"};
  OptionalLEDsSubsystem leds_{init_leds_.value(), "leds"};
  OptionalLimelightSubsystem limelight_{init_limelight_.value(), "limelight"};

  std::vector<frc846::SubsystemBase*> all_subsystems_{
      &driver_,  &operator_, &drivetrain_, &telescope_, &pivot_, &intake_,
      &gripper_, &conveyor_, &wrist_,      &limelight_, &leds_};
};

#endif  // FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_