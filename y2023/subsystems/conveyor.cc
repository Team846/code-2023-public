#include "y2023/subsystems/conveyor.h"

ConveyorSubsystem::ConveyorSubsystem()
    : frc846::Subsystem<ConveyorReadings, ConveyorTarget>{"conveyor"} {
  belt_esc_.SetInverted(true);

  left_roller_esc_.SetInverted(true);
}

ConveyorTarget ConveyorSubsystem::ZeroTarget() const {
  ConveyorTarget target;
  target.left_roller_speed = 0.0;
  target.right_roller_speed = 0.0;
  target.belt_speed = 0.0;
  return target;
}

bool ConveyorSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(left_roller_esc_.IsAlive(), ok, "left_roller_esc not alive");
  FRC846_VERIFY(right_roller_esc_.IsAlive(), ok, "right_roller_esc not alive");
  FRC846_VERIFY(belt_esc_.IsAlive(), ok, "belt_esc not connected");
  FRC846_VERIFY(left_roller_esc_.GetInverted() == true, ok,
                "left_roller_esc incorrect invert state");
  FRC846_VERIFY(right_roller_esc_.GetInverted() == false, ok,
                "right_roller_esc incorrect invert state");
  FRC846_VERIFY(belt_esc_.GetInverted() == true, ok,
                "belt esc incorrect invert state");
  return ok;
}

ConveyorReadings ConveyorSubsystem::GetNewReadings() {
  ConveyorReadings readings;

  return readings;
}

void ConveyorSubsystem::WriteToHardware(ConveyorTarget target) {
  if (target.left_roller_speed != 0) {
    loops++;
    if ((loops + 50) % 100 < 20) {
      left_roller_esc_.Set(target.left_roller_speed +
                           differential_speed_.value() / 2);
    } else {
      left_roller_esc_.Set(target.left_roller_speed);
    }

    if (loops % 100 < 20) {
      right_roller_esc_.Set(target.right_roller_speed -
                            differential_speed_.value() / 2);
    } else {
      right_roller_esc_.Set(target.right_roller_speed);
    }
  }
  right_roller_esc_.Set(target.right_roller_speed);
  left_roller_esc_.Set(target.left_roller_speed);
  belt_esc_.Set(target.belt_speed);
}