#include "y2023/subsystems/wrist.h"

#include "frc846/motor/helper.h"
#include "frc846/subsystem.h"

WristSubsystem::WristSubsystem()
    : frc846::Subsystem<WristReadings, WristTarget>{"Wrist"} {
  esc_helper_.OnInit([&] {
    esc_helper_.DisableStatusFrames(
        {ctre::StatusFrameEnhanced::Status_1_General});
  });

  esc_helper_.Setup();
}

WristTarget WristSubsystem::ZeroTarget() const {
  WristTarget target;
  target.speed = 0.0;
  return target;
}

bool WristSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_helper_.VerifyConnected(), ok, "esc not connected");
  return ok;
}

WristReadings WristSubsystem::GetNewReadings() {
  WristReadings readings;

  return readings;
}

void WristSubsystem::WriteToHardware(WristTarget target) {
  target_speed_graph_.Graph(target.speed);
  if (target.speed > 0.0) {
    prev_state_ = kForward;
  } else if (target.speed < 0.0) {
    prev_state_ = kBackward;
  }

  esc_helper_.Write({frc846::motor::ControlMode::Percent, target.speed});
}