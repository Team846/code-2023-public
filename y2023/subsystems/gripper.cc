#include "y2023/subsystems/gripper.h"

#include "frc846/motor/helper.h"
#include "frc846/subsystem.h"

GripperSubsystem::GripperSubsystem()
    : frc846::Subsystem<GripperReadings, GripperTarget>{"gripper"} {
  esc_helper_.OnInit([&] { esc_.SetInverted(true); });

  esc_helper_.Setup();
  roller_current_sensor.Reset();
}

GripperTarget GripperSubsystem::ZeroTarget() const {
  GripperTarget target;
  target.speed = 0.0;
  return target;
}

bool GripperSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_helper_.VerifyConnected(), ok, "esc not connected");
  FRC846_VERIFY(esc_.GetInverted() == true, ok, "esc incorrect invert state");
  return ok;
}

GripperReadings GripperSubsystem::GetNewReadings() {
  GripperReadings readings;

  units::ampere_t current = units::ampere_t(esc_.GetSupplyCurrent());
  roller_current_sensor.Calculate(current);
  current_graph_.Graph(current);
  avg_current_graph_.Graph(roller_current_sensor.GetAvgCurrent());

  if (roller_current_sensor.GetSpiked()) {
    has_piece_ = true;
  }

  if (has_piece_) {
    loops_since_++;
  } else {
    loops_since_ = 0;
  }

  readings.has_piece = has_piece_ && loops_since_ > 10;

  kLoops++;
  return readings;
}


void GripperSubsystem::WriteToHardware(GripperTarget target) {
  auto speed = 0.0;
  if (target.speed == -2.0) {
    speed = grab_speed_.value();
    has_piece_ = false;
  } else if (!readings().has_piece && target.speed < 0.0) {
    // Stop running
    speed = target.speed;
  } else if (target.speed > 0.0) {
    // Let go of a piece
    has_piece_ = false;
    loops_since_ = 0;
    speed = target.speed;
  } else {
    speed = idle_speed_.value();
  }

  has_piece_graph_.Graph(has_piece_);
  target_speed_graph_.Graph(speed);

  esc_helper_.Write({frc846::motor::ControlMode::Percent, speed});
}