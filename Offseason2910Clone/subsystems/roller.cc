#include "Offseason2910Clone/subsystems/roller.h"

#include <units/math.h>

#include "transitionLib846/motor/helper.h"
#include "transitionLib846/subsystem.h"

RollerSubsystem::RollerSubsystem()
    : transitionLib846::Subsystem<RollerReadings, RollerTarget>{"roller"} {
  esc_helper_.OnInit([&] {
    esc_.SetInverted(true);
    esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  });
  esc_helper_.Setup();
}

RollerTarget RollerSubsystem::ZeroTarget() const {
  RollerTarget target;
  target.state = kEmpty;
  return target;
}

RollerTarget RollerSubsystem::GrabTarget() {
  RollerTarget target;
  target.state = kGrab;
  wanted_state = kGrab;
  return target;
}

RollerTarget RollerSubsystem::ReleaseTarget() {
  RollerTarget target;
  target.state = kRelease;
  wanted_state = kRelease;
  return target;
}

bool RollerSubsystem::VerifyHardware() {
  bool ok = true;
  transitionLib846_VERIFY(esc_helper_.VerifyConnected(), ok,
                          "esc not connected");
  transitionLib846_VERIFY(esc_.GetInverted() == true, ok,
                          "esc incorrect invert state");
  return ok;
}

void RollerSubsystem::ToggleCubeMode() { cube_mode = !cube_mode; }

void RollerSubsystem::ToggleCubeMode(bool mode) { cube_mode = mode; }

bool RollerSubsystem::GetCubeMode() { return cube_mode; }

RollerReadings RollerSubsystem::GetNewReadings() {
  RollerReadings readings;

  units::ampere_t current = units::ampere_t(esc_.GetOutputCurrent());
  current_graph_.Graph(current);
  avg_current_graph_.Graph(roller_current_sensor.GetAvgCurrent());

  double speed_detection_thresh_ = (!cube_mode) ? speed_threshold_cone_.value()
                                                : speed_threshold_cube_.value();

  if ((units::math::abs(
           converter_.NativeToRealVelocity(esc_helper_.encoder().GetVelocity()))
           .to<double>()) < speed_detection_thresh_ &&
      wanted_state == kGrab) {
    if (loops_since_ > 30) {
      readings.has_piece = true;
      has_piece_ = true;
      loops_since_ = 0;
    } else {
      loops_since_++;
    }
  } else {
    loops_since_ = 0;
  }

  return readings;
}

void RollerSubsystem::WriteToHardware(RollerTarget target) {
  double roller_speed_ = last_roller_speed_;

  target.state = wanted_state;

  if (target.state == kRelease) {
    roller_speed_ =
        (!cube_mode) ? place_cone_speed_.value() : place_cube_speed_.value();
    has_piece_ = false;
  } else if (target.state == kHold || has_piece_) {
    roller_speed_ =
        (!cube_mode) ? idle_speed_cone_.value() : idle_speed_cube_.value();
  } else if (target.state == kGrab) {
    roller_speed_ =
        (!cube_mode) ? grab_cone_speed_.value() : grab_cube_speed_.value();
  } else if (target.state == kEmpty) {
    roller_speed_ = 0.0;
  }

  last_roller_speed_ = roller_speed_;

  has_piece_graph_.Graph(has_piece_);

  esc_helper_.Write(
      {transitionLib846::motor::ControlMode::Percent, roller_speed_});
}