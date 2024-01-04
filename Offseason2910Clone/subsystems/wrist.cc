#include "Offseason2910Clone/subsystems/wrist.h"

#include <variant>

#include "transitionLib846/motor/helper.h"

// TODO add homing and limiting
WristSubsystem::WristSubsystem()
    : transitionLib846::Subsystem<WristReadings, WristTarget>{"wrist"} {
  esc_helper_.OnInit([this] {
    esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    esc_.SetInverted(false);
    ResetPosition();
  });
}

WristTarget WristSubsystem::ZeroTarget() const {
  WristTarget target;
  target.output = 0.0;
  return target;
}

bool WristSubsystem::VerifyHardware() {
  bool ok = true;
  transitionLib846_VERIFY(esc_helper_.VerifyConnected(), ok,
                          "left_esc not connected");
  return ok;
}

WristTarget WristSubsystem::HoldPosition() {
  WristTarget target;
  target.output = readings().position;
  return target;
}

WristTarget WristSubsystem::ToPosition(
    transitionLib846::Pref<units::degree_t> pos) {
  WristTarget target;
  target.output = pos.value() - 19_deg;
  return target;
}

WristTarget WristSubsystem::IncrementPosition(units::degree_t increment) {
  WristTarget target;
  target.output = readings().position + increment;
  return target;
}

void WristSubsystem::SetHasHomed(bool homed) { has_homed_ = homed; }

void WristSubsystem::ResetPosition() {
  esc_helper_.encoder().SetPosition(0.0);
  has_homed_ = true;
}

WristReadings WristSubsystem::GetNewReadings() {
  WristReadings readings;
  readings.position =
      converter_.NativeToRealPosition(esc_helper_.encoder().GetPosition());

  position_graph_.Graph(readings.position);
  has_homed_graph_.Graph(has_homed_);

  return readings;
}

void WristSubsystem::WriteToHardware(WristTarget target) {
  // units::degree_t current_position = readings().position;

  if (has_homed_) {
    if (auto* position = std::get_if<WristPosition>(&target.output)) {
      auto output = *position;

      prev_target = output;

      if ((output - readings().position) <= 0.7_deg)
        esc_helper_.pid_controller().SetOutputRange(-0.15, -0.005);
      else if ((output - readings().position) >= 0.7_deg)
        esc_helper_.pid_controller().SetOutputRange(0.005, 0.15);

      esc_helper_.Write({transitionLib846::motor::ControlMode::Position,
                         converter_.RealToNativePosition(output)});

      target_position_graph_.Graph(readings().position);
    }
    esc_helper_.Write({transitionLib846::motor::ControlMode::Position,
                       converter_.RealToNativePosition(prev_target)});
    // else if (auto* output = std::get_if<DutyCycle>(&target.output)){
    //   double speed = *output;
    //     esc_helper_.Write({transitionLib846::motor::ControlMode::Percent,
    //     speed});
    // }
  }
}