#ifndef Offseason2910Clone_SUBSYSTEMS_OPERATOR_H_
#define Offseason2910Clone_SUBSYSTEMS_OPERATOR_H_

#include "Offseason2910Clone/ports.h"
#include "transitionLib846/grapher.h"
#include "transitionLib846/math.h"
#include "transitionLib846/pref.h"
#include "transitionLib846/subsystem.h"
#include "transitionLib846/xbox.h"


using OperatorReadings = transitionLib846::XboxReadings;

struct OperatorTarget {
  bool rumble;
};

class OperatorSubsystem
    : public transitionLib846::Subsystem<OperatorReadings, OperatorTarget> {
 public:
  OperatorSubsystem();

  OperatorTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  transitionLib846::Pref<double> trigger_threshold_{*this, "trigger_threshold",
                                                    0.3};
  transitionLib846::Pref<double> rumble_strength_{*this, "rumble_strength",
                                                  1.0};

  frc::XboxController xbox_{ports::operator_::kXbox_DSPort};

  transitionLib846::Named target_named_{*this, "target"};
  transitionLib846::Grapher<bool> target_rumble_graph_{target_named_, "rumble"};

  OperatorReadings GetNewReadings() override;

  void WriteToHardware(OperatorTarget target) override;
};

#endif  // Offseason2910Clone_SUBSYSTEMS_Dawg_H_