#ifndef transitionLib846_MOTOR_GAINS_H_
#define transitionLib846_MOTOR_GAINS_H_

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <rev/CANSparkMax.h>

#include "transitionLib846/ctre_namespace.h"
#include "transitionLib846/pref.h"
#include "units/time.h"
#include <units/current.h>

transitionLib846_CTRE_NAMESPACE()

namespace transitionLib846::motor {

struct Gains {
  double p;
  double i;
  double d;
  double f;
  double max_integral_accumulator;
};

class GainsHelper : public Named {
 public:
  GainsHelper(Named& parent, Gains gains, units::ampere_t currentLimit = 20.0_A);

  transitionLib846::Pref<double> p_;
  transitionLib846::Pref<double> i_;
  transitionLib846::Pref<double> d_;
  transitionLib846::Pref<double> f_;
  transitionLib846::Pref<double> max_integral_accumulator_;
  transitionLib846::Pref<double> current_limit_;

  void Write(ctre::BaseTalon& esc, Gains& cache,
             units::time::millisecond_t timeout, bool ignore_cache = false);

  void Write(rev::SparkMaxPIDController& pid_controller, Gains& cache,
             bool ignore_cache = false);

 private:
  static const int idx = 0;

  void UpdateCache(Gains& cache);
};

}  // namespace transitionLib846::motor

#endif  // transitionLib846_MOTOR_GAINS_H_