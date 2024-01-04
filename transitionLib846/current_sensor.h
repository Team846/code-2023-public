#ifndef transitionLib846_CURRENT_SENSOR_H_
#define transitionLib846_CURRENT_SENSOR_H_

#include <fmt/format.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <deque>

#include "transitionLib846/pref.h"
#include "units/current.h"

namespace transitionLib846 {
class CurrentSensor {
 public:
  CurrentSensor(transitionLib846::Pref<units::ampere_t>& setpoint);

  void Reset();
  void Calculate(units::ampere_t reading);

  units::ampere_t GetAvgCurrent();
  bool GetSpiked();
  void SetSpikeSetpoint(units::ampere_t setpoint);

 private:
  std::deque<units::ampere_t> cache;
  transitionLib846::Pref<units::ampere_t>& spike_setpoint_;
  units::ampere_t  current_state_;
  units::ampere_t  prev_state_;
};
}  // namespace transitionLib846

#endif  // transitionLib846_CURRENT_SENSOR_H_