#include "transitionLib846/wpilib/time.h"

namespace transitionLib846::wpilib {

units::second_t CurrentFPGATime() {
  // TODO lol
  int err;
  return units::microsecond_t(HAL_GetFPGATime(&err));
}

}  // namespace transitionLib846::wpilib