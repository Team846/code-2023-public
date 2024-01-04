#ifndef transitionLib846_TIME_H_
#define transitionLib846_TIME_H_

#include <hal/HALBase.h>
#include <units/time.h>

namespace transitionLib846 {
namespace wpilib {

// Get the current time.
units::second_t CurrentFPGATime();

}  // namespace wpilib
}  // namespace transitionLib846

#endif  // transitionLib846_TIME_H_