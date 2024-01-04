#ifndef transitionLib846_SENDABLE_CALLBACK_H_
#define transitionLib846_SENDABLE_CALLBACK_H_

#include <wpi/sendable/Sendable.h>

#include <functional>

namespace transitionLib846 {

// Like an instant command but works when robot is disabled.
//
// Use with SmartDashboard::PutData to have buttons call functions on
// Shuffleboard.
class SendableCallback : public wpi::Sendable {
 public:
  SendableCallback(std::function<void()> callback);

  void InitSendable(wpi::SendableBuilder& builder);

 private:
  std::function<void()> callback_;
};

}  // namespace transitionLib846

#endif  // transitionLib846_SENDABLE_CALLBACK_H_