#ifndef Offseason2910Clone_FUNKY_ROBOT_H_
#define Offseason2910Clone_FUNKY_ROBOT_H_

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <hal/Types.h>
#include <units/time.h>

#include "Offseason2910Clone/autos/balance_auto.h"
#include "Offseason2910Clone/autos/drive_auto.h"
#include "Offseason2910Clone/autos/scoring_auto.h"
#include "Offseason2910Clone/subsystems/robot_container.h"
#include "transitionLib846/pref.h"


enum Mode { kNone, kDisabled, kAutonomous, kTeleop, kTest };

class FunkyRobot : public frc::RobotBase, public transitionLib846::Named {
 public:
  static constexpr auto kPeriod = 20_ms;  // 50hz

  FunkyRobot();

  ~FunkyRobot() override;

  void StartCompetition() override;
  void EndCompetition() override;

  void InitTeleopDefaults();
  void InitTeleopTriggers();

  void InitTestDefaults();

  void VerifyHardware();

 private:
  hal::Handle<HAL_NotifierHandle> notifier_;
  units::microsecond_t next_loop_time_;

  Mode last_mode_;

 private:
  RobotContainer container_;

  transitionLib846::Grapher<int> time_remaining_graph_{*this, "time"};

  transitionLib846::Grapher<int> warnings_graph_{*this, "warnings"};
  transitionLib846::Grapher<int> errors_graph_{*this, "errors"};

  transitionLib846::Grapher<double> can_usage_graph_{*this, "CAN_usage"};
  transitionLib846::Grapher<units::millisecond_t> loop_time_graph_{*this,
                                                                   "loop_time"};

  frc2::Command* auto_command_ = nullptr;
  frc::SendableChooser<frc2::Command*> auto_chooser_;

  // TODO: Find fix for this
  // Seperate blue vs red because command is generated prior to the alliance
  // color being changed

  // Scoring
  frc2::CommandPtr drive_auto_ = DriveAuto{container_, true}.ToPtr();

  frc2::CommandPtr scoring_auto_L = ScoringAuto{container_, true}.ToPtr();

  frc2::CommandPtr scoring_auto_R = ScoringAuto{container_, false}.ToPtr();

  frc2::CommandPtr balance_auto = BalanceAuto{container_, false}.ToPtr();
};

#endif  // Offseason2910Clone_FUNKY_ROBOT_H_