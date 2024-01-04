#ifndef Y2023_FUNKY_ROBOT_H_
#define Y2023_FUNKY_ROBOT_H_

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <hal/Types.h>
#include <units/time.h>

#include "frc846/pref.h"
#include "y2023/autos/alliance_two_piece_auto_command.h"
#include "y2023/autos/alliance_two_piece_dock_auto_command.h"
#include "y2023/autos/mid_one_piece_dock_auto_command.h"
#include "y2023/autos/mid_one_piece_pickup_dock_auto_command.h"
#include "y2023/autos/scoring_two_piece_auto_command.h"
#include "y2023/autos/drive_auto.h"
#include "y2023/subsystems/robot_container.h"

enum Mode { kNone, kDisabled, kAutonomous, kTeleop, kTest };

class FunkyRobot : public frc::RobotBase, public frc846::Named {
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

  frc846::Grapher<int> time_remaining_graph_{*this, "time"};

  frc846::Grapher<int> warnings_graph_{*this, "warnings"};
  frc846::Grapher<int> errors_graph_{*this, "errors"};

  frc846::Grapher<double> can_usage_graph_{*this, "CAN_usage"};
  frc846::Grapher<units::millisecond_t> loop_time_graph_{*this, "loop_time"};

  frc2::Command* auto_command_ = nullptr;
  frc::SendableChooser<frc2::Command*> auto_chooser_;

  // TODO: Find fix for this
  // Seperate blue vs red because command is generated prior to the alliance
  // color being changed

  // Scoring
  frc2::CommandPtr drive_auto_ =
      DriveAuto{container_, true}.ToPtr();

  frc2::CommandPtr scoring_two_piece_auto_red_ =
      ScoringTwoPieceAutoCommand{container_, true}.ToPtr();
  frc2::CommandPtr scoring_two_piece_auto_blue_ =
      ScoringTwoPieceAutoCommand{container_, false}.ToPtr();

  // Alliance
  frc2::CommandPtr alliance_two_piece_auto_red_ =
      AllianceTwoPieceAutoCommand{container_, true}.ToPtr();
  frc2::CommandPtr alliance_two_piece_auto_blue_ =
      AllianceTwoPieceAutoCommand{container_, false}.ToPtr();

  frc2::CommandPtr alliance_two_piece_dock_auto_red_ =
      AllianceTwoPieceDockAutoCommand{container_, true}.ToPtr();
  frc2::CommandPtr alliance_two_piece_dock_auto_blue_ =
      AllianceTwoPieceDockAutoCommand{container_, false}.ToPtr();

  // Mid
  frc2::CommandPtr mid_one_piece_dock_auto_red_ =
      MidOnePieceDockAutoCommand{container_, true}.ToPtr();
  frc2::CommandPtr mid_one_piece_dock_auto_blue_ =
      MidOnePieceDockAutoCommand{container_, false}.ToPtr();

  frc2::CommandPtr mid_one_piece_pickup_dock_auto_red_ =
      MidOnePiecePickupDockAutoCommand{container_, true}.ToPtr();
  frc2::CommandPtr mid_one_piece_pickup_dock_auto_blue_ =
      MidOnePiecePickupDockAutoCommand{container_, false}.ToPtr();
};

#endif  // Y2023_FUNKY_ROBOT_H_