#include "Offseason2910Clone/funky_robot.h"

#include <frc/DSControlWord.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/button/Trigger.h>
#include <hal/Notifier.h>

#include "Offseason2910Clone/commands/drive_command.h"
#include "Offseason2910Clone/commands/extend_command.h"
#include "Offseason2910Clone/commands/follow_trajectory_command.h"
#include "Offseason2910Clone/commands/leds_command.h"
#include "Offseason2910Clone/commands/roller_command.h"
#include "Offseason2910Clone/subsystems/leds.h"
#include "frc/DataLogManager.h"
#include "frc2/command/ParallelDeadlineGroup.h"
#include "frc2/command/WaitCommand.h"
#include "transitionLib846/commons.h"
#include "transitionLib846/named.h"
#include "transitionLib846/sendable_callback.h"
#include "transitionLib846/wpilib/time.h"
#include "transitionLib846/xbox.h"


FunkyRobot::FunkyRobot() : transitionLib846::Named{"funky_robot"} {
  next_loop_time_ = transitionLib846::wpilib::CurrentFPGATime();

  int32_t status = 0;
  notifier_ = HAL_InitializeNotifier(&status);
  FRC_CheckErrorStatus(status, "{}", "InitializeNotifier");

  HAL_SetNotifierName(notifier_, "FunkyRobot", &status);
}

FunkyRobot::~FunkyRobot() {
  int32_t status = 0;
  HAL_StopNotifier(notifier_, &status);
  HAL_CleanNotifier(notifier_, &status);
}

void FunkyRobot::StartCompetition() {
  // Silence warnings related to missing joystick
  // (Doesn't do anything when connected to FMS)

  frc::DriverStation::SilenceJoystickConnectionWarning(true);

  // Disable live window
  frc::LiveWindow::DisableAllTelemetry();

  frc::DataLogManager::Start();

  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  // Add dashboard buttons
  // frc::SmartDashboard::PutData(
  //     "zero_modules", new transitionLib846::SendableCallback(
  //                         [this] { container_.drivetrain_.ZeroModules(); }));
  frc::SmartDashboard::PutData("zero_bearing",
                               new transitionLib846::SendableCallback([this] {
                                 container_.drivetrain_.SetBearing(0_deg);
                               }));
  frc::SmartDashboard::PutData(
      "zero_odometry", new transitionLib846::SendableCallback(
                           [this] { container_.drivetrain_.ZeroOdometry(); }));

  frc::SmartDashboard::PutData(
      "home_pivot", new transitionLib846::SendableCallback([this] {
        if (container_.pivot_.Initialized()) {
          container_.pivot_.subsystem()->ZeroPosition(true);
        }
      }));
  frc::SmartDashboard::PutData(
      "home_wrist", new transitionLib846::SendableCallback([this] {
        if (container_.wrist_.Initialized()) {
          container_.wrist_.subsystem()->ResetPosition();
        }
      }));

  frc::SmartDashboard::PutData(
      "all_coast", new transitionLib846::SendableCallback([this] {
        if (container_.pivot_.Initialized()) {
          container_.pivot_.subsystem()->MakeAllCoastMode();
        }
        if (container_.telescope_.Initialized()) {
          container_.telescope_.subsystem()->MakeAllCoast();
        }
      }));

  frc::SmartDashboard::PutData(
      "all_brake", new transitionLib846::SendableCallback([this] {
        if (container_.pivot_.Initialized()) {
          container_.pivot_.subsystem()->MakeAllBrakeMode();
        }
        if (container_.telescope_.Initialized()) {
          container_.telescope_.subsystem()->MakeAllBrake();
        }
      }));

  frc::SmartDashboard::PutData(
      "verify_hardware",
      new transitionLib846::SendableCallback([this] { VerifyHardware(); }));

  // Add autos here
  // Default

  auto_chooser_.AddOption("drive_auto", drive_auto_.get());

  // if swappping L/R -- RECHECK, AND SWITCH ALL
  // if swappping L/R -- RECHECK, AND SWITCH ALL
  // if swappping L/R -- RECHECK, AND SWITCH ALL
  // if swappping L/R -- RECHECK, AND SWITCH ALL

  auto_chooser_.SetDefaultOption("scoring_bump_side_blue",
                                 scoring_auto_R.get());

  auto_chooser_.AddOption("scoring_bump_side_red", scoring_auto_L.get());

  auto_chooser_.AddOption("scoring_free_side_red", scoring_auto_R.get());

  auto_chooser_.AddOption("scoring_free_side_blue", scoring_auto_L.get());

  auto_chooser_.AddOption("past_balance_auto", balance_auto.get());

  // if swappping L/R -- RECHECK, AND SWITCH ALL
  // if swappping L/R -- RECHECK, AND SWITCH ALL
  // if swappping L/R -- RECHECK, AND SWITCH ALL
  // if swappping L/R -- RECHECK, AND SWITCH ALL

  // Other options
  frc::SmartDashboard::PutData(&auto_chooser_);

  // Verify robot hardware
  VerifyHardware();

  // Set initial target for all subsystems to zero.
  for (auto subsystem : container_.all_subsystems_) {
    subsystem->SetTargetZero();
  }

  // Report to driver station that robot is ready
  Debug("\n********** Funky robot initialized **********\n");
  HAL_ObserveUserProgramStarting();

  for (;;) {
    frc::DriverStation::RefreshData();
    next_loop_time_ += kPeriod;

    // Set new notifier time
    int32_t status = 0;
    HAL_UpdateNotifierAlarm(notifier_, next_loop_time_.to<uint64_t>(), &status);
    FRC_CheckErrorStatus(status, "{}", "UpdateNotifierAlarm");

    // Wait for notifier
    auto time = HAL_WaitForNotifierAlarm(notifier_, &status);
    FRC_CheckErrorStatus(status, "{}", "WaitForNotifierAlarm");

    if (time == 0 || status != 0) {
      break;
    }

    // Start loop timing
    auto loop_start_time = transitionLib846::wpilib::CurrentFPGATime();

    // Get current control mode
    frc::DSControlWord word{};
    Mode mode = Mode::kNone;
    if (word.IsDisabled()) {
      HAL_ObserveUserProgramDisabled();
      mode = Mode::kDisabled;
    } else if (word.IsAutonomous()) {
      HAL_ObserveUserProgramAutonomous();
      mode = Mode::kAutonomous;
    } else if (word.IsTeleop()) {
      HAL_ObserveUserProgramTeleop();
      mode = Mode::kTeleop;
    } else if (word.IsTest()) {
      HAL_ObserveUserProgramTest();
      mode = Mode::kTest;
    }

    // If mode changed
    if (last_mode_ != mode) {
      if (mode == Mode::kDisabled) {
        // Clear command scheduler
        Debug("Clearing command scheduler");
        frc2::CommandScheduler::GetInstance().CancelAll();
        frc::EventLoop loop;
        if (container_.telescope_.Initialized() &&
            container_.telescope_.subsystem()->has_zeroed) {
          if (container_.roller_.Initialized() &&
              container_.roller_.subsystem()->GetCubeMode()) {
            container_.leds_.SetTarget({LEDsState::kCube, true});
          } else {
            container_.leds_.SetTarget({LEDsState::kCone, true});
          }
        } else {
          container_.leds_.SetTarget({LEDsState::kNotZeroed, false});
        }
        if (container_.pivot_.Initialized() &&
            container_.telescope_.Initialized() &&
            container_.wrist_.Initialized()) {
          container_.pivot_.SetTargetZero();
          container_.telescope_.SetTargetZero();
          container_.wrist_.SetTargetZero();
        }
        loop.Clear();
      } else if (mode == Mode::kAutonomous) {
        frc2::CommandScheduler::GetInstance().CancelAll();
        if (container_.pivot_.Initialized()) {
          container_.pivot_.subsystem()->MakeAllBrakeMode();
        }
        if (container_.telescope_.Initialized()) {
          container_.telescope_.subsystem()->MakeAllBrake();
        }
        // Rezero

        // Get and run selected auto command
        auto_command_ = auto_chooser_.GetSelected();

        if (auto_command_ != nullptr) {
          Debug("Running auto: {}", auto_command_->GetName());
          auto_command_->Schedule();
        } else {
          Error("Auto command null!");
        }
      } else if (mode == Mode::kTeleop) {
        if (container_.pivot_.Initialized()) {
          container_.pivot_.subsystem()->MakeAllBrakeMode();
        }
        if (container_.telescope_.Initialized()) {
          container_.telescope_.subsystem()->MakeAllBrake();
        }
        // Cancel auto command and setup teleop defaults/triggers
        if (auto_command_ != nullptr) {
          Debug("Cancelling auto command");
          auto_command_->Cancel();
          auto_command_ = nullptr;
        }

        Debug("Setting up teleop default/triggers");
        InitTeleopDefaults();
        InitTeleopTriggers();
      } else if (mode == Mode::kTest) {
        /// Cancel auto command and setup Test defaults/triggers
        if (auto_command_ != nullptr) {
          Debug("Cancelling auto command");
          auto_command_->Cancel();
          auto_command_ = nullptr;
        }

        Debug("Setting up test default/triggers");
        // Different defaults as Teleop
        InitTestDefaults();
        // Same triggers as Teleop
        InitTeleopTriggers();
      }

      last_mode_ = mode;
    }

    // Update subsystem readings
    for (auto subsystem : container_.all_subsystems_) {
      subsystem->UpdateReadings();
    }

    // Tick command scheduler
    frc2::CommandScheduler::GetInstance().Run();

    // Update subsystem hardware
    for (auto subsystem : container_.all_subsystems_) {
      subsystem->UpdateHardware();
    }

    // ZeroAllSubsystems
    // Needs to work when disabled, so this is not in the command scheduler.
    if (frc::RobotController::GetUserButton()) {
      if (container_.wrist_.Initialized()) {
        container_.wrist_.subsystem()->ResetPosition();
      }
      if (container_.pivot_.Initialized()) {
        container_.pivot_.subsystem()->ZeroPosition(true);
      }
      if (container_.telescope_.Initialized()) {
        container_.telescope_.subsystem()->SetHasZeroed(true);
      }
      if (container_.leds_.Initialized()) {
        if (container_.telescope_.Initialized() &&
            container_.telescope_.subsystem()->has_zeroed) {
          if (container_.roller_.Initialized() &&
              container_.roller_.subsystem()->GetCubeMode()) {
            container_.leds_.SetTarget({LEDsState::kCube, true});
          } else {
            container_.leds_.SetTarget({LEDsState::kCone, true});
          }
        } else {
          container_.leds_.SetTarget({LEDsState::kNotZeroed, false});
        }
      }
    }

    // Update dashboards
    frc::SmartDashboard::UpdateValues();
    frc::Shuffleboard::Update();

    // Update graphs
    time_remaining_graph_.Graph(frc::DriverStation::GetMatchTime());

    warnings_graph_.Graph(transitionLib846::Named::warn_count());
    errors_graph_.Graph(transitionLib846::Named::error_count());

    can_usage_graph_.Graph(
        frc::RobotController::GetCANStatus().percentBusUtilization * 100);

    auto loop_time =
        transitionLib846::wpilib::CurrentFPGATime() - loop_start_time;
    loop_time_graph_.Graph(transitionLib846::wpilib::CurrentFPGATime() -
                           loop_start_time);

    // Check loop time
    if (loop_time > kPeriod * 2) {
      Warn("Bad loop overrun: {} (loop period: {})",
           loop_time.convert<units::millisecond>(), kPeriod);
    }
  }
}

void FunkyRobot::EndCompetition() {
  Debug("\n********** Robot code ending **********\n");
}

void FunkyRobot::InitTeleopDefaults() {
  container_.drivetrain_.SetDefaultCommand(DriveCommand{container_});

  if (container_.leds_.Initialized()) {
    container_.leds_.SetDefaultCommand(LEDsCommand{container_});
  }
}

void FunkyRobot::InitTeleopTriggers() {
  // WARNING: Driver left_bumper & right_bumper already taken
  frc2::Trigger drivetrain_zero_bearing_trigger{
      [&] { return container_.driver_.readings().back_button; }};

  // Cone vs Cube mode (Operator)
  frc2::Trigger cube_mode_toggle{
      [&] { return container_.operator_.readings().right_trigger; }};

  frc2::Trigger rollers_out_trigger{
      [&] { return container_.operator_.readings().left_trigger; }};

  // Operator wrist controls
  frc2::Trigger wrist_plus_five{[&] {
    return container_.operator_.readings().pov ==
           transitionLib846::XboxPOV::kUp;
  }};
  frc2::Trigger wrist_minus_five{[&] {
    return container_.operator_.readings().pov ==
           transitionLib846::XboxPOV::kDown;
  }};

  frc2::Trigger wrist_zero{
      [&] { return container_.operator_.readings().start_button; }};

  // Operator pivot controls
  frc2::Trigger pivot_up{
      [&] { return container_.operator_.readings().y_button; }};
  frc2::Trigger pivot_down{
      [&] { return container_.operator_.readings().x_button; }};

  // Operator
  frc2::Trigger tele_out{
      [&] { return container_.operator_.readings().b_button; }};
  frc2::Trigger tele_in{
      [&] { return container_.operator_.readings().a_button; }};

  frc2::Trigger has_piece_trigger{
      [&] { return container_.roller_.readings().has_piece; }};

  // Bind Triggers to commands
  drivetrain_zero_bearing_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.drivetrain_.ZeroBearing();
      }).ToPtr());

  // TODO - add all the telescope + pivot commands

  frc2::Trigger cone_high_trigger{[&] {
    return !container_.roller_.subsystem()->GetCubeMode() &&
           container_.driver_.readings().y_button;
  }};

  frc2::Trigger cone_mid_trigger{[&] {
    return !container_.roller_.subsystem()->GetCubeMode() &&
           container_.driver_.readings().x_button;
  }};

  frc2::Trigger cone_low_trigger{[&] {
    return !container_.roller_.subsystem()->GetCubeMode() &&
           container_.driver_.readings().pov ==
               transitionLib846::XboxPOV::kDown;
  }};

  frc2::Trigger cone_shelf_trigger{[&] {
    return !container_.roller_.subsystem()->GetCubeMode() &&
           container_.driver_.readings().b_button;
  }};

  // frc2::Trigger cone_shelf_min_trigger{
  //   [&] {return !container_.roller_.subsystem()->GetCubeMode() &&
  //   container_.driver_.readings().left_bumper; }
  // };

  frc2::Trigger cone_intake_trigger{[&] {
    return !container_.roller_.subsystem()->GetCubeMode() &&
           container_.driver_.readings().left_trigger;
  }};

  frc2::Trigger cone_single_trigger{[&] {
    return !container_.roller_.subsystem()->GetCubeMode() &&
           container_.driver_.readings().pov == transitionLib846::XboxPOV::kUp;
  }};

  frc2::Trigger cube_high_trigger{[&] {
    return container_.roller_.subsystem()->GetCubeMode() &&
           container_.driver_.readings().y_button;
  }};

  frc2::Trigger cube_mid_trigger{[&] {
    return container_.roller_.subsystem()->GetCubeMode() &&
           container_.driver_.readings().x_button;
  }};

  frc2::Trigger cube_low_trigger{[&] {
    return container_.roller_.subsystem()->GetCubeMode() &&
           container_.driver_.readings().pov ==
               transitionLib846::XboxPOV::kRight;
  }};

  frc2::Trigger cube_shelf_trigger{[&] {
    return container_.roller_.subsystem()->GetCubeMode() &&
           container_.driver_.readings().b_button;
  }};

  // frc2::Trigger cube_shelf_min_trigger{
  //   [&] {return container_.roller_.subsystem()->GetCubeMode() &&
  //   container_.driver_.readings().left_bumper; }
  // };

  frc2::Trigger cube_intake_trigger{[&] {
    return container_.roller_.subsystem()->GetCubeMode() &&
           container_.driver_.readings().left_trigger;
  }};

  frc2::Trigger cube_single_trigger{[&] {
    return !container_.roller_.subsystem()->GetCubeMode() &&
           container_.driver_.readings().pov == transitionLib846::XboxPOV::kUp;
  }};

  frc2::Trigger stow_trigger{
      [&] { return container_.driver_.readings().a_button; }};

  // Roller
  if (container_.roller_.Initialized()) {
    // cube_mode_toggle.OnTrue(frc2::InstantCommand([this] {
    // container_.roller_.subsystem() -> ToggleCubeMode(true); }).ToPtr());
    // cube_mode_toggle.OnFalse(frc2::InstantCommand([this] {
    // container_.roller_.subsystem() -> ToggleCubeMode(false); }).ToPtr());

    cube_mode_toggle.OnTrue(frc2::InstantCommand([this] {
                              container_.roller_.subsystem()->ToggleCubeMode();
                            }).ToPtr());

    rollers_out_trigger.OnTrue(
        frc2::InstantCommand([this] {
          container_.roller_.subsystem()->SetTarget(
              container_.roller_.subsystem()->ReleaseTarget());
        }).ToPtr());

    rollers_out_trigger.OnFalse(
        frc2::InstantCommand([this] {
          container_.roller_.subsystem()->SetTarget(
              container_.roller_.subsystem()->GrabTarget());
        }).ToPtr());

    has_piece_trigger.OnTrue(
        frc2::InstantCommand([this] { container_.operator_.SetTarget({true}); })
            .WithTimeout(1_s)
            .AndThen(frc2::WaitCommand(1_s).ToPtr())
            .AndThen(frc2::InstantCommand([this] {
                       container_.operator_.SetTarget({false});
                     }).ToPtr()));
  }

  if (container_.wrist_.Initialized()) {
    wrist_plus_five.OnTrue(
        frc2::InstantCommand([this] {
          container_.wrist_.SetTarget(
              {container_.wrist_.subsystem()->IncrementPosition(5_deg)});
        }).ToPtr());
    wrist_minus_five.OnTrue(
        frc2::InstantCommand([this] {
          container_.wrist_.SetTarget(
              {container_.wrist_.subsystem()->IncrementPosition(-5_deg)});
        }).ToPtr());
    wrist_zero.OnTrue(frc2::InstantCommand([this] {
                        container_.wrist_.subsystem()->ResetPosition();
                      }).ToPtr());
  }

  if (container_.pivot_.Initialized()) {
    pivot_up.OnTrue(frc2::InstantCommand([this] {
                      container_.pivot_.SetTarget(
                          {container_.pivot_.subsystem()->UpTarget()});
                    }).ToPtr());
    pivot_up.OnFalse(frc2::InstantCommand([this] {
                       container_.pivot_.SetTarget(
                           {container_.pivot_.subsystem()->HoldPosition()});
                     }).ToPtr());
    pivot_down.OnTrue(frc2::InstantCommand([this] {
                        container_.pivot_.SetTarget(
                            {container_.pivot_.subsystem()->DownTarget()});
                      }).ToPtr());
    pivot_down.OnFalse(frc2::InstantCommand([this] {
                         container_.pivot_.SetTarget(
                             {container_.pivot_.subsystem()->HoldPosition()});
                       }).ToPtr());
  }

  if (container_.telescope_.Initialized()) {
    tele_out.OnTrue(frc2::InstantCommand([this] {
                      container_.telescope_.SetTarget(
                          {container_.telescope_.subsystem()->OutTarget()});
                    }).ToPtr());
    tele_out.OnFalse(frc2::InstantCommand([this] {
                       container_.telescope_.SetTarget(
                           {container_.telescope_.subsystem()->HoldPosition()});
                     }).ToPtr());
    tele_in.OnTrue(frc2::InstantCommand([this] {
                     container_.telescope_.SetTarget(
                         {container_.telescope_.subsystem()->InTarget()});
                   }).ToPtr());
    tele_in.OnFalse(frc2::InstantCommand([this] {
                      container_.telescope_.SetTarget(
                          {container_.telescope_.subsystem()->HoldPosition()});
                    }).ToPtr());
  }

  if (container_.telescope_.Initialized() && container_.pivot_.Initialized() &&
      container_.wrist_.Initialized()) {
    cone_high_trigger.OnTrue(
        ExtendCommand{container_, container_.pivot_.subsystem()->cone_high_,
                      container_.telescope_.subsystem()->cone_high_position_,
                      container_.wrist_.subsystem()->cone_high_position_}
            .ToPtr());
    cone_mid_trigger.OnTrue(
        ExtendCommand{container_, container_.pivot_.subsystem()->cone_mid_,
                      container_.telescope_.subsystem()->cone_mid_position_,
                      container_.wrist_.subsystem()->cone_mid_position_}
            .ToPtr());
    cone_low_trigger.OnTrue(ExtendCommand{
        container_, container_.pivot_.subsystem()->cone_intake_low_,
        container_.telescope_.subsystem()->cone_intake_position_,
        container_.wrist_.subsystem()->cone_intake_position_}
                                .ToPtr());
    cone_shelf_trigger.OnTrue(
        ExtendCommand{container_, container_.pivot_.subsystem()->cone_shelf_,
                      container_.telescope_.subsystem()->cone_shelf_position_,
                      container_.wrist_.subsystem()->cone_shelf_position_}
            .ToPtr());
    cone_intake_trigger.OnTrue(ExtendCommand{
        container_, container_.pivot_.subsystem()->cone_intake_low_,
        container_.telescope_.subsystem()->cone_intake_position_,
        container_.wrist_.subsystem()->cone_intake_position_}
                                   .ToPtr());
    cone_single_trigger.OnTrue(
        ExtendCommand{container_, container_.pivot_.subsystem()->cone_single_,
                      container_.telescope_.subsystem()->cone_single_position_,
                      container_.wrist_.subsystem()->cone_single_position_}
            .ToPtr());
    // cone_shelf_min_trigger.OnTrue(ExtendCommand{container_,
    // container_.pivot_.subsystem()->cone_shelf_min_,
    // container_.telescope_.subsystem()->cone_shelf_position_,
    // container_.wrist_.subsystem()->cone_shelf_position_}.ToPtr());

    cube_high_trigger.OnTrue(
        ExtendCommand{container_, container_.pivot_.subsystem()->cube_high_,
                      container_.telescope_.subsystem()->cube_high_position_,
                      container_.wrist_.subsystem()->cube_place_position_}
            .ToPtr());
    cube_mid_trigger.OnTrue(
        ExtendCommand{container_, container_.pivot_.subsystem()->cube_mid_,
                      container_.telescope_.subsystem()->cube_mid_position_,
                      container_.wrist_.subsystem()->cube_place_position_}
            .ToPtr());
    cube_low_trigger.OnTrue(ExtendCommand{
        container_, container_.pivot_.subsystem()->cube_intake_low_,
        container_.telescope_.subsystem()->cube_intake_position_,
        container_.wrist_.subsystem()->cube_intake_position_}
                                .ToPtr());
    cube_shelf_trigger.OnTrue(
        ExtendCommand{container_, container_.pivot_.subsystem()->cube_shelf_,
                      container_.telescope_.subsystem()->cube_shelf_position_,
                      container_.wrist_.subsystem()->cube_shelf_position_}
            .ToPtr());
    cube_intake_trigger.OnTrue(ExtendCommand{
        container_, container_.pivot_.subsystem()->cube_intake_low_,
        container_.telescope_.subsystem()->cube_intake_position_,
        container_.wrist_.subsystem()->cube_intake_position_}
                                   .ToPtr());
    cube_single_trigger.OnTrue(
        ExtendCommand{container_, container_.pivot_.subsystem()->cube_single_,
                      container_.telescope_.subsystem()->cube_single_position_,
                      container_.wrist_.subsystem()->cube_single_position_}
            .ToPtr());
    // cube_shelf_min_trigger.OnTrue(ExtendCommand{container_,
    // container_.pivot_.subsystem()->cube_shelf_min_,
    // container_.telescope_.subsystem()->cube_shelf_position_,
    // container_.wrist_.subsystem()->cube_shelf_position_}.ToPtr());

    stow_trigger.OnTrue(
        ExtendCommand{container_, container_.pivot_.subsystem()->stow_position_,
                      container_.telescope_.subsystem()->retract_position_,
                      container_.wrist_.subsystem()->idle_position_}
            .ToPtr());
  }
}

void FunkyRobot::InitTestDefaults() {
  container_.drivetrain_.SetDefaultCommand(DriveCommand{container_});
}

void FunkyRobot::VerifyHardware() {
  Debug("Verifying hardware...");
  for (auto subsystem : container_.all_subsystems_) {
    bool ok = subsystem->VerifyHardware();
    if (!ok) {
      subsystem->Error("Failed hardware verification!!");
    }
  }
  Debug("Done verifying hardware");
}
