#include "y2023/funky_robot.h"

#include <frc/DSControlWord.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/button/Trigger.h>
#include <hal/Notifier.h>

#include "frc/DataLogManager.h"
#include "frc2/command/ParallelDeadlineGroup.h"
#include "frc2/command/WaitCommand.h"
#include "frc846/named.h"
#include "frc846/sendable_callback.h"
#include "frc846/wpilib/time.h"
#include "frc846/xbox.h"
#include "y2023/commands/auto_balance_command.h"
#include "y2023/commands/base_pickup_command.h"
#include "y2023/commands/conveyor_command.h"
#include "y2023/commands/drive_command.h"
#include "y2023/commands/extend_command.h"
#include "y2023/commands/follow_trajectory_command.h"
#include "y2023/commands/gripper_command.h"
#include "y2023/commands/intake_command.h"
#include "y2023/commands/leds_command.h"
#include "y2023/commands/load_conveyor_command.h"
#include "y2023/commands/manual_pivot_command.h"
#include "y2023/commands/manual_telescope_command.h"
#include "y2023/commands/pickup_conveyor_command.h"
#include "y2023/commands/place_command.h"
#include "y2023/commands/position_pivot_command.h"
#include "y2023/commands/position_telescope_command.h"
#include "y2023/commands/retract_command.h"
#include "y2023/commands/wrist_command.h"
#include "y2023/commands/wrist_hold_command.h"
#include "y2023/subsystems/intake.h"
#include "y2023/subsystems/leds.h"
#include "y2023/subsystems/wrist.h"

FunkyRobot::FunkyRobot() : frc846::Named{"funky_robot"} {
  next_loop_time_ = frc846::wpilib::CurrentFPGATime();

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
  //     "zero_modules", new frc846::SendableCallback(
  //                         [this] { container_.drivetrain_.ZeroModules(); }));
  frc::SmartDashboard::PutData("zero_bearing",
                               new frc846::SendableCallback([this] {
                                 container_.drivetrain_.SetBearing(0_deg);
                               }));
  frc::SmartDashboard::PutData(
      "zero_odometry", new frc846::SendableCallback(
                           [this] { container_.drivetrain_.ZeroOdometry(); }));
  frc::SmartDashboard::PutData("zero_pivot",
                               new frc846::SendableCallback([this] {
                                 if (container_.pivot_.Initialized()) {
                                   container_.pivot_.subsystem()->Zero();
                                 }
                               }));
  frc::SmartDashboard::PutData(
      "zero_telescope", new frc846::SendableCallback([this] {
        container_.telescope_.subsystem()->SetHasZeroed(true);
        container_.telescope_.subsystem()->ResetPosition();
      }));

  frc::SmartDashboard::PutData(
      "home_telescope", new frc846::SendableCallback([this] {
        container_.telescope_.subsystem()->SetHasZeroed(false);
      }));
  frc::SmartDashboard::PutData(
      "home_intake", new frc846::SendableCallback([this] {
        container_.intake_.subsystem()->SetHasHomed(false);
      }));
  frc::SmartDashboard::PutData(
      "zero_intake", new frc846::SendableCallback([this] {
        container_.intake_.subsystem()->ResetPosition();
      }));

  frc::SmartDashboard::PutData(
      "verify_hardware",
      new frc846::SendableCallback([this] { VerifyHardware(); }));

  // Add autos here
  // Default

  auto_chooser_.AddOption("drive_auto_red",
                                 drive_auto_.get());

  auto_chooser_.SetDefaultOption("scoring_two_piece_auto_red",
                                 scoring_two_piece_auto_red_.get());
  auto_chooser_.AddOption("scoring_two_piece_auto_blue",
                          scoring_two_piece_auto_blue_.get());

// Alliance
  // Alliance
  auto_chooser_.AddOption("alliance_two_piece_auto_red",
                          alliance_two_piece_auto_red_.get());
  auto_chooser_.AddOption("alliance_two_piece_auto_blue",
                          alliance_two_piece_auto_blue_.get());

  auto_chooser_.AddOption("alliance_two_piece_dock_auto_red",
                          alliance_two_piece_dock_auto_red_.get());
  auto_chooser_.AddOption("alliance_two_piece_dock_auto_blue",
                          alliance_two_piece_dock_auto_blue_.get());

  // Dock
  auto_chooser_.AddOption("mid_one_piece_dock_auto_red",
                          mid_one_piece_dock_auto_red_.get());
  auto_chooser_.AddOption("mid_one_piece_dock_auto_blue",
                          mid_one_piece_dock_auto_blue_.get());
  auto_chooser_.AddOption("mid_one_piece_pickup_dock_auto_red",
                          mid_one_piece_pickup_dock_auto_red_.get());
  auto_chooser_.AddOption("mid_one_piece_pickup_dock_auto_blue",
                          mid_one_piece_pickup_dock_auto_blue_.get());

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
    auto loop_start_time = frc846::wpilib::CurrentFPGATime();

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
        if (container_.pivot_.Initialized() &&
            container_.telescope_.Initialized()) {
          container_.pivot_.SetTargetZero();
          container_.telescope_.SetTargetZero();
        }
        container_.leds_.SetTarget(
            {LEDsState::kCone, LEDsState::kNoPiece,
             container_.telescope_.subsystem()->GetHasZeroed()});
        loop.Clear();
      } else if (mode == Mode::kAutonomous) {
        // Rezero
        if (container_.pivot_.Initialized()) {
          container_.pivot_.subsystem()->Zero();
          container_.pivot_.subsystem()->SetTargetZero();
        }
        if (container_.telescope_.Initialized()) {
          container_.telescope_.subsystem()->SetTargetZero();
          // container_.telescope_.subsystem()->SetHasZeroed(false);
        }

        // Get and run selected auto command
        auto_command_ = auto_chooser_.GetSelected();

        if (auto_command_ != nullptr) {
          Debug("Running auto: {}", auto_command_->GetName());
          auto_command_->Schedule();
        } else {
          Error("Auto command null!");
        }
      } else if (mode == Mode::kTeleop) {
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

    // ZeroTelescope
    // Needs to work when disabled, so this is not in the command scheduler.
    if (frc::RobotController::GetUserButton()) {
      container_.telescope_.subsystem()->ResetPosition();
      container_.telescope_.subsystem()->SetHasZeroed(true);
      container_.leds_.SetTarget(
          {LEDsState::kCone, LEDsState::kNoPiece,
           container_.telescope_.subsystem()->GetHasZeroed()});
    }

    // Update dashboards
    frc::SmartDashboard::UpdateValues();
    frc::Shuffleboard::Update();

    // Update graphs
    time_remaining_graph_.Graph(frc::DriverStation::GetMatchTime());

    warnings_graph_.Graph(frc846::Named::warn_count());
    errors_graph_.Graph(frc846::Named::error_count());

    can_usage_graph_.Graph(
        frc::RobotController::GetCANStatus().percentBusUtilization * 100);

    auto loop_time = frc846::wpilib::CurrentFPGATime() - loop_start_time;
    loop_time_graph_.Graph(frc846::wpilib::CurrentFPGATime() - loop_start_time);

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
  if (container_.conveyor_.Initialized()) {
    container_.conveyor_.SetDefaultCommand(LoadConveyorCommand{
        container_, container_.conveyor_.subsystem()->should_run_});
    container_.gripper_.SetDefaultCommand(PickupConveyorCommand{
        container_, container_.gripper_.subsystem()->should_auto_pickup_});
    container_.wrist_.SetDefaultCommand(WristHoldCommand{container_});
  }

  if (container_.leds_.Initialized()) {
    container_.leds_.SetDefaultCommand(LEDsCommand{container_});
  }
  if (container_.intake_.Initialized()) {
    container_.intake_.SetDefaultCommand(
        IntakeCommand{container_,
                      container_.intake_.subsystem()->should_auto_stow_.value()
                          ? IntakeState::kStow
                          : IntakeState::kIdle,
                      container_.intake_.subsystem()->zero_roller_speed_});
  }
}

void FunkyRobot::InitTeleopTriggers() {
  if (container_.pivot_.Initialized()) {
    container_.pivot_.subsystem()->Zero();
    container_.pivot_.subsystem()->SetTargetZero();
  }

  if (container_.telescope_.Initialized()) {
    container_.telescope_.subsystem()->SetTargetZero();
  }

  // WARNING: Driver left_bumper & right_bumper already taken
  frc2::Trigger drivetrain_zero_bearing_trigger{
      [&] { return container_.driver_.readings().back_button; }};

  // Intake (Driver)
  frc2::Trigger deploy_cone_intake_trigger{[&] {
    return container_.driver_.readings().left_trigger &&
           !container_.operator_.readings().left_trigger;
  }};

  frc2::Trigger deploy_cube_intake_trigger{[&] {
    return container_.driver_.readings().left_trigger &&
           container_.operator_.readings().left_trigger;
  }};

  // Cone vs Cube mode (Operator)
  frc2::Trigger cone_mode_trigger{
      [&] { return !container_.operator_.readings().left_trigger; }};

  frc2::Trigger cube_mode_trigger{
      [&] { return container_.operator_.readings().left_trigger; }};

  // Arm Setpoint Triggers (Driver)
  frc2::Trigger cone_shelf_trigger{[&] {
    return container_.driver_.readings().b_button &&
           !container_.operator_.readings().left_trigger;
  }};
  frc2::Trigger cube_shelf_trigger{[&] {
    return container_.driver_.readings().b_button &&
           container_.operator_.readings().left_trigger;
  }};
  frc2::Trigger cube_place_trigger{[&] {
    return container_.driver_.readings().left_bumper &&
           container_.operator_.readings().left_trigger;
  }};
  frc2::Trigger cone_place_trigger{[&] {
    return container_.driver_.readings().left_bumper &&
           !container_.operator_.readings().left_trigger;
  }};

  frc2::Trigger cone_retract_trigger{[&] {
    return container_.driver_.readings().a_button &&
           !container_.operator_.readings().left_trigger;
  }};
  frc2::Trigger cube_retract_trigger{[&] {
    return container_.driver_.readings().a_button &&
           container_.operator_.readings().left_trigger;
  }};

  frc2::Trigger cone_mid_trigger{[&] {
    return container_.driver_.readings().x_button &&
           !container_.operator_.readings().left_trigger;
  }};
  frc2::Trigger cube_mid_trigger{[&] {
    return container_.driver_.readings().x_button &&
           container_.operator_.readings().left_trigger;
  }};

  frc2::Trigger cone_high_trigger{[&] {
    return container_.driver_.readings().y_button &&
           !container_.operator_.readings().left_trigger;
  }};

  frc2::Trigger hybrid_trigger{[&] {
    return container_.driver_.readings().pov == frc846::XboxPOV::kDown;
  }};
  frc2::Trigger cube_high_trigger{[&] {
    return container_.driver_.readings().y_button &&
           container_.operator_.readings().left_trigger;
  }};

  // Manual Arm Triggers (operator)
  // Telescope Manual (Operator)
  frc2::Trigger manual_extend_telescope_trigger{
      [&] { return container_.operator_.readings().a_button; }};
  frc2::Trigger manual_retract_telescope_trigger{ 
      [&] { return container_.operator_.readings().b_button; }};
  // Pivot Manual (Operator)
  frc2::Trigger manual_up_pivot_trigger{
      [&] { return container_.operator_.readings().x_button; }};
  frc2::Trigger manual_down_pivot_trigger{
      [&] { return container_.operator_.readings().y_button; }};

  frc2::Trigger balance_trigger{
      [&] { return container_.driver_.readings().right_trigger; }};

  // Gripper Triggers (Operator)
  frc2::Trigger gripper_out_trigger{
      [&] { return container_.operator_.readings().right_trigger; }};
  frc2::Trigger gripper_in_trigger{
      [&] { return container_.operator_.readings().right_bumper; }};

  //Gripper Triggers (Driver-tester)
  frc2::Trigger gripper_reset_trigger{
      [&] { return container_.driver_.readings().pov==frc846::XboxPOV::kLeft; }};

  frc2::Trigger has_piece_trigger{
      [&] { return container_.gripper_.readings().has_piece; }};

  // Wrist Triggers (Operator)
  frc2::Trigger wrist_counterclockwise_trigger{
      [&] { return container_.operator_.readings().back_button; }};
  frc2::Trigger wrist_clockwise_trigger{
      [&] { return container_.operator_.readings().start_button; }};

  // Conveyor Triggers (Operator)
  frc2::Trigger conveyor_intake_trigger{[&] {
    return container_.operator_.readings().left_bumper &&
           container_.operator_.readings().pov == frc846::XboxPOV::kDown;
  }};

  frc2::Trigger conveyor_forward_trigger{[&] {
    return container_.operator_.readings().pov == frc846::XboxPOV::kUp;
  }};
  frc2::Trigger conveyor_reverse_trigger{[&] {
    return container_.operator_.readings().pov == frc846::XboxPOV::kDown;
  }};

  frc2::Trigger base_pickup_trigger{[&] {
    return container_.operator_.readings().pov == frc846::XboxPOV::kLeft;
  }};

  // Handoff
  // Push piece into place
  frc2::Trigger conveyor_position_trigger{[&] {
    return container_.conveyor_.Initialized() &&
           container_.pivot_.Initialized() &&
           container_.conveyor_.subsystem()->should_run_.value() &&
           (container_.pivot_.readings().position <
            container_.pivot_.subsystem()->min_conveyor_.value());
  }};

  // Bind Triggers to commands
  drivetrain_zero_bearing_trigger.WhileTrue(
      frc2::InstantCommand([this] {
        container_.drivetrain_.ZeroBearing();
      }).ToPtr());

  // Telescope
  if (container_.telescope_.Initialized()) {
    auto telescope = container_.telescope_.subsystem();
    // Manual
    manual_extend_telescope_trigger.WhileTrue(
        ManualTelescopeCommand{container_, telescope->extend_speed_}.ToPtr());
    manual_retract_telescope_trigger.WhileTrue(
        ManualTelescopeCommand{container_, telescope->retract_speed_}.ToPtr());
  }

  // Pivot
  if (container_.pivot_.Initialized()) {
    // Manual
    manual_up_pivot_trigger.WhileTrue(
        ManualPivotCommand{container_, container_.pivot_.subsystem()->up_speed_}
            .ToPtr());
    manual_down_pivot_trigger.WhileTrue(ManualPivotCommand{
        container_, container_.pivot_.subsystem()->down_speed_}
                                            .ToPtr());
  }

  // Commands with safeties to prevent conveyor / bumper collision
  if (container_.telescope_.Initialized() && container_.pivot_.Initialized()) {
    // OnTrue: arm continues to output to stay in the retract position
    ((cube_retract_trigger || deploy_cube_intake_trigger))
        .OnTrue(RetractCommand{
            container_, container_.pivot_.subsystem()->cube_conveyor_position_,
            container_.telescope_.subsystem()->cube_conveyor_position_}
                    .ToPtr());

    (cone_retract_trigger || deploy_cone_intake_trigger)
        .OnTrue(RetractCommand{
            container_, container_.pivot_.subsystem()->cube_conveyor_position_,
            container_.telescope_.subsystem()->cube_conveyor_position_}
                    .ToPtr());

    // Shelf
    cone_shelf_trigger.OnTrue(ExtendCommand{
        container_, container_.pivot_.subsystem()->cone_shelf_position_,
        container_.telescope_.subsystem()->shelf_position_,
        container_.pivot_.subsystem()->min_pivot_}
                                  .ToPtr());
    cube_shelf_trigger.OnTrue(ExtendCommand{
        container_, container_.pivot_.subsystem()->cube_shelf_position_,
        container_.telescope_.subsystem()->shelf_position_,
        container_.pivot_.subsystem()->min_pivot_}
                                  .ToPtr());

    // Cone
    cone_mid_trigger.OnTrue(ExtendCommand{
        container_, container_.pivot_.subsystem()->cone_mid_position_,
        container_.telescope_.subsystem()->cone_mid_position_,
        container_.pivot_.subsystem()->cone_min_mid_}
                                .ToPtr());
    cone_high_trigger.OnTrue(ExtendCommand{
        container_, container_.pivot_.subsystem()->cone_high_position_,
        container_.telescope_.subsystem()->cone_high_position_,
        container_.pivot_.subsystem()->cone_min_mid_}
                                 .ToPtr());
    hybrid_trigger.OnTrue(ExtendCommand{
        container_, container_.pivot_.subsystem()->hybrid_position_,
        container_.telescope_.subsystem()->cube_conveyor_position_,
        container_.pivot_.subsystem()->min_pivot_}
                              .ToPtr());

    // Cube
    cube_mid_trigger.OnTrue(ExtendCommand{
        container_, container_.pivot_.subsystem()->cube_mid_position_,
        container_.telescope_.subsystem()->cube_mid_position_,
        container_.pivot_.subsystem()->cube_min_mid_}
                                .ToPtr());
    cube_high_trigger.OnTrue(ExtendCommand{
        container_, container_.pivot_.subsystem()->cube_high_position_,
        container_.telescope_.subsystem()->cube_high_position_,
        container_.pivot_.subsystem()->cube_min_high_}
                                 .ToPtr());
    cone_place_trigger.WhileTrue(PlaceCommand{container_}.ToPtr());
    cube_place_trigger.WhileTrue(GripperCommand{
        container_, container_.gripper_.subsystem()->place_speed_}
                                     .ToPtr());
  }

  // Intake
  if (container_.intake_.Initialized()) {
    deploy_cone_intake_trigger.WhileTrue(
        IntakeCommand{container_, IntakeState::kDeploy,
                      container_.intake_.subsystem()->cone_roller_speed_}
            .ToPtr());
    deploy_cube_intake_trigger.WhileTrue(
        IntakeCommand{container_, IntakeState::kDeploy,
                      container_.intake_.subsystem()->cube_roller_speed_}
            .ToPtr());

    conveyor_intake_trigger.WhileTrue(
        IntakeCommand{container_, IntakeState::kDeploy,
                      container_.intake_.subsystem()->reverse_roller_speed_}
            .ToPtr());
  }

  // Gripper
  if (container_.gripper_.Initialized()) {
    auto gripper = container_.gripper_.subsystem();
    gripper_in_trigger.WhileTrue(
        GripperCommand{container_, gripper->grab_speed_}.ToPtr());
    gripper_out_trigger.WhileTrue(
        GripperCommand{container_, gripper->place_speed_}.ToPtr());
    gripper_reset_trigger.OnTrue(
        frc2::InstantCommand([this] {container_.operator_.SetTarget({false});})
            .WithTimeout(1_s)
            .AndThen(frc2::WaitCommand(1_s).ToPtr())
            .AndThen(GripperCommand{container_, gripper->reset_target_speed_}.ToPtr()));

    has_piece_trigger.OnTrue(
        frc2::InstantCommand([this] { container_.operator_.SetTarget({true}); })
            .WithTimeout(1_s)
            .AndThen(frc2::WaitCommand(1_s).ToPtr())
            .AndThen(frc2::InstantCommand([this] {
                       container_.operator_.SetTarget({false});
                     }).ToPtr()));
  }

  if (container_.wrist_.Initialized()) {
    auto wrist = container_.wrist_.subsystem();
    wrist_counterclockwise_trigger.WhileTrue(
        WristCommand{container_, wrist->unflip_speed_}.ToPtr());
    wrist_clockwise_trigger.WhileTrue(
        WristCommand{container_, wrist->flip_speed_}.ToPtr());
  }

  // Conveyor
  if (container_.conveyor_.Initialized()) {
    auto conveyor = container_.conveyor_.subsystem();
    conveyor_forward_trigger.WhileTrue(ConveyorCommand{
        container_, conveyor->belt_speed_, conveyor->default_roller_speed_}
                                           .ToPtr());
    conveyor_reverse_trigger.WhileTrue(
        ConveyorCommand{container_, conveyor->reverse_belt_speed_,
                        conveyor->reverse_roller_speed_}
            .ToPtr());
    base_pickup_trigger.WhileTrue(BasePickupCommand{container_}.ToPtr());
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
