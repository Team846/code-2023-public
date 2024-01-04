cc_library(
    name = "subsystems",
    srcs = [
        "conveyor.cc",
        "driver.cc",
        "drivetrain.cc",
        "gripper.cc",
        "operator.cc",
        "pivot.cc",
        "swerve_module.cc",
        "telescope.cc",
        "intake.cc",
        "wrist.cc",
        "leds.cc",
        "limelight.cc",
    ],
    hdrs = [
        "conveyor.h",
        "driver.h",
        "drivetrain.h",
        "gripper.h",
        "operator.h",
        "pivot.h",
        "robot_container.h",
        "swerve_module.h",
        "telescope.h",
        "intake.h",
        "wrist.h",
        "leds.h",
        "limelight.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//frc846",
        "//frc846/motor",
        "//frc846/wpilib",
        "//y2023:field",
        "//y2023:ports",
        "@bazelrio//libraries/cpp/ctre/phoenix",
        "@bazelrio//libraries/cpp/kauailabs/navx",
        "@bazelrio//libraries/cpp/wpilib/new_commands",
        "@bazelrio//libraries/cpp/wpilib/wpilibc",
    ],
)

cc_test(
    name = "drivetrain_test",
    size = "small",
    srcs = [
        "drivetrain_test.cc",
    ],
    linkstatic = 1,
    deps = [
        ":subsystems",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "swerve_module_test",
    size = "small",
    srcs = [
        "swerve_module_test.cc",
    ],
    linkstatic = 1,
    deps = [
        ":subsystems",
        "@com_google_googletest//:gtest_main",
    ],
)
