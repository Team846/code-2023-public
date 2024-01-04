cc_library(
    name = "subsystems",
    srcs = [
        "driver.cc",
        "drivetrain.cc",
        "operator.cc",
        "swerve_module.cc",
        "leds.cc",
        "limelight.cc",
        "roller.cc",
        "pivot.cc", 
        "wrist.cc", 
        "telescope.cc",
    ],
    hdrs = [
        "driver.h",
        "drivetrain.h",
        "operator.h",
        "robot_container.h",
        "swerve_module.h",
        "leds.h",
        "limelight.h",
        "roller.h",
        "pivot.h",
        "wrist.h", 
        "telescope.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//transitionLib846",
        "//transitionLib846/motor",
        "//transitionLib846/wpilib",
        "//Offseason2910Clone:field",
        "//Offseason2910Clone:ports",
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
