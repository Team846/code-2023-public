cc_library(
    name = "commands",
    srcs = [
        "drive_command.cc",
        "follow_trajectory_command.cc",
        "leds_command.cc",
        "roller_command.cc",
        "extend_command.cc",
        "auto_balance_command.cc",
        "drive_till_tilt.cc"
    ],
    hdrs = [
        "drive_command.h",
        "follow_trajectory_command.h",
        "leds_command.h",
        "roller_command.h",
        "extend_command.h",
        "auto_balance_command.h",
        "drive_till_tilt.h"
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//Offseason2910Clone:field",
        "//Offseason2910Clone/subsystems",
        "@bazelrio//libraries/cpp/wpilib/new_commands",
        "@bazelrio//libraries/cpp/wpilib/wpilibc",
    ],
)
