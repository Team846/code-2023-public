load("@bazelrio//:defs.bzl", "robot_cc_binary")

cc_library(
    name = "ports",
    hdrs = [
        "ports.h",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "field",
    srcs = [
        "field.cc"
    ],
    hdrs = [
        "field.h",
    ],
    visibility = ["//visibility:public"],
    deps = ["//transitionLib846"],
)

robot_cc_binary(
    name = "Offseason2910Clone",
    srcs = [
        "funky_robot.cc",
        "funky_robot.h",
        "main.cc",
    ],
    halsim_configs = {
        "gui": ["@bazelrio//libraries/cpp/wpilib/simulation/halsim_gui"],
    },
    linkopts = ["-ldl"],
    team_number = 846,
    deps = [
        "//transitionLib846",
        "//Offseason2910Clone/autos",
        "//Offseason2910Clone/commands",
        "//Offseason2910Clone/subsystems",
    ],
)
