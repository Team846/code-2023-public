cc_library(
    name = "motor",
    srcs = [
        "config.cc",
        "gains.cc",
        "helper.cc",
    ],
    hdrs = [
        "config.h",
        "gains.h",
        "helper.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//transitionLib846",
        "@bazelrio//libraries/cpp/ctre/phoenix",
        "@bazelrio//libraries/cpp/rev/revlib",
    ],
)
