# code-2023-public

Snapshot of Team 846's robot code as of 7/10/23
 - Code run at SVR(mostly)
 - Added a few small fixes for new bugs
 - Minor cleanup and formatting

Features
 - Command based architecture with C++/bazel using [bazelrio](https://github.com/bazelRio/bazelRio/)
 - 5 different auto routines
 - Custom swerve drive control, trajectory generator, and follower
 - Hot configurable preferences
 - Motor controller wrappers to cache gains/configurations and reduce CAN usage
 
---

## Setup
1. install bazel
2. install clangd vscode extension (not microsoft c++ extension)
3. run `bazel run @hedron_compile_commands//:refresh_all` for c++ autocompletion
4. restart vscode

## Build for roborio
`bazel build //y2023:y2023.deploy --platforms=@bazelrio//platforms/roborio`

## Deploy to roborio
`bazel run //y2023:y2023.deploy --platforms=@bazelrio//platforms/roborio`

## Run desktop simulation
`bazel run //y2023:y2023.simulation.gui`

## Run unit tests
`bazel test --test_output=all //...`

