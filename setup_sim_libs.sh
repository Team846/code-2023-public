#!/bin/sh

# Run this script to run unit tests & simulation on macOS (must run a bazel build before)
#
# symlinks wpilib libraries to /usr/local/lib

INSTALL_PATH="/usr/local/lib/"


ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenix_sim_api-cpp-sim_osxuniversal/osx/universal/shared/libCTRE_PhoenixSim.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenix_sim_cci-sim_osxuniversal/osx/universal/shared/libCTRE_PhoenixCCISim.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenix_sim_wpiapi-cpp-sim_osxuniversal/osx/universal/shared/libCTRE_Phoenix_WPISim.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenixpro_sim_simcancoder_osxuniversal/osx/universal/shared/libCTRE_SimCANCoder.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenixpro_sim_simpigeonimu_osxuniversal/osx/universal/shared/libCTRE_SimPigeonIMU.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenixpro_sim_simtalonfx_osxuniversal/osx/universal/shared/libCTRE_SimTalonFX.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenixpro_sim_simtalonsrx_osxuniversal/osx/universal/shared/libCTRE_SimTalonSRX.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenixpro_sim_simvictorspx_osxuniversal/osx/universal/shared/libCTRE_SimVictorSPX.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_cscore_cscore-cpp_osxuniversal/osx/universal/shared/libcscore.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_cscore_cscore-cpp_osxuniversal/osx/universal/shared/libcscorejni.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_hal_hal-cpp_osxuniversal/osx/universal/shared/libwpiHal.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_hal_hal-cpp_osxuniversal/osx/universal/shared/libwpiHaljni.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_halsim_halsim_gui_osxuniversal/osx/universal/shared/libhalsim_gui.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_ntcore_ntcore-cpp_osxuniversal/osx/universal/shared/libntcore.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_ntcore_ntcore-cpp_osxuniversal/osx/universal/shared/libntcorejni.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpilibc_wpilibc-cpp_osxuniversal/osx/universal/shared/libwpilibc.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpilibnewcommands_wpilibnewcommands-cpp_osxuniversal/osx/universal/shared/libwpilibNewCommands.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpimath_wpimath-cpp_osxuniversal/osx/universal/shared/libwpimathjni.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpimath_wpimath-cpp_osxuniversal/osx/universal/shared/libwpimath.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpinet_wpinet-cpp_osxuniversal/osx/universal/shared/libwpinet.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpinet_wpinet-cpp_osxuniversal/osx/universal/shared/libwpinetjni.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpiutil_wpiutil-cpp_osxuniversal/osx/universal/shared/libwpiutil.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpiutil_wpiutil-cpp_osxuniversal/osx/universal/shared/libwpiutiljni.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenixpro_sim_tools-sim_osxuniversal/osx/universal/shared/libCTRE_PhoenixTools_Sim.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenixpro_sim_simprocancoder_osxuniversal/osx/universal/shared/libCTRE_SimProCANcoder.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenixpro_sim_simpropigeon2_osxuniversal/osx/universal/shared/libCTRE_SimProPigeon2.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenixpro_sim_simprotalonfx_osxuniversal/osx/universal/shared/libCTRE_SimProTalonFX.dylib $INSTALL_PATH