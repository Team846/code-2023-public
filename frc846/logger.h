#ifndef FRC846_LOGGER_H_
#define FRC846_LOGGER_H_

#include <networktables/NetworkTableInstance.h>
#include <units/base.h>

#include <type_traits>
#include <variant>

#include "pref.h"
#include "grapher.h"
#include "frc846/named.h"
#include <frc/Timer.h>
#include "frc/DataLogManager.h"
#include <fmt/core.h>

#include <initializer_list>
#include <iostream>
#include <fstream>
#include <filesystem>

namespace frc846 {

  class Logger {
    private:
      std::string caller;
      bool w_record;
      bool w_graph;

      std::string_view last_log;
      double last_timestamp;

      std::shared_ptr<nt::NetworkTable> table_;

    public:
      Logger(std::string t_caller) {
        Logger(t_caller, false, false);
      }

      Logger(std::string t_caller, bool t_graph) {
        Logger(t_caller, t_graph, false);
      }

      Logger(std::string t_caller, bool t_graph, bool t_record) {
        caller = t_caller;
        w_record = t_record;
        w_graph = t_graph;

        table_ = nt::NetworkTableInstance::GetDefault().GetTable(fmt::format("[{}]", t_caller));
      }

      int sendLog(std::string msg, auto log) {
        last_log = fmt::format("{}", log);
        last_timestamp = frc::Timer::GetFPGATimestamp().to<double>();

        auto t_log = fmt::format("Logger [{}] --> {} & {}", caller, msg, log);
        frc::DataLogManager::Log(t_log);

        if (w_graph) table_ -> PutString(fmt::format("{}", msg), fmt::format("{}", log));

        auto filename = "/home/lvuser/"+caller+".log";

        if (w_record) {
          if (!std::filesystem::exists(filename) || std::filesystem::file_size(filename) > 1000000)  {

            std::ofstream t_file(filename,  std::fstream::in | std::fstream::out | std::fstream::trunc);
            t_file << "Logger Overwrite " << fmt::format("{}", last_timestamp) << "\n";
            t_file << "Logger [" << caller << "]" << fmt::format("{}", last_timestamp) << "--> " << msg << " & " << fmt::format("{}", log) << "\n";
            t_file.close();

          } else {
            std::ofstream t_file(filename, std::fstream::in | std::fstream::out | std::fstream::app); 
             t_file << "Logger [" << caller << "] " << fmt::format("{}", last_timestamp) << " --> " << msg << " & " << fmt::format("{}", log) << "\n";
            t_file.close();
          }
        }

        return 0;
      }

      int sendLog(auto log) {
        last_log = fmt::format("{}", log);
        last_timestamp = frc::Timer::GetFPGATimestamp().to<double>();

        auto t_log = fmt::format("Logger [{}] --> {}", caller, log);
        frc::DataLogManager::Log(t_log);

        auto filename = "/home/lvuser/"+caller+".log";

        if (w_record) {
          if (!std::filesystem::exists(filename) || std::filesystem::file_size(filename) > 1000000)  {

            std::ofstream t_file(filename,  std::fstream::in | std::fstream::out | std::fstream::trunc);
            t_file << "Logger Overwrite " << fmt::format("{}", last_timestamp) << "\n";
            t_file << "Logger [" << caller << "]" << fmt::format("{}", last_timestamp) << "--> " << fmt::format("{}", log) << "\n";
            t_file.close();

          } else {
            std::ofstream t_file(filename, std::fstream::in | std::fstream::out | std::fstream::app); 
            t_file << "Logger [" << caller << "] " << fmt::format("{}", last_timestamp) << " --> " << fmt::format("{}", log) << "\n";
            t_file.close();
          }
        }

        return 0;
      }

      std::string_view retrieveLastLog() {
        return last_log;
      }

      double timeOfLastLog() {
        return last_timestamp;
      }
  };

}  // namespace frc846

#endif  // FRC846_LOGGER_H_