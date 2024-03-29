#ifndef transitionLib846_NAMED_H_
#define transitionLib846_NAMED_H_

#include <fmt/core.h>

#include <initializer_list>
#include <iostream>

#include "frc/DataLogManager.h"

namespace transitionLib846 {

// An object with a name that is used for logging, configs, etc.
//
// Names should be in snake_case.
class Named {
 public:
  // Construct a named object.
  explicit Named(std::string_view name);

  // Construct a named object under a parent object.
  Named(const Named& parent, std::string_view name);

  virtual ~Named() = default;

  // Get the object's name.
  const std::string& name() const { return name_; }

  // Total number of warnings logged.
  static unsigned int warn_count() { return warn_count_; }

  // Total number of errors logged.
  static unsigned int error_count() { return error_count_; }

  // Join together multiple names to create a full path separated by '/'.
  static std::string Join(std::initializer_list<std::string_view> elements);

  // Log a debug message to standard output.
  template <typename... T>
  void Debug(fmt::format_string<T...> fmt, T&&... args) const {
    FormatLog("DEBUG", std::forward<fmt::format_string<T...>>(fmt),
              std::forward<T>(args)...);
  }

  // Log a warning message to standard output.
  template <typename... T>
  void Warn(fmt::format_string<T...> fmt, T&&... args) const {
    ++warn_count_;
    FormatLog("WARN ", std::forward<fmt::format_string<T...>>(fmt),
              std::forward<T>(args)...);
    // frc::DriverStation::GetInstance().ReportWarning(msg);
  }

  // Log an error message to standard output.
  template <typename... T>
  void Error(fmt::format_string<T...> fmt, T&&... args) const {
    ++error_count_;
    FormatLog("ERROR", std::forward<fmt::format_string<T...>>(fmt),
              std::forward<T>(args)...);
    // frc::DriverStation::GetInstance().ReportError(msg);
  }

 private:
  std::string name_;

  static unsigned int warn_count_;
  static unsigned int error_count_;

  // Format a log message.
  //
  // e.g. "DEBUG [subsystem/name] ..."
  template <typename... T>
  void FormatLog(std::string_view level, fmt::format_string<T...> fmt,
                 T&&... args) const {
    auto msg = fmt::format(std::forward<fmt::format_string<T...>>(fmt),
                           std::forward<T>(args)...);

    auto final_message = fmt::format("{} [{}] {}", level, name(), msg);
    frc::DataLogManager::Log(final_message);
  }
};

}  // namespace transitionLib846

#endif  // transitionLib846_NAMED_H_