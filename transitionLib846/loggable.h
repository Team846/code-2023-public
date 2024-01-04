#ifndef transitionLib846_LOGGABLE_H_
#define transitionLib846_LOGGABLE_H_

#include "frc/DataLogManager.h"
#include <initializer_list>
#include <iostream>
#include <fstream>
#include <filesystem>

#include <networktables/NetworkTableInstance.h>
#include <units/base.h>

#include <type_traits>
#include <variant>

namespace transitionLib846 {
    class Loggable {
        public:
        Loggable(Loggable& parent_, std::string name) {
            Loggable(parent_.name() + "/" + name);
        }

        Loggable(std::string name) { name_ = name; }

        std::string name() { return name_; }

        private:
            std::string name_;
    };
}

#endif