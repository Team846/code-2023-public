#ifndef transitionLib846_COMMONS_H_
#define transitionLib846_COMMONS_H_

#include <exception>
#include <map>
#include <string>

namespace transitionLib846 {

class Commons {
 public:
    static std::map<std::string, double> fmap;

    static double GetVal(std::string key) {
        try {
            return fmap[key];
        } catch (std::exception err) {
            return 0.0;
        }
    }

    static void SetVal(std::string key, double val) {
        fmap[key] = val;
    }
};

}  // namespace transitionLib846

#endif  // transitionLib846_COMMONS_H_