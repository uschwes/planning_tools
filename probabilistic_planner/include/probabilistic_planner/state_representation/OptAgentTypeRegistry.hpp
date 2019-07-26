/*
 * OptAgentTypeRegistry.hpp
 *
 *  Created on: Jul 27, 2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_OPTAGENTTYPEREGISTRY_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_OPTAGENTTYPEREGISTRY_HPP_

#include <iostream>
#include <string>
#include <map>

namespace prob_planner {

enum OptAgentType { UNKNOWN, PEDESTRIAN, CAR, ROBOT, ALL};

class OptAgentTypeRegistry {

 public:
  OptAgentTypeRegistry() {

    _typeMap.clear();
    _stringMap.clear();

    _typeMap.insert(std::make_pair("UNKNOWN", OptAgentType::UNKNOWN));
    _typeMap.insert(std::make_pair("PEDESTRIAN", OptAgentType::PEDESTRIAN));
    _typeMap.insert(std::make_pair("CAR", OptAgentType::CAR));
    _typeMap.insert(std::make_pair("ROBOT", OptAgentType::ROBOT));
    _typeMap.insert(std::make_pair("ALL", OptAgentType::ALL));

    _stringMap.insert(std::make_pair(OptAgentType::UNKNOWN, "UNKNOWN"));
    _stringMap.insert(std::make_pair(OptAgentType::PEDESTRIAN, "PEDESTRIAN"));
    _stringMap.insert(std::make_pair(OptAgentType::CAR, "CAR"));
    _stringMap.insert(std::make_pair(OptAgentType::ROBOT, "ROBOT"));
    _stringMap.insert(std::make_pair(OptAgentType::ALL, "ALL"));

  }
  ~OptAgentTypeRegistry() { }

  OptAgentType fromString(const std::string& stringType);
  std::string& toString(const OptAgentType type);
  static OptAgentTypeRegistry& getRegistry();

 private:
  std::map<std::string, OptAgentType> _typeMap;
  std::map<OptAgentType, std::string> _stringMap;
};

inline std::ostream& operator<<(std::ostream& os, const OptAgentType& type) {
  return os << OptAgentTypeRegistry::getRegistry().toString(type);
}

inline std::istream& operator>>(std::istream& is, OptAgentType& type) {
  std::string stringType;
  is >> stringType;
  type = OptAgentTypeRegistry::getRegistry().fromString(stringType);
  return is;
}

} /* namespace prob_planner */


#endif /* INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_OPTAGENTTYPEREGISTRY_HPP_ */
