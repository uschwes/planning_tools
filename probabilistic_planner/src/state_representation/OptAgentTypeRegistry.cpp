/*
 * OptAgentTypeRegistry.cpp
 *
 *  Created on: Jul 27, 2015
 *      Author: pfmark
 */

#include <probabilistic_planner/state_representation/OptAgentTypeRegistry.hpp>

namespace prob_planner {


OptAgentType OptAgentTypeRegistry::fromString(const std::string& stringType) {
  try {
    return _typeMap.at(stringType);
  } catch(std::out_of_range & e){
    throw std::out_of_range(std::string("No OptAgentType registered for ") + stringType);
  }
}

std::string& OptAgentTypeRegistry::toString(const OptAgentType type) {
  return _stringMap.at(type);
}

OptAgentTypeRegistry& OptAgentTypeRegistry::getRegistry() {
  static OptAgentTypeRegistry registry;
  return registry;
}

} /* namespace prob_planner */
