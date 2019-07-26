#include <planner_interfaces/PlannerInterface.hpp>
#include <planner_interfaces/Exceptions.hpp>

namespace planning2d {

  PlannerInterface::PlannerInterface() {
    static_assert(sizeof(OccupancyValue) == 1, "OccupancyValue should have size of one byte!");
  }

  PlannerInterface::~PlannerInterface() {
      
  }
  
}
