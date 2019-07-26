#ifndef TARGET_PLANNER_INTERFACE_HPP_
#define TARGET_PLANNER_INTERFACE_HPP_

// local includes
#include "PlannerInterface.hpp"

namespace planning2d {
    
  class TargetPlannerInterface : public virtual PlannerInterface {

    public:
      PLANNING_2D_POINTER_TYPEDEFS(TargetPlannerInterface);
      
    public:
      //! Destructor
      virtual ~TargetPlannerInterface(){}

      /**
       * Call this method upon a new target for the ego agent
       * @param[in] target Target pose
       * @param[in] isFinal the planner should be able to stop at this target
       * @return Return code
       */
      virtual ReturnCode callbackSetTarget(const Pose2d& target, bool isFinal) = 0;
  }; /* class TargetPlannerInterface */

} /* namespace planning2d */

#endif /* TARGET_PLANNER_INTERFACE_HPP_ */
