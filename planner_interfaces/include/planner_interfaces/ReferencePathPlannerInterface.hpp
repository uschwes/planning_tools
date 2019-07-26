#ifndef REFERENCE_PATH_PLANNER_INTERFACE_HPP_
#define REFERENCE_PATH_PLANNER_INTERFACE_HPP_

// local includes
#include "PlannerInterface.hpp"

namespace planning2d {
    
  class ReferencePathPlannerInterface : public virtual PlannerInterface {

    public:
      PLANNING_2D_POINTER_TYPEDEFS(ReferencePathPlannerInterface);
      
    public:
      //! Destructor
      virtual ~ReferencePathPlannerInterface(){}
      
      /**
       * Call this method upon a new reference path for the ego agent
       * @param[in] path Reference path
       * @return Return code
       */
      virtual ReturnCode callbackReferencePath(const Path& path) = 0;
  }; /* class ReferencePathPlannerInterface */

} /* namespace planning2d */

#endif /* REFERENCE_PATH_PLANNER_INTERFACE_HPP_ */
