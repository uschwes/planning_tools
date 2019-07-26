#ifndef PLANNER_INTERFACE_HPP_
#define PLANNER_INTERFACE_HPP_

// standard includes
#include <vector>

// boost includes
#include <boost/shared_ptr.hpp>

// Schweizer Messer includes
#include <sm/assert_macros.hpp>

// local includes
#include "Support.hpp"
#include "ReturnCodes.hpp"
#include "Agent.hpp"
#include "Frame.hpp"
#include "OccupancyGrid.hpp"

namespace planning2d {
    
  class PlannerInterface {

    public:
      PLANNING_2D_POINTER_TYPEDEFS(PlannerInterface);
      
    public:
      //! Default constructor
      PlannerInterface();
      //! Destructor
      virtual ~PlannerInterface();
      
      //! Resets the planner to original state
      virtual void reset() { }

      //! Sets the frame into which all data should be transformed
      inline void setPlanningFrame(const Frame& frame) { _planningFrame = frame; }

      /**
       * Computes a plan to be executed by the ego agent
       * @param[in] currentTime Current time
       * @param[out] stateInputSequency Sequence of timestamped state input pairs. The inputs can be forwarded to the platform as
       * feedforward commands. The states can be used to have a feedback control law.
       * @return Return code
       */
      virtual ReturnCode computePlan(const Time& currentTime, StateInputTrajectory& stateInputSequence) = 0;

      /**
       * Call this method upon a new occupancy grid
       * @param[in] grid Occupancy grid
       * @return Return code
       */
      virtual ReturnCode callbackOccupancyGrid(const OccupancyGridStamped& grid) = 0;

      /**
       * Call this method upon a new state measurement of the ego agent
       * @param[in] stateStamped Timestamped state of the ego agent
       * @return Return code
       */
      virtual ReturnCode callbackCurrentState(const planning2d::StateStamped & stateStamped) = 0;

      /**
       * Call this method upon a new set of dynamic objects measurements
       * @param[in] dynamicObjects Set of dynamic objects
       * @return Return code
       */
      virtual ReturnCode callbackDynamicObjects(const std::vector<Agent::ConstPtr>& dynamicObjects) = 0;

      //! serialization method
      template<class Archive>
      inline void serialize(Archive & ar, const unsigned int /*version*/) { ar & _planningFrame; }

    private:
      PLANNING_2D_DISALLOW_COPY_AND_ASSIGN(PlannerInterface);

      std::string _planningFrame; //! The frame into which all data should be transformed

  }; /* class PlannerInterface */

} /* namespace planning2d */

#endif /* PLANNER_INTERFACE_HPP_ */
