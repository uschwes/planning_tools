/*
 * TestAgent.hpp
 *
 *  Created on: Sep 30, 2014
 *      Author: sculrich
 */

#ifndef TESTAGENT_HPP_
#define TESTAGENT_HPP_

#include <planner_interfaces/Agent.hpp>

namespace planning2d {

  class TestAgent : public Agent {

   public:
    struct StateStamped : public virtual planning2d::State, public ::planning2d::StateStamped {
      StateStamped() : ::planning2d::State(1UL), ::planning2d::StateStamped() { }
      StateStamped(const ::planning2d::StateStamped& state) :
        ::planning2d::State(state), ::planning2d::StateStamped(state.stamp()) {

      }
      virtual ~StateStamped() { }
      inline double linearVelocity() const { return this->operator()(0); }
      inline double& linearVelocity() { return this->operator()(0); }

      //! serialization method
      template<class Archive>
      inline void serialize(Archive & ar, const unsigned int /*version*/) {
        boost::serialization::void_cast_register<TestAgent, Agent>();
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(State);
        ar & boost::serialization::base_object<::planning2d::StateStamped>(*this);
      }

    };

    struct SystemInput : public ::planning2d::SystemInput {
      SystemInput() : ::planning2d::SystemInput(1UL) { }
      SystemInput(const ::planning2d::SystemInput& input) : ::planning2d::SystemInput(input) { }
      virtual ~SystemInput() { }
      inline const double& linearVelocity() const { return this->operator()(0); }
      inline double& linearVelocity() { return this->operator()(0); }
    };

   public:
    TestAgent();
    TestAgent(Id id);
    ~TestAgent();


    inline virtual const ::planning2d::StateStamped& stateStamped() const { return _currentState; }
    inline virtual ::planning2d::StateStamped& stateStamped() { return _currentState; }

    // system model implementation
    virtual void applyInput(const planning2d::SystemInput& input, Duration dt);

    // Controller implementations
    virtual ::planning2d::SystemInput computeControlOutput(const ::planning2d::StateStamped& referenceState) const;
    virtual ::planning2d::SystemInput computeControlOutput(const ::planning2d::Pose2d& referencePose) const;
    virtual ::planning2d::SystemInput computeControlOutput(const ::planning2d::StateTrajectory& referenceTrajectory) const;
    virtual ::planning2d::SystemInput computeControlOutput(const ::planning2d::HolonomicVelocity& referenceSpeed) const;

    // collision query
    virtual CollisionObjectPtr getWorkspaceTimeCollisionObject() const;
    virtual CollisionGeometryConstPtr getCollisionGeometry() const;

    //! Return approximation of the agent's 2D shape as a collection of discs
    virtual const DiscApproximation& getDiscApproximation(std::size_t nMaxCircles) const;

    //! deep copy
    virtual Agent::Ptr clone() const;

    //! serialization method
    template<class Archive>
    inline void serialize(Archive & ar, const unsigned int /*version*/) {
      ar & boost::serialization::base_object<Agent>(*this);
      ar & _currentState;
    }

   private:
    TestAgent::StateStamped _currentState;
    DiscApproximation _diskApproximation;
    CollisionObjectPtr _collisionObject;
  };

}

#endif /* TESTAGENT_HPP_ */
