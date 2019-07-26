#include <gtest/gtest.h>

#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <sm/BoostPropertyTree.hpp>
#include <sm/logging.hpp>
#include <sm/random.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <sm/eigen/gtest.hpp>

#include <planner_interfaces/MathSupport.hpp>
#include <planner_interfaces/Pose2d.hpp>
#include <planner_interfaces/Time.hpp>
#include <planner_interfaces/OccupancyGrid.hpp>
#include <planner_interfaces/Agent.hpp>
#include <planner_interfaces/TestAgent.hpp>

#include "TestPlanner.hpp"

using namespace std;
using namespace planning2d;

/**
 * Saves serializable data structure to string
 * @return
 */
template <class T>
string serialize(const T& data) {
  ostringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << data;
  return ss.str();
}

template <class T>
T deserialize(const std::string& str) {
    stringstream s;
    s << str;

    boost::archive::binary_iarchive ia(s);
    T obj;
    ia >> obj;

    return obj;
}

template <int METHOD>
struct CubicInterpolateFunctorX {
  typedef Eigen::Matrix<double, 1, 1> Vector1d;
  typedef Vector1d input_t;
  typedef Vector1d value_t;
  typedef double scalar_t;
  typedef Vector1d jacobian_t;
  CubicInterpolateFunctorX(const Eigen::Vector4d& points) : _points(points) { }
  input_t update(const input_t& x0, unsigned /*c*/, scalar_t delta) {
    return x0 + input_t::Constant(delta);
  }
  value_t operator()(const input_t & dr) {
    return value_t::Constant(math::Interpolator1d<METHOD>::interpolate(_points, (scalar_t)dr[0]));
  }
  Eigen::Vector4d _points;
};

template <int METHOD>
struct CubicInterpolateFunctorP {
  typedef Eigen::Matrix<double, 1, 1> Vector1d;
  typedef Eigen::Vector4d input_t;
  typedef Vector1d value_t;
  typedef double scalar_t;
  typedef Eigen::RowVector4d jacobian_t;
  CubicInterpolateFunctorP(const scalar_t x) : _x(x) { }
  input_t update(const input_t& x0, unsigned c, scalar_t delta) {
    input_t ret = x0;
    ret[c] += delta;
    return ret;
  }
  value_t operator()(const input_t & dr) {
    return value_t::Constant(math::Interpolator1d<METHOD>::interpolate(dr, _x));
  }
  double _x;
};

template <int METHOD>
struct BicubicInterpolateFunctor {
  typedef Eigen::Matrix<double, 4, 4> Points;
  typedef Eigen::Matrix<double, 1, 1> Vector1d;
  typedef Eigen::Vector2d input_t;
  typedef Vector1d value_t;
  typedef double scalar_t;
  typedef Eigen::RowVector2d jacobian_t;
  BicubicInterpolateFunctor(const Points& points) : _points(points) { }
  input_t update(const input_t& x0, unsigned c, scalar_t delta) {
    input_t xout = x0;
    xout[c] += delta;
    return xout;
  }
  value_t operator()(const input_t & dr) {
    return value_t::Constant(1, math::Interpolator2d<METHOD>::interpolate(_points, (scalar_t)dr[0], (scalar_t)dr[1]));
  }
  Points _points;
};

TEST(planner_interfaces_TESTSUITE, MathSupport) {

  typedef math::Interpolator1d<math::InterpolationMethod::CUBIC_CATMULL_ROM> I1dCR;
  typedef math::Interpolator1d<math::InterpolationMethod::CUBIC_PCHIP> I1dPCHIP;
  typedef math::Interpolator2d<math::InterpolationMethod::CUBIC_CATMULL_ROM> I2dCR;
  typedef math::Interpolator2d<math::InterpolationMethod::CUBIC_PCHIP> I2dPCHIP;

  // Test cubic interpolation
  {
    Eigen::Vector4d points;
    points << 0.0, 1.0, 2.0, 3.0;

    // First with a linearly increasing function
    EXPECT_DOUBLE_EQ(1.0, I1dCR::interpolate(points, 0.0));
    EXPECT_DOUBLE_EQ(1.5, I1dCR::interpolate(points, 0.5));
    EXPECT_DOUBLE_EQ(2.0, I1dCR::interpolate(points, 1.0));
    EXPECT_DOUBLE_EQ(1.0, I1dCR::gradient(points, 0.0));
    EXPECT_DOUBLE_EQ(1.0, I1dCR::gradient(points, 0.5));
    EXPECT_DOUBLE_EQ(1.0, I1dCR::gradient(points, 1.0));

    Eigen::Vector4d coeff;
    math::cubicInterpolatePchipCoefficients(points, coeff);
    sm::eigen::assertEqual((Eigen::Vector4d() << 1.0, 1.0, 0.0, 0.0).finished(), coeff, SM_SOURCE_FILE_POS);
    EXPECT_DOUBLE_EQ(1.0, I1dPCHIP::interpolate(points, 0.0));
    EXPECT_DOUBLE_EQ(1.5, I1dPCHIP::interpolate(points, 0.5));
    EXPECT_DOUBLE_EQ(2.0, I1dPCHIP::interpolate(points, 1.0));
    EXPECT_DOUBLE_EQ(1.0, I1dPCHIP::gradient(points, 0.0));
    EXPECT_DOUBLE_EQ(1.0, I1dPCHIP::gradient(points, 0.5));
    EXPECT_DOUBLE_EQ(1.0, I1dPCHIP::gradient(points, 1.0));

    // Now with something more complicated
    points << 2.0, 1.0, 2.0, 3.0;
    EXPECT_DOUBLE_EQ(1.0, I1dCR::interpolate(points, 0.0));
    EXPECT_DOUBLE_EQ(1.375, I1dCR::interpolate(points, 0.5));
    EXPECT_DOUBLE_EQ(2.0, I1dCR::interpolate(points, 1.0));

    Eigen::Vector4d coeffTrue = (Eigen::Vector4d() << 1.0, 0.0, 2.0, -1.0).finished();
    math::cubicInterpolatePchipCoefficients(points, coeff);
    sm::eigen::assertEqual(coeffTrue, coeff, SM_SOURCE_FILE_POS);
    EXPECT_DOUBLE_EQ(1.0, I1dPCHIP::interpolate(points, 0.0));
    EXPECT_DOUBLE_EQ(1.375, I1dPCHIP::interpolate(points, 0.5));
    EXPECT_DOUBLE_EQ(2.0, I1dPCHIP::interpolate(points, 1.0));

    // test gradients
    EXPECT_DOUBLE_EQ((points[2] - points[0])/2., I1dCR::gradient(points, 0.0));
    EXPECT_DOUBLE_EQ((points[3] - points[1])/2., I1dCR::gradient(points, 1.0));

    EXPECT_DOUBLE_EQ(0.0, I1dPCHIP::gradient(points, 0.0));
    EXPECT_DOUBLE_EQ(2.*(points[3] - points[2])*(points[2] - points[1])/(points[3] - points[1]), I1dPCHIP::gradient(points, 1.0));
    EXPECT_DOUBLE_EQ(coeffTrue.tail(3).transpose()*(Eigen::Vector3d() << 1., 2.*0.5, 3.*math::square(0.5)).finished(), I1dPCHIP::gradient(points, 0.5));
    sm::eigen::assertNear((Eigen::Vector4d() << 0., 1., 0., 0.).finished(), math::gradientCubicInterpolatePchipP(points, 0.0), 1e-5, SM_SOURCE_FILE_POS);
    sm::eigen::assertNear((Eigen::Vector4d() << 0., 0., 1., 0.).finished(), math::gradientCubicInterpolatePchipP(points, 1.0), 1e-5, SM_SOURCE_FILE_POS);

    // Test gradients w.r.t. x numerically
    {
      typedef CubicInterpolateFunctorX<math::InterpolationMethod::CUBIC_CATMULL_ROM> Functor;
      Functor functorX(points);
      sm::eigen::NumericalDiff<Functor> numdiffX(functorX);
      for (double x=1e-6; x<=1.0-1e-6; x+=0.011) {
        const auto gradEst = numdiffX.estimateJacobian(Functor::input_t::Constant(x));
        const auto gradComp = Functor::jacobian_t::Constant(1, 1, I1dCR::gradient(points, x)).eval();
        SCOPED_TRACE(testing::Message() << "Testing Catmull-Rom 1D interpolation at x = " << x);
        sm::eigen::assertNear(gradEst, gradComp, 1e-5, SM_SOURCE_FILE_POS);
      }
    }
    { // TODO: This code duplication is a bit ugly
      typedef CubicInterpolateFunctorX<math::InterpolationMethod::CUBIC_PCHIP> Functor;
      Functor functorX(points);
      sm::eigen::NumericalDiff<Functor> numdiffX(functorX);
      for (double x=1e-6; x<=1.0-1e-6; x+=0.011) {
        const auto gradEst = numdiffX.estimateJacobian(Functor::input_t::Constant(x));
        const auto gradComp = Functor::jacobian_t::Constant(1, 1, I1dCR::gradient(points, x)).eval();
        SCOPED_TRACE(testing::Message() << "Testing PCHIP 1D interpolation at x = " << x);
        sm::eigen::assertNear(gradEst, gradComp, 1e-5, SM_SOURCE_FILE_POS);
      }
    }

    // Test gradients w.r.t. points numerically
    {
      typedef CubicInterpolateFunctorP<math::InterpolationMethod::CUBIC_CATMULL_ROM> Functor;
      Functor functorP(0.234);
      sm::eigen::NumericalDiff<Functor> numdiffP(functorP);
      for (size_t i=0; i<100; i++) {
        const auto p = Functor::input_t::Random().eval();
        const auto gradEst = numdiffP.estimateJacobian(p).transpose().eval();
        const auto gradComp = math::gradientCubicInterpolateP(0.234);
        SCOPED_TRACE(testing::Message() << "Testing Catmull-Rom interpolation at x = " << p.transpose());
        sm::eigen::assertNear(gradEst, gradComp, 1e-5, SM_SOURCE_FILE_POS);
      }
    }
    { // TODO: This code duplication is a bit ugly
      typedef CubicInterpolateFunctorP<math::InterpolationMethod::CUBIC_PCHIP> Functor;
      Functor functorP(0.234);
      sm::eigen::NumericalDiff<Functor> numdiffP(functorP);
      for (size_t i=0; i<100; i++) {
        const auto p = Functor::input_t::Random().eval();
        const auto gradEst = numdiffP.estimateJacobian(p).transpose().eval();
        const auto gradComp = math::gradientCubicInterpolatePchipP(p, 0.234);
        SCOPED_TRACE(testing::Message() << "Testing PCHIP interpolation wrt. points at x = " << p.transpose());
        sm::eigen::assertNear(gradEst, gradComp, 1e-5, SM_SOURCE_FILE_POS);
      }
    }
  }

  // Test bicubic interpolation
  {
    // test gradients
    typedef BicubicInterpolateFunctor<math::InterpolationMethod::CUBIC_CATMULL_ROM> Functor;
    Functor::Points points = Functor::Points::Random();
    Functor functor(points);
    sm::eigen::NumericalDiff<Functor> numdiff(functor);
    for (size_t i=0; i<100; i++) {
      const auto p = Functor::input_t::NullaryExpr(2, 1, [&] (int) { return sm::random::randLU(0.001, 0.999); }).eval();
      const auto gradEst = numdiff.estimateJacobian(p);
      const auto gradComp = I2dCR::gradient(points, p.x(), p.y()).transpose().eval();
      SCOPED_TRACE(testing::Message() << "Testing Catmull-Rom 2D interpolation at x = " << p.transpose());
      sm::eigen::assertNear(gradEst, gradComp, 1e-5, SM_SOURCE_FILE_POS);
    }
  }
  { // TODO: This code duplication is a bit ugly
    // test gradients
    typedef BicubicInterpolateFunctor<math::InterpolationMethod::CUBIC_PCHIP> Functor;
    Functor::Points points = Functor::Points::Random();
    Functor functor(points);
    sm::eigen::NumericalDiff<Functor> numdiff(functor);
    for (size_t i=0; i<100; i++) {
      const auto p = Functor::input_t::NullaryExpr(2, 1, [&] (int) { return sm::random::randLU(0.001, 0.999); }).eval();
      const auto gradEst = numdiff.estimateJacobian(p);
      const auto gradComp = I2dPCHIP::gradient(points, p.x(), p.y()).transpose().eval();
      SCOPED_TRACE(testing::Message() << "Testing PCHIP 2D interpolation at x = " << p.transpose());
      sm::eigen::assertNear(gradEst, gradComp, 1e-5, SM_SOURCE_FILE_POS);
    }
  }

  // Test linspace
  {
    // Test floating point
    {
      std::vector<double> vals;
      const std::size_t numVals = 10;
      math::linspace(0.0, 1.0, numVals, vals, true);
      ASSERT_EQ(numVals, vals.size());
      EXPECT_EQ(0.0, vals.front());
      EXPECT_EQ(1.0, vals.back());
      for (std::size_t i=0; i<vals.size(); i++)
        EXPECT_DOUBLE_EQ(i*1.0/(numVals - 1), vals[i]);
    }
    // Test integer
    {
      std::vector<int> vals;
      const std::size_t numVals = 10;
      math::linspace(0, 10, numVals, vals, true);
      ASSERT_EQ(numVals, vals.size());
      EXPECT_EQ(0, vals.front());
      EXPECT_EQ(10, vals.back());
      int maxDiff = std::numeric_limits<int>::min();
      int minDiff = std::numeric_limits<int>::max();
      for (std::size_t i=1; i<vals.size(); i++) {
        int diff = vals[i] - vals[i-1];
        maxDiff = std::max(diff, maxDiff);
        minDiff = std::min(diff, minDiff);
      }
      EXPECT_EQ(2, maxDiff);
      EXPECT_EQ(1, minDiff);
    }
    // Test Time
    {
      std::vector<Time> vals;
      const std::size_t numVals = 10;
      math::linspace(Time(1.0), Time(10.0), numVals, vals, true);
      ASSERT_EQ(numVals, vals.size());
      for (std::size_t i=0; i<vals.size(); i++)
        EXPECT_EQ(Time(static_cast<double>(i)+1.0), vals[i]);
    }
  }

}

TEST(planner_interfaces_TESTSUITE, Position2d) {

  try {

#ifndef NDEBUG
    // test that in debug mode we get exceptions if do not initialize properly
    {
      Position2d p;
      EXPECT_THROW(p == p, InitializationException);
      EXPECT_THROW(p != p, InitializationException);
      EXPECT_THROW(p + p, InitializationException);
      EXPECT_THROW(p.norm(), InitializationException);
      EXPECT_NO_THROW(p.x() = 3.0);
    }
#endif

    {
      Position2d p(0.5, 1.0);

      std::ostringstream stream;
      stream << p;
      EXPECT_EQ("(0.5, 1)", stream.str());

      EXPECT_DOUBLE_EQ(0.5, p.x());
      EXPECT_DOUBLE_EQ(1.0, p.y());

      p.setZero();
      EXPECT_DOUBLE_EQ(0.0, p.x());
      EXPECT_DOUBLE_EQ(0.0, p.y());

      p.setConstant(1.0);
      EXPECT_DOUBLE_EQ(1.0, p.x());
      EXPECT_DOUBLE_EQ(1.0, p.y());

      p = Position2d::Zero();
      EXPECT_DOUBLE_EQ(0.0, p.x());
      EXPECT_DOUBLE_EQ(0.0, p.y());

      p = Position2d::Constant(2.0);
      EXPECT_DOUBLE_EQ(2.0, p.x());
      EXPECT_DOUBLE_EQ(2.0, p.y());
    }

#ifndef NDEBUG
    {
      Position2d p;
      EXPECT_TRUE(std::isnan(p.x()));
      EXPECT_TRUE(std::isnan(p.y()));
    }
#endif

    {
      Position2d p(2.0, 2.0);
      EXPECT_NEAR(sqrt(8.0), p.norm(), 1e-6);
      EXPECT_NEAR(8.0, p.squaredNorm(), 1e-6);
      EXPECT_NEAR(4.0, p.sum(), 1e-6);
    }

    {
      Position2d p(2.0, 2.0);

      p-=Position2d(2.0,2.0);
      EXPECT_DOUBLE_EQ(0.0, p.x());
      EXPECT_DOUBLE_EQ(0.0, p.y());

      p+=Position2d(1.0,1.0);
      EXPECT_DOUBLE_EQ(1.0, p.x());
      EXPECT_DOUBLE_EQ(1.0, p.y());
    }

    {
      Position2d p(-0.1, 1.6);
      p.floor();
      EXPECT_DOUBLE_EQ(-1.0, p.x());
      EXPECT_DOUBLE_EQ(+1.0, p.y());

      p = Position2d(-0.1, 1.6);
      p.ceil();
      EXPECT_DOUBLE_EQ(+0.0, p.x());
      EXPECT_DOUBLE_EQ(+2.0, p.y());
    }

    {
      vector<Position2dStamped> positions;
      positions.push_back(Position2dStamped(2.0, 3.0, Time(1.0)));
      positions.push_back(Position2dStamped(Position2d(2.0, 3.0), Time(1.0)));
      positions.push_back(Position2dStamped(Eigen::Vector2d(2.0, 3.0), Time(1.0)));

      for (auto& p: positions) {
        EXPECT_DOUBLE_EQ(2.0, p.x());
        EXPECT_DOUBLE_EQ(3.0, p.y());
        EXPECT_EQ(Time(1.0), p.stamp());
      }
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(planner_interfaces_TESTSUITE, Pose2d) {

  try {

#ifndef NDEBUG
    // test that in debug mode we get exceptions if do not initialize properly
    {
      Pose2d p;
      EXPECT_THROW(p == p, InitializationException);
      EXPECT_THROW(p != p, InitializationException);
      EXPECT_THROW(p + p, InitializationException);
      EXPECT_NO_THROW(p.yaw() = 3.0);
    }
#endif

    {
      Pose2d pose(0.5, 1.0, 2.0);

      std::ostringstream stream;
      stream << pose;
      EXPECT_EQ("(0.5, 1, 2)", stream.str());

      EXPECT_DOUBLE_EQ(3, pose.dimension());
      EXPECT_DOUBLE_EQ(0.5, pose.position().x());
      EXPECT_DOUBLE_EQ(1.0, pose.position().y());
      EXPECT_DOUBLE_EQ(2.0, pose.yaw());
    }

    {
      Pose2d pose;
      pose.setVector(Pose2d::Vector(0.5, 1.0, 2.0));
      EXPECT_EQ(Pose2d(0.5, 1.0, 2.0), pose);
    }

#ifndef NDEBUG
    {
      Pose2d pose;
      EXPECT_TRUE(std::isnan(pose.position().x()));
      EXPECT_TRUE(std::isnan(pose.position().y()));
      EXPECT_TRUE(std::isnan(pose.yaw()));
    }
#endif

    {
      Pose2d pose;
      pose.position().x() = 0.5;
      pose.position().y() = 1.0;
      pose.yaw() = 2.0;
      EXPECT_DOUBLE_EQ(0.5, pose.position().x());
      EXPECT_DOUBLE_EQ(1.0, pose.position().y());
      EXPECT_DOUBLE_EQ(2.0, pose.yaw());
    }

    // Test something that produces a wrap around in angular dimension
    {
      Pose2d pose(0.0,0.0,7.0);
      EXPECT_DOUBLE_EQ(0.0, pose.position().x());
      EXPECT_DOUBLE_EQ(0.0, pose.position().y());
      EXPECT_DOUBLE_EQ(7.0, pose.yaw());
      EXPECT_DOUBLE_EQ(7.0-planning2d::TWOPI, pose.normalizeYaw().yaw());

      pose.yaw () = +planning2d::PI;
      EXPECT_DOUBLE_EQ(+planning2d::PI, pose.normalizeYaw().yaw());
      pose.yaw() = -planning2d::PI-1e-6;
      EXPECT_DOUBLE_EQ(+planning2d::PI-1e-6, pose.normalizeYaw().yaw());
    }

    // transform routines
    {
      Position2d positionT;
      Pose2d pose(0.0, 1.0, planning2d::PI);
      Position2d position(1.0, 2.0);
      positionT = pose.transformTo(position);
      EXPECT_NEAR(-1.0, positionT.x(), 1e-6);
      EXPECT_NEAR(-1.0, positionT.y(), 1e-6);
      positionT = pose.transformFrom(position);
      EXPECT_NEAR(-1.0, positionT.x(), 1e-6);
      EXPECT_NEAR(-1.0, positionT.y(), 1e-6);
    }

    // stamped type
    {
      vector<Pose2dStamped> poses;
      poses.push_back(Pose2dStamped(2.0, 3.0, 1.0, Time(1.0)));
      poses.push_back(Pose2dStamped(Position2d(2.0, 3.0), 1.0, Time(1.0)));
      poses.push_back(Pose2dStamped(Pose2d(2.0, 3.0, 1.0), Time(1.0)));
      poses.push_back(Pose2dStamped(Eigen::Vector3d(2.0, 3.0, 1.0), Time(1.0)));

      for (auto& p: poses) {
        EXPECT_DOUBLE_EQ(2.0, p.x());
        EXPECT_DOUBLE_EQ(3.0, p.y());
        EXPECT_DOUBLE_EQ(1.0, p.yaw());
        EXPECT_EQ(Time(1.0), p.stamp());
      }
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(planner_interfaces_TESTSUITE, State) {

  try {

    {
      State s;
      EXPECT_EQ(0, s.dimension());
      EXPECT_EQ(s.pose().dimension(), s.fullDimension());
      EXPECT_EQ(0, s.state().size());
    }

    {
      State s(2);
      EXPECT_EQ(2, s.dimension());
      EXPECT_EQ(s.pose().dimension() + 2, s.fullDimension());
      EXPECT_EQ(2, s.state().size());
#ifndef NDEBUG
      EXPECT_FALSE(s.state().allFinite());
#endif
    }

    {
      Eigen::Vector2d svars(0.1, 0.2);
      State s(svars, Pose2d(0.3, 0.4, 0.5));

      EXPECT_DOUBLE_EQ(0.3, s.pose().position().x());
      EXPECT_DOUBLE_EQ(0.3, s.position().x());
      EXPECT_DOUBLE_EQ(0.3, s.x());

      EXPECT_DOUBLE_EQ(0.4, s.pose().position().y());
      EXPECT_DOUBLE_EQ(0.4, s.position().y());
      EXPECT_DOUBLE_EQ(0.4, s.y());

      EXPECT_DOUBLE_EQ(0.5, s.pose().yaw());
      EXPECT_DOUBLE_EQ(0.5, s.yaw());

      EXPECT_TRUE(s.state().isApprox(svars));
      EXPECT_DOUBLE_EQ(0.1, s(0));
      EXPECT_DOUBLE_EQ(0.2, s(1));

      EXPECT_TRUE(s == s);
    }

    // Test inheritance
    {
      TestAgent::StateStamped ssta;
      ssta.linearVelocity() = 1.0;
      EXPECT_DOUBLE_EQ(1.0, ssta.linearVelocity());
      EXPECT_DOUBLE_EQ(1.0, ssta(0));

      StateStamped& ss = ssta;
      ssta = static_cast<TestAgent::StateStamped>(ss);
      EXPECT_DOUBLE_EQ(1.0, ssta.linearVelocity());
      EXPECT_DOUBLE_EQ(1.0, ssta(0));
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(planner_interfaces_TESTSUITE, Agent) {

  try {

    TestAgent::StateStamped agentState;
    agentState.pose() = Pose2d(0.0, 0.0, 0.0);
    agentState.stamp() = Time(0L);
    agentState.linearVelocity() = 0.1;

    EXPECT_EQ(Time(0L), agentState.stamp());
    EXPECT_EQ(Pose2d(0.0,0.0,0.0), agentState.pose());
    EXPECT_DOUBLE_EQ(0.1, agentState.linearVelocity());

    TestAgent agent(0 /*id*/);
    agent.stateStamped() = agentState;
    agent.setInteractionAware(true);

    planning2d::StateStamped agentStateOut = agent.stateStamped();
    EXPECT_EQ(agentState.stamp(), agentStateOut.stamp());
    EXPECT_EQ(agentState.pose(), agentStateOut.pose());
    EXPECT_EQ(agentState.state(), agentStateOut.state());
    EXPECT_TRUE(agent.interactionAware());

    // test state cast
    TestAgent::StateStamped ss = static_cast<TestAgent::StateStamped>(agentStateOut);
    EXPECT_EQ(agentState, ss);
    EXPECT_DOUBLE_EQ(0.1, ss.linearVelocity());

    TestAgent::SystemInput systemInput = agent.computeControlOutput(agentState.pose());
    ASSERT_EQ(systemInput.dimension(), 1);
    EXPECT_DOUBLE_EQ(0.1, systemInput.linearVelocity());

    agent.applyInput(systemInput, Duration(1.0));
    TestAgent::StateStamped agentStateNew = agent.stateStamped();
    EXPECT_DOUBLE_EQ(0.1, agentStateNew.pose().position().x()); // 0.0 + 1.0*0.1
    EXPECT_DOUBLE_EQ(0.0, agentStateNew.pose().position().y());
    EXPECT_DOUBLE_EQ(1.0, agentStateNew.pose().yaw()); // 0.0 + 1.0
    EXPECT_DOUBLE_EQ(0.1, agentStateNew.linearVelocity());
    EXPECT_EQ(Time(1.0), agentStateNew.stamp());

    // reset agent
    agent.stateStamped() = agentState;

    // create another agent to test the collision query functions
    TestAgent otherAgent(1 /*id*/);
    agentState.pose() = Pose2d(1.0,1.0,0.0);
    agentState.stamp() = Time(0L);
    agentState.linearVelocity() = 0.1;
    otherAgent.stateStamped() = agentState;

    DiscApproximation discApproxAgent = agent.getDiscApproximation(1);
    EXPECT_EQ(1, discApproxAgent.getNumDiscs());
    DiscApproximation discApproxOtherAgent = otherAgent.getDiscApproximation(1);
    EXPECT_EQ(1, discApproxOtherAgent.getNumDiscs());

    // transform the disks into the global frame of the agents
    discApproxAgent = discApproxAgent.transformFrom(agent.stateStamped().pose());
    discApproxOtherAgent = discApproxOtherAgent.transformFrom(otherAgent.stateStamped().pose());
    EXPECT_EQ(1, discApproxAgent.getNumDiscs());
    EXPECT_EQ(1, discApproxOtherAgent.getNumDiscs());
    EXPECT_EQ(Position2d(0.0, 0.0) , discApproxAgent.getPosition(0));
    EXPECT_EQ(Position2d(1.0, 1.0) , discApproxOtherAgent.getPosition(0));

    double distanceBetweenAgents = distance(discApproxAgent.getPosition(0UL), discApproxOtherAgent.getPosition(0UL));
    EXPECT_DOUBLE_EQ(sqrt(2.) , distanceBetweenAgents);
    EXPECT_TRUE(discApproxAgent.getRadius(0UL) + discApproxOtherAgent.getRadius(0UL) > distanceBetweenAgents );
  } catch (const exception& e) {
    FAIL() << e.what();
  }
}


TEST(planner_interfaces_TESTSUITE, Time) {

  try {

    {
      Time t;
      EXPECT_FALSE(t.isValid());
    }

    {
      Time t(0L);
      EXPECT_TRUE(t.isZero());
    }

    {
      Time t(1L);
      EXPECT_EQ(1, t.nanosec);
    }

    {
      Time t(1.0);
      EXPECT_EQ(1000000000, t.nanosec);
    }

    {
      Time t(1.1);
      EXPECT_EQ(1100000000, t.nanosec);
    }

    {
      Time t(time::fromSec(2.0) + 1100); // = 2 000 001 100 ns
      boost::posix_time::ptime tim = time::toBoost(t.nanosec);
  #if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
      EXPECT_EQ(2000001100L, tim.time_of_day().total_nanoseconds());
  #else
      EXPECT_EQ(2000001000L, tim.time_of_day().total_nanoseconds());
  #endif
      EXPECT_EQ(2000001L, tim.time_of_day().total_microseconds());
      EXPECT_EQ(2L, tim.time_of_day().total_seconds());
    }

    {
      Time t = time::getCurrentTime();
      std::cout << "Current time: " << t << std::endl;
      std::cout << "Current time: " << t.toDateString() << std::endl;
    }

    {
      Time t0(0L);
      Duration d(100L);
      Time t1 = t0 + d;
      EXPECT_EQ(100L, t1.nanosec);

      d.nanosec = -100L;
      t1 += d;
      EXPECT_EQ(0L, t1.nanosec);
    }

    {
      Time t(0L);
      const std::string date = t.toDateString();
      EXPECT_EQ("1970-Jan-01 00:00:00", date);
    }
    {
      Time t(0L);
      const std::string date = t.toDateString("%Y%m");
      EXPECT_EQ("197001", date);
    }

    {
      Duration t(70.123);
      const std::string dur = t.format("%H:%M:%S%F");
      EXPECT_EQ("00:01:10.123000", dur);
    }

    // Test formatting
    {
      using namespace planning2d::time;
      Time t(1.0);
      std::ostringstream os;
      os << fixed << t.format(Formatter(SEC, 3, "(", ")"));
      EXPECT_EQ("1.000(s)", os.str());
    }
  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(planner_interfaces_TESTSUITE, Duration) {

  try {

    {
      Duration d;
      EXPECT_EQ(0, d.toSec());
    }

    {
      Duration d(1L);
      EXPECT_EQ(1, d.nanosec);
      EXPECT_DOUBLE_EQ(1e-9, d.toSec());
    }

    {
      Duration d(1.0);
      EXPECT_EQ(1000000000, d.nanosec);
      EXPECT_DOUBLE_EQ(1.0, d.toSec());
    }

    {
      Duration d(1.0);
      EXPECT_DOUBLE_EQ(1.0, (double)d); // cast operator
    }

    {
      Duration d(0.1);
      const auto start = time::now();
      d.sleep();
      const auto now = time::now();
      EXPECT_NEAR(now.toSec(), (start + d).toSec(), 1e-3);
    }

    {
      // Check that rounding works with floating point multiplication and division
      Duration d(1.0);
      d *= 4./3.; // 1333333333.3333333
      EXPECT_DOUBLE_EQ(1333333333, d.nanosec); // round down
      d = d*0.5000000001; // 666666666.6333333
      EXPECT_DOUBLE_EQ(666666667, d.nanosec); // round up
      d = d/0.5000000001; // 1333333333.7333333
      EXPECT_DOUBLE_EQ(1333333334, d.nanosec); // round up
      d /= 4./3.; // 1000000000.5
      EXPECT_DOUBLE_EQ(1000000001, d.nanosec); // round up
    }

    // Test formatting
    {
      using namespace planning2d::time;
      Duration d(1.0);
      std::ostringstream os;
      os << fixed;
      std::streamsize prec = os.precision();
      os << d.format(Formatter(SEC, 0));
      EXPECT_EQ("1 [s]", os.str());
      EXPECT_EQ(prec, os.precision());
      os.str(""); os.clear();
      os << d.format(Formatter(SEC, 3, "(", ")"));
      EXPECT_EQ("1.000(s)", os.str());
      os.str(""); os.clear();
      os << d.format(Formatter(MILLISEC, 4, "{", "}"));
      EXPECT_EQ("1000.0000{ms}", os.str());
      os.str(""); os.clear();
      os << d.format(Formatter(MICROSEC, 1, "[", "&"));
      EXPECT_EQ("1000000.0[us&", os.str());
      os.str(""); os.clear();
      os << d.format(Formatter(NANOSEC, -1, "%", "%"));
      ostringstream expct;
      expct << "1000000000.";
      for (int i=0; i<prec; i++) expct << "0";
      expct << "%ns%";
      EXPECT_EQ(expct.str(), os.str());
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(planner_interfaces_TESTSUITE, Rate) {

  try {

    {
      Rate rate(10.0);
      EXPECT_EQ(Duration(0.1), rate.expectedCycleTime());
      EXPECT_EQ(Duration(0L), rate.cycleTime());
    }

    {
      Rate rate(Duration(0.1));
      EXPECT_EQ(Duration(0.1), rate.expectedCycleTime());
    }

    {
      const Duration cycleTime(0.1);
      Rate rate(cycleTime);
      const auto start = time::now();
      (cycleTime/2.).sleep(); // let 50 ms pass
      EXPECT_TRUE(rate.sleep()); // should sleep for another 50 ms to keep rate of 10 Hz
      const auto now = time::now();
      EXPECT_NEAR(rate.cycleTime().toSec(), cycleTime.toSec()/2., 1e-3);
      EXPECT_NEAR(now.toSec(), (start + cycleTime).toSec(), 1e-3);
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

inline double manhattanDistance(const Map<float>& map, const Map<float>::Index& index) {
  return index.template cast<float>().sum() * map.resolution();
}

inline void fillWithManhattanDistance(Map<float>& map) {
  Map<float>::Index index;
  for (index.x() = 0; static_cast<size_t>(index.x()) < map.sizeInCellsX(); ++index.x()) {
    for (index.y() = 0; static_cast<size_t>(index.y()) < map.sizeInCellsY(); ++index.y())
      map.at(index) = manhattanDistance(map, index);
  }
}

TEST(planner_interfaces_TESTSUITE, Map_Basics) {
  try
  {
    typedef Map<float> MapTest;
    typedef MapTest::Index Index;
    typedef Eigen::Matrix<MapTest::Index::Scalar, 2, 1> VectorIndex;

    // Test initialization
    {
      MapTest map(Pose2d(0.0, 0.0, 0.0), 0.1);

      EXPECT_DOUBLE_EQ(0.1, map.resolution());
      EXPECT_EQ(Pose2d(0.0, 0.0, 0.0), map.getOrigin());
      EXPECT_TRUE(map.empty());

      EXPECT_FALSE(map.isInsideMap(Index(-1, -1)));
      EXPECT_FALSE(map.isInsideMap(Index(0, 0)));
      EXPECT_FALSE(map.isInsideMap(Position2d(-1., -1.)));
      EXPECT_FALSE(map.isInsideMap(Position2d(0.0, 0.0)));

      EXPECT_ANY_THROW(map.at(Index(0, 0)));

      map.setResolution(0.2);
      EXPECT_DOUBLE_EQ(0.2, map.resolution());
      EXPECT_DOUBLE_EQ(1./0.2, map.reciprocalResolution());

      map.matrix() = MapTest::Matrix::Constant(1, 2, 1.f);
      EXPECT_FALSE(map.empty());
      EXPECT_NO_THROW(map.at(Index(0, 0)));
      EXPECT_EQ(2, map.sizeInCellsX());
      EXPECT_EQ(1, map.sizeInCellsY());
      EXPECT_DOUBLE_EQ(0.4, map.sizeInMetersX());
      EXPECT_DOUBLE_EQ(0.2, map.sizeInMetersY());
      EXPECT_EQ(Point2d<double>(0.4, 0.2), map.sizeInMeters());

    } /* Test initialization */

    // Test comparators
    {
      // Test with empty data
      EXPECT_TRUE(MapTest(Pose2d(0.0, 0.0, 0.0), 0.1) == MapTest(Pose2d(0.0, 0.0, 0.0), 0.1));
      EXPECT_TRUE(MapTest(Pose2d(0.0, 0.0, 0.0), 0.1, MapTest::Size2d(0,0)) == MapTest(Pose2d(0.0, 0.0, 0.0), 0.1, MapTest::Size2d(0,0)));

      // Test with matching data
      EXPECT_TRUE(MapTest(Pose2d(0.0, 0.0, 0.0), 0.1, MapTest::Size2d(1,1), 1.f) == MapTest(Pose2d(0.0, 0.0, 0.0), 0.1, MapTest::Size2d(1,1), 1.f));

      // Test with non-matching data
      EXPECT_FALSE(MapTest(Pose2d(0.0, 1.0, 0.0), 0.1) == MapTest(Pose2d(0.0, 0.0, 0.0), 0.1));
      EXPECT_FALSE(MapTest(Pose2d(0.0, 0.0, 0.0), 0.2, MapTest::Size2d(1,1)) == MapTest(Pose2d(0.0, 0.0, 0.0), 0.1, MapTest::Size2d(1,1)));
      EXPECT_FALSE(MapTest(Pose2d(0.0, 0.0, 0.0), 0.1, MapTest::Size2d(1,1), 2.f) == MapTest(Pose2d(0.0, 0.0, 0.0), 0.1, MapTest::Size2d(1,1), 1.f));
    } /* Test comparators */

    // Test aligned box size
    {
      MapTest map(Pose2d(0.0, 0.0, 0.0), 0.1, MapTest::Size2d(100,100));
      const auto boxi = map.alignedBoxIndex();
      EXPECT_EQ(0, boxi.min().x());
      EXPECT_EQ(0, boxi.min().y());
      EXPECT_EQ(map.sizeInCellsX(), boxi.max().x());
      EXPECT_EQ(map.sizeInCellsY(), boxi.max().y());
      EXPECT_EQ(map.sizeInCellsX(), boxi.sizes().x());
      EXPECT_EQ(map.sizeInCellsY(), boxi.sizes().y());
    } /* Test aligned box size */

    // Test out of bound access
    {
      MapTest map(Pose2d(0.0, 0.0, 0.0), 0.1, MapTest::Size2d(100,100), 1.f);
      EXPECT_THROW(map.at(Index(-1,+1)), planning2d::OutOfBoundAccessException);
      EXPECT_THROW(map.at(Index(+1,-1)), planning2d::OutOfBoundAccessException);
      EXPECT_THROW(map.at(Index(100,0)), planning2d::OutOfBoundAccessException);
      EXPECT_THROW(map.at(Index(0,100)), planning2d::OutOfBoundAccessException);
      EXPECT_NO_THROW(map.at(Index(0,0)));
    } /* Test out of bound access */

    // Test position/index conversion
    {
      MapTest map(Pose2d(1.0, 0.0, 0.0), 0.1, MapTest::Size2d(100,100), 1.f);
      EXPECT_EQ(Index(0,1), map.toIndex(Position2d(1.01, 0.11)));
      EXPECT_LT( (MapTest::InterpolatedIndex(0.1,1.1) - map.toInterpolatedIndex(Position2d(1.01, 0.11))).norm(), 1e-10 );
      EXPECT_NEAR(map.toPosition(MapTest::InterpolatedIndex(0.1,1.1)).x(), 1.01, 1e-6);
      EXPECT_NEAR(map.toPosition(MapTest::InterpolatedIndex(0.1,1.1)).y(), 0.11, 1e-6);
      EXPECT_EQ(Index(-1,100), map.toIndex(Position2d(1.0 - 0.01, 10.01)));
      EXPECT_EQ(Position2d(1.0 - 0.1, 10.0), map.toPosition(Index(-1,100)));

      EXPECT_TRUE(map.isInsideMap(Index(0,1)));
      EXPECT_FALSE(map.isInsideMap(Index(-1,100)));

      // test conversion with a yaw angle included
      map.setOrigin(Pose2d(1.0, 1.0, planning2d::PI/2.0));
      EXPECT_NEAR(-9.9, map.toInterpolatedIndex(Position2d(0.11, 0.01)).x(), 1e-6);
      EXPECT_NEAR(8.9, map.toInterpolatedIndex(Position2d(0.11, 0.01)).y(), 1e-6);
      EXPECT_EQ(Index(-10, 8), map.toIndex(Position2d(0.11, 0.01)));
      EXPECT_NEAR(0.2, map.toPosition(Index(-10, 8)).x(), 1e-6);
      EXPECT_NEAR(0.0, map.toPosition(Index(-10, 8)).y(), 1e-6);
    } /* Test position/index conversion */

    // Test iterators
    {
      // Test column major
      {
        typedef Map<size_t> MapTest;
        MapTest map(Pose2d(0.0, 0.0, 0.0), 1.0, MapTest::Size2d(5,5));

        Index index;
        size_t cnt = 0;

        // The map is stored in row major order. By iterating over x first, we're filling the values
        // in this order.
        for (index.y() = 0; index.y() < static_cast<int>(map.sizeInCellsY()); index.y()++)
          for (index.x() = 0; index.x() < static_cast<int>(map.sizeInCellsX()); index.x()++) {
            map(index) = cnt++;
        }

        cnt = 0;
        index = Index::Zero();
        for (MapTest::const_iterator it = map.begin(); it != map.end(); it++) {
          EXPECT_EQ(index, map.toIndex(it)) << "Failure at cnt = " << cnt;
          EXPECT_EQ(cnt, (*it)) << "Failure at cnt = " << cnt;
          index.x()++;
          cnt++;
          if (cnt % map.sizeInCellsX() == 0 && cnt > 0) {
            index.x() = 0;
            index.y()++;
          }
        }
      } /* Test column major */

      // Test row major
      {
        typedef Map<size_t, Eigen::ColMajor> MapTest;
        MapTest map(Pose2d(0.0, 0.0, 0.0), 1.0, MapTest::Size2d(5,5));

        Index index;
        size_t cnt = 0;

        // The map is stored in column major order. By iterating over y first, we're filling the values
        // in this order.
        for (index.x() = 0; index.x() < static_cast<int>(map.sizeInCellsX()); index.x()++) {
          for (index.y() = 0; index.y() < static_cast<int>(map.sizeInCellsY()); index.y()++)
            map(index) = cnt++;
        }

        cnt = 0;
        index = Index(0,0);
        for (MapTest::const_iterator it = map.begin(); it != map.end(); it++) {
          EXPECT_EQ(index, map.toIndex(it)) << "Failure at cnt = " << cnt;
          EXPECT_EQ(cnt, (*it)) << "Failure at cnt = " << cnt;
          index.y()++;
          cnt++;
          if (cnt % map.sizeInCellsY() == 0 && cnt > 0) {
            index.y() = 0;
            index.x()++;
          }
        }
      } /* Test row major */

    } /* Test iterators */

    // Test nearest neighbor index projection
    {
      MapTest map(Pose2d(0.0, 0.0, 0.0), 0.1, MapTest::Size2d(5,5));
      EXPECT_EQ(Index(0,0), map.projectToNearestIndex(Index(0,0)));
      EXPECT_EQ(Index(4,4), map.projectToNearestIndex(Index(4,10)));
      EXPECT_EQ(Index(4,2), map.projectToNearestIndex(Index(11,2)));
    } /* Test nearest neighbor index projection */

    // Test shift region
    {
      MapTest map(Pose2d(0.0, 0.0, 0.0), 1.0, MapTest::Size2d(5,5));
      MapTest::AlignedBoxIndex region;

      region = MapTest::AlignedBoxIndex(VectorIndex(4,4), VectorIndex(10,10)); // Too large in both directions
      EXPECT_THROW(map.shiftRegionToMap(region), planning2d::FunctionInputException);
      region = MapTest::AlignedBoxIndex(VectorIndex(4,4), VectorIndex(9,9)); // Identical size
      EXPECT_NO_THROW(map.shiftRegionToMap(region));
      region = MapTest::AlignedBoxIndex(VectorIndex(4,4), VectorIndex(10,5)); // Too large in x directions
      EXPECT_THROW(map.shiftRegionToMap(region), planning2d::FunctionInputException);
      region = MapTest::AlignedBoxIndex(VectorIndex(4,4), VectorIndex(5,10)); // Too large in y directions
      EXPECT_THROW(map.shiftRegionToMap(region), planning2d::FunctionInputException);

      region = MapTest::AlignedBoxIndex(VectorIndex(4,4), VectorIndex(6,6));
      map.shiftRegionToMap(region);
      EXPECT_EQ(3, region.min().x());
      EXPECT_EQ(5, region.max().x());
      EXPECT_EQ(3, region.min().y());
      EXPECT_EQ(5, region.max().y());

      region = MapTest::AlignedBoxIndex(VectorIndex(-1,-1), VectorIndex(1,1));
      map.shiftRegionToMap(region);
      EXPECT_EQ(0, region.min().x());
      EXPECT_EQ(2, region.max().x());
      EXPECT_EQ(0, region.min().y());
      EXPECT_EQ(2, region.max().y());

      region = MapTest::AlignedBoxIndex(VectorIndex(0,4), VectorIndex(2,6));
      map.shiftRegionToMap(region);
      EXPECT_EQ(0, region.min().x());
      EXPECT_EQ(2, region.max().x());
      EXPECT_EQ(3, region.min().y());
      EXPECT_EQ(5, region.max().y());

      region = MapTest::AlignedBoxIndex(VectorIndex(0,-2), VectorIndex(3,0));
      map.shiftRegionToMap(region);
      EXPECT_EQ(0, region.min().x());
      EXPECT_EQ(3, region.max().x());
      EXPECT_EQ(0, region.min().y());
      EXPECT_EQ(2, region.max().y());
    } /* Test shift region */

    // test inversion
    {
      MapTest map(Pose2d(0.0, 0.0, 0.0), 0.1, MapTest::Matrix::Random(5,5));
      map(2,2) = 0;
      MapTest inverted = map;
      inverted.invert();
      for (auto it = inverted.begin(); it != inverted.end(); ++it)
      {
        const auto index = inverted.toIndex(it);
        if (map(index) == 0.)
          EXPECT_EQ(255, *it);
        else
          EXPECT_EQ(0, *it);
      }
    } /* test inversion */

    // Test extrapolation
    {
      MapTest map(Pose2d(0.0, 0.0, 0.0), 0.1, MapTest::Size2d(5,5));
      fillWithManhattanDistance(map);

      // test lookup with extrapolated functions inside and outside the map
      Index index;
      for (index.x() = -10; index.x() < static_cast<int>(map.sizeInCellsX())+10; index.x()++) {
        for (index.y() = -10; index.y() < static_cast<int>(map.sizeInCellsY())+10; index.y()++) {
          const float expectedLinear = manhattanDistance(map, index);
          const float expectedConstant = map.isInsideMap(index) ? expectedLinear : manhattanDistance(map, map.projectToNearestIndex(index));
          EXPECT_FLOAT_EQ(expectedConstant, map.atExtrapolated<grid::ExtrapolationMethod::CONSTANT>(index)) << "Constant extrapolation failed at index " << index;
          // linear interpolation should be exact outside the map with Manhattan distance
          EXPECT_NEAR(expectedLinear, map.atExtrapolated<grid::ExtrapolationMethod::LINEAR>(index), 1e-4) << "Linear extrapolation failed at index " << index;
        }
      }

    } /* Test extrapolation */

    // Test region extraction
    {
      MapTest map(Pose2d(0.0, 0.0, 0.0), 1.0f, MapTest::Size2d(5,5));
      fillWithManhattanDistance(map);

      // Test block operation
      Index min(0,0);
      MapTest::Size2d size(2,1);
      MapTest::Matrix matBlock = map.block(min, size);
      EXPECT_EQ(2, matBlock.cols());
      EXPECT_EQ(1, matBlock.rows());
      EXPECT_FLOAT_EQ(0.f, matBlock(0,0));
      EXPECT_FLOAT_EQ(1.f, matBlock(0,1));

      // Test getRegion() for a fully overlapping region
      MapTest::AlignedBoxIndex region(min.asVector(), size.template cast<MapTest::Index::Scalar>().asVector());
      MapTest::Matrix matRegion = map.getRegion<MapTest::ExtrapolationMethod::NONE>(region);
      MapTest::Matrix matRegion2 = map.getRegion<2, 1, MapTest::ExtrapolationMethod::NONE>(min);
      EXPECT_EQ(2, matRegion.cols());
      EXPECT_EQ(1, matRegion.rows());
      EXPECT_EQ(2, matRegion2.cols());
      EXPECT_EQ(1, matRegion2.rows());
      EXPECT_FLOAT_EQ(0.f, matRegion(0,0));
      EXPECT_FLOAT_EQ(1.f, matRegion(0,1));
      EXPECT_FLOAT_EQ(0.f, matRegion2(0,0));
      EXPECT_FLOAT_EQ(1.f, matRegion2(0,1));

      // test not fully overlapping region
      {

        // at upper right corner
        min = Index(4,4);
        size = MapTest::Size2d(2,2);
        region = MapTest::AlignedBoxIndex(min.asVector(), min.asVector() + size.template cast<MapTest::Index::Scalar>().asVector());

        // no extrapolation
        EXPECT_THROW(map.getRegion<MapTest::ExtrapolationMethod::NONE>(region), planning2d::OutOfBoundAccessException);

        // constant extrapolation
        matRegion = map.getRegion<MapTest::ExtrapolationMethod::CONSTANT>(region);
        matRegion2 = map.getRegion<2, 2, MapTest::ExtrapolationMethod::CONSTANT>(min);
        EXPECT_TRUE(matRegion.isApprox(MapTest::Matrix::Constant(2, 2, 8.f))) << "matRegion: " << std::endl << matRegion;
        EXPECT_TRUE(matRegion2.isApprox(MapTest::Matrix::Constant(2, 2, 8.f))) << "matRegion: " << std::endl << matRegion2;

        // linear extrapolation
        matRegion = map.getRegion<MapTest::ExtrapolationMethod::LINEAR>(region);
        EXPECT_TRUE(matRegion.isApprox((MapTest::Matrix(2, 2) << 8.f, 9.f, 9.f, 10.f).finished())) << "matRegion: " << std::endl << matRegion;

        // at lower left corner
        min = Index(-1,-1);
        size = MapTest::Size2d(2,2);
        region = MapTest::AlignedBoxIndex(min.asVector(), min.asVector() + size.template cast<MapTest::Index::Scalar>().asVector());

        // no extrapolation
        EXPECT_THROW(map.getRegion<MapTest::ExtrapolationMethod::NONE>(region), planning2d::OutOfBoundAccessException);

        // constant extrapolation
        matRegion = map.getRegion<MapTest::ExtrapolationMethod::CONSTANT>(region);
        matRegion2 = map.getRegion<2,2,MapTest::ExtrapolationMethod::CONSTANT>(min);
        EXPECT_TRUE(matRegion.isApprox(MapTest::Matrix::Constant(2, 2, 0.f))) << "matRegion: " << std::endl << matRegion;
        EXPECT_TRUE(matRegion2.isApprox(MapTest::Matrix::Constant(2, 2, 0.f))) << "matRegion2: " << std::endl << matRegion2;

        // linear extrapolation
        matRegion = map.getRegion<MapTest::ExtrapolationMethod::LINEAR>(region);
        EXPECT_TRUE(matRegion.isApprox((MapTest::Matrix(2, 2) << -2.f, -1.f, -1.f, 0.f).finished())) << "matRegion: " << std::endl << matRegion;
      } /* test not fully overlapping region */

    } /* Test region extraction */

  } catch (const exception& e)
  {
    FAIL() << e.what();
  }
}

TEST(planner_interfaces_TESTSUITE, Map_Interpolation) {
  try
  {
    typedef Map<float> MapTest;
    typedef MapTest::Index Index;
    MapTest map(Pose2d(0.0, 0.0, 0.0), 0.1, MapTest::Size2d(10,10), 1.f);

    // lookup at lower left corner (1,1)
    // all extrapolation methods should return the same value since no extrapolation takes place
    MapTest::InterpolatedIndex idxp = map.toInterpolatedIndex(Position2d(0.11, 0.18));
    EXPECT_FLOAT_EQ(1.f, map.atInterpolatedBilinear(idxp, MapTest::ExtrapolationMethod::NONE));
    EXPECT_FLOAT_EQ(1.f, map.atInterpolatedBilinear(idxp, MapTest::ExtrapolationMethod::CONSTANT));
    EXPECT_FLOAT_EQ(1.f, map.atInterpolatedBilinear(idxp, MapTest::ExtrapolationMethod::LINEAR));

    // set the two lower cells of the four used for interpolation to 2.0
    map.at(Index(1,1)) = 2.f;
    map.at(Index(2,1)) = 2.f;
    EXPECT_FLOAT_EQ(0.2f*2.0f + 0.8f*1.0f, map.atInterpolatedBilinear(idxp, MapTest::ExtrapolationMethod::NONE));

    // set the lower right cell to 3.0
    map.at(Index(2,1)) = 3.f;
    EXPECT_FLOAT_EQ(0.2f*0.9f*2.0f + 0.2f*0.1f*3.f + 0.8f*1.0f, map.atInterpolatedBilinear(idxp, MapTest::ExtrapolationMethod::NONE));

    // Test lookup at border
    idxp = map.toInterpolatedIndex(Position2d(0.91, 0.91)); // index (9,9)
    map.at(Index(8,8)) = 1.f;
    map.at(Index(8,9)) = 2.f;
    map.at(Index(9,8)) = 3.f;
    map.at(Index(9,9)) = 4.f;
    EXPECT_THROW(map.atInterpolatedBilinear(idxp, MapTest::ExtrapolationMethod::NONE), planning2d::OutOfBoundAccessException);
    EXPECT_FLOAT_EQ(4.f, map.atInterpolatedBilinear(idxp, MapTest::ExtrapolationMethod::CONSTANT));
    EXPECT_FLOAT_EQ(0.9f*(0.9f*4.f + 0.1f*6.f) + 0.1f*(0.9f*5.f + 0.1f*7.f), map.atInterpolatedBilinear(idxp, MapTest::ExtrapolationMethod::LINEAR));

    // test interpolation values on a Manhattan distance map from pixel 0.0
    fillWithManhattanDistance(map);

    for (idxp.y() = 0.0; idxp.y() < (double)map.sizeInCellsY(); idxp.y() += 1./3.) {
      for (idxp.x() = 0.0; idxp.x() < (double)map.sizeInCellsX(); idxp.x() += 1./3.) {
        const float expected = map.toPosition(idxp).template cast<MapTest::Scalar>().sum();

        // Bilinear interpolation
        try {
          double val = map.atInterpolatedBilinear(idxp, MapTest::ExtrapolationMethod::NONE);
          EXPECT_NEAR(expected, val, 1e-5) << "Bilinear interpolation failure at interpolated index " << idxp;
          // if the lookup with no extrapolation succeeds then no extrapolation is needed,
          // hence both bilinear extrapolation functions should be accurate and return the same value
          const double valConstExtrapolation = map.atInterpolatedBilinear(idxp, MapTest::ExtrapolationMethod::CONSTANT);
          const double valLinExtrapolation = map.atInterpolatedBilinear(idxp, MapTest::ExtrapolationMethod::LINEAR);
          EXPECT_NEAR(expected, valConstExtrapolation, 1e-5) <<
              "Bilinear interpolation failed at interpolated index " << idxp << " with extrapolation " << MapTest::ExtrapolationMethod::CONSTANT;
          EXPECT_NEAR(expected, valLinExtrapolation, 1e-5) <<
              "Bilinear interpolation failed at interpolated index " << idxp << " with extrapolation " << MapTest::ExtrapolationMethod::LINEAR;
          EXPECT_DOUBLE_EQ(valConstExtrapolation, valLinExtrapolation) << "Bilinear interpolation failed at interpolated index " << idxp;
        } catch (const planning2d::OutOfBoundAccessException& e) { // we need extrapolation
          // constant extrapolation: constant extrapolation will not estimate the values beyond the boundaries correctly -> higher tolerance
          EXPECT_NEAR(expected, map.atInterpolatedBilinear(idxp, MapTest::ExtrapolationMethod::CONSTANT), 3e-1) <<
              "Bilinear interpolation failed at interpolated index " << idxp << " with extrapolation " << MapTest::ExtrapolationMethod::CONSTANT;
          // linear extrapolation: linear extrapolation on Manhattan distance map should be accurate -> low tolerance
          EXPECT_NEAR(expected, map.atInterpolatedBilinear(idxp, MapTest::ExtrapolationMethod::LINEAR), 1e-5) <<
              "Bilinear interpolation failed at interpolated index " << idxp << " with extrapolation " << MapTest::ExtrapolationMethod::CONSTANT;
        }

        // Bicubic interpolation
        try {
          // no extrapolation
          double val = map.atInterpolatedBicubic(idxp, MapTest::ExtrapolationMethod::NONE);
          EXPECT_NEAR(expected, val, 1e-2) << "Bicubic interpolation failed at interpolated index " << idxp;
          // if the lookup with no extrapolation succeeds then no extrapolation is needed,
          // hence both bicubic extrapolation functions should return the same value
          const double valConstExtrapolation = map.atInterpolatedBicubic(idxp, MapTest::ExtrapolationMethod::CONSTANT);
          const double valLinExtrapolation = map.atInterpolatedBicubic(idxp, MapTest::ExtrapolationMethod::LINEAR);
          EXPECT_NEAR(expected, valConstExtrapolation, 1e-2) <<
              "Bicubic interpolation failed at interpolated index " << idxp << " with extrapolation " << MapTest::ExtrapolationMethod::CONSTANT;
          EXPECT_NEAR(expected, valLinExtrapolation, 1e-2) <<
              "Bicubic interpolation at interpolated index " << idxp << " with extrapolation " << MapTest::ExtrapolationMethod::LINEAR;
          EXPECT_DOUBLE_EQ(valConstExtrapolation, valLinExtrapolation) << "Bicubic interpolation at interpolated index " << idxp;
        } catch (const planning2d::OutOfBoundAccessException& e) { // we need extrapolation
          // constant extrapolation: constant extrapolation will not estimate the values beyond the boundaries correctly -> higher tolerance
          EXPECT_NEAR(expected, map.atInterpolatedBicubic(idxp, MapTest::ExtrapolationMethod::CONSTANT), 5e-1) <<
              "Bicubic interpolation at interpolated index " << idxp << " with extrapolation " << MapTest::ExtrapolationMethod::CONSTANT;
          // linear extrapolation: linear extrapolation on Manhattan distance map should be accurate -> low tolerance
          EXPECT_NEAR(expected, map.atInterpolatedBicubic(idxp, MapTest::ExtrapolationMethod::LINEAR), 1e-2) <<
              "Bicubic interpolation at interpolated index " << idxp << " with extrapolation " << MapTest::ExtrapolationMethod::LINEAR;
        }
      }
    }

  } catch (const exception& e)
  {
    FAIL() << e.what();
  }
}


template <typename T>
struct MapInterpolateFunctor {
  typedef Eigen::Matrix<double, 1, 1> Vector1d;
  typedef Eigen::Vector2d input_t;
  typedef Vector1d value_t;
  typedef double scalar_t;
  typedef Eigen::RowVector2d jacobian_t;
  MapInterpolateFunctor(const Map<T>& map, const int interpolationOrder,
                        const typename Map<T>::ExtrapolationMethod extrapolationOrder = Map<T>::ExtrapolationMethod::CONSTANT)
      : _map(map), _interpolationOrder(interpolationOrder), _extrapolation(extrapolationOrder) { }
  input_t update(const input_t& x0, unsigned c, scalar_t delta) {
    input_t xout = x0;
    xout[c] += delta;
    return xout;
  }
  value_t operator()(const input_t & dr) {
    Vector1d out;
    typename Map<T>::InterpolatedIndex idxp = _map.toInterpolatedIndex(Position2d(dr));
    if (_interpolationOrder == 1)
      out << _map.atInterpolatedBilinear(idxp, _extrapolation);
    else
      out << _map.atInterpolatedBicubic(idxp, _extrapolation);
    return out;
  }
  const Map<T>& _map;
  int _interpolationOrder;
  typename Map<T>::ExtrapolationMethod _extrapolation;
};

TEST(planner_interfaces_TESTSUITE, Map_Gradient) {

  try
  {
    typedef Map<float> MapTest;
    MapTest map(Pose2d(0.0, 0.0, 0.0), 0.5, MapTest::Matrix::Random(15,15));

    MapTest::InterpolatedIndex idxp;
    const MapTest::ExtrapolationMethod extrapolation = MapTest::ExtrapolationMethod::CONSTANT;
    typedef MapInterpolateFunctor<float> Functor;
    sm::eigen::NumericalDiff<Functor> numdiffBilinear(Functor(map, 1, extrapolation));
    sm::eigen::NumericalDiff<Functor> numdiffBicubic(Functor(map, 2, extrapolation));

    // Test bilinear interpolation
    double border = 1e-4; // stay away from the border of the grid cells since the bilinear interpolation does not provide continuous gradients
    for (idxp.y() = 0.0 + border; idxp.y() < (double)map.sizeInCellsY(); idxp.y() += 1./3.) {
      for (idxp.x() = 0.0 + border; idxp.x() < (double)map.sizeInCellsX(); idxp.x() += 1./3.) {

        Position2d pos = map.toPosition(idxp);
        Functor::jacobian_t gradEstBilinear = numdiffBilinear.estimateJacobian(pos.asVector());

        Functor::jacobian_t gradCompBilinear;
        map.gradientInterpolatedBilinear(idxp, gradCompBilinear.x(), gradCompBilinear.y(), extrapolation);

        SCOPED_TRACE(testing::Message() << "Testing gradient of bilinear interpolation at position " << pos);
        sm::eigen::expectNear(gradEstBilinear, gradCompBilinear, 1e-5, SM_SOURCE_FILE_POS);
      }
    }

    // Test bicubic interpolation
    for (idxp.y() = 0.0; idxp.y() < (double)map.sizeInCellsY(); idxp.y() += 1./3.) {
      for (idxp.x() = 0.0; idxp.x() < (double)map.sizeInCellsX(); idxp.x() += 1./3.) {

        Position2d pos = map.toPosition(idxp);
        Functor::jacobian_t gradEstBicubic = numdiffBicubic.estimateJacobian(pos.asVector());

        Functor::jacobian_t gradCompBicubic;
        map.gradientInterpolatedBicubic(idxp, gradCompBicubic.x(), gradCompBicubic.y(), extrapolation);

        SCOPED_TRACE(testing::Message() << "Testing gradient of bicubic interpolation at position " << pos);
        sm::eigen::expectNear(gradEstBicubic, gradCompBicubic, 1e-5, SM_SOURCE_FILE_POS);
      }
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(planner_interfaces_TESTSUITE, Map_Merge) {

  try {

    typedef Map<float> MapTest;
    MapTest m0(Pose2d(0.0, 0.0, 0.0), 1.0, MapTest::Matrix::Random(3,3));

    for (bool fillComplement : {false, true}) {

      boost::optional<float> fillValue = fillComplement ? 1.f : boost::optional<float>();

      { // non-overlapping
        MapTest m1(Pose2d(5.0, 4.0, 0.0), 1.0, MapTest::Matrix::Random(10,4));
        const auto m1Bak = m1;
        m0.mergeInto(m1, [&](const float lhs, const float rhs) { return lhs + rhs; }, fillValue);
        if (fillComplement)
          sm::eigen::assertEqual(MapTest::Matrix::Constant(m1.matrix().rows(), m1.matrix().cols(), fillValue.get()), m1.matrix(), SM_SOURCE_FILE_POS, "Testing non-overlapping map merge with equal metrics");
        else
          sm::eigen::assertEqual(m1Bak.matrix(), m1.matrix(), SM_SOURCE_FILE_POS, "Testing non-overlapping map merge with equal metrics");
      }

      { // equal metrics
        MapTest m1(Pose2d(0.0, 0.0, 0.0), 1.0, MapTest::Matrix::Random(3,3));
        const auto m1Bak = m1;
        m0.mergeInto(m1, [&](const float lhs, const float rhs) { return lhs + rhs; }, fillValue);
        sm::eigen::assertEqual(m0.matrix() + m1Bak.matrix(), m1.matrix(), SM_SOURCE_FILE_POS, "Testing map merge with equal metrics");
      }

      { // aligned
        MapTest m1(Pose2d(1.0, 2.0, 0.0), 1.0, MapTest::Matrix::Random(4,5));
        const auto m1Bak = m1;
        m0.mergeInto(m1, [&](const float lhs, const float rhs) { return lhs + rhs; }, fillValue);
        for (auto it = m1.begin(); it != m1.end(); ++it) {
          const auto index = m1.toIndex(it);
          if (index == MapTest::Index(0,0) || index == MapTest::Index(1,0))
            EXPECT_FLOAT_EQ(m0.at(index + MapTest::Index(1,2)) + m1Bak.at(index), m1.at(index));
          else
            EXPECT_FLOAT_EQ(fillComplement ? fillValue.get() : m1Bak.at(index), m1.at(index)) << "Error with fillComplement = " << fillComplement;
        }
      }

      { // unaligned
        MapTest m0(Pose2d(0.0, 0.0, 0.0), 1.0, MapTest::Matrix::Random(3,3));
        for (int i = 0; i < m0.matrix().size(); i++) { auto d = m0.matrix().data() + i; *d = i; };
        MapTest m1(Pose2d(0.0, 2.99, -PI/2.), 1.0, MapTest::Matrix::Random(3,3));
        for (int i = 0; i < m1.matrix().size(); i++) { auto d = m1.matrix().data() + i; *d = i; };
        const auto m1Bak = m1;
        m0.mergeInto(m1, [&](const float lhs, const float rhs) { return lhs + rhs; }, fillValue);
        sm::eigen::assertEqual((m0.matrix().colwise().reverse()).transpose() + m1Bak.matrix(), m1.matrix(), SM_SOURCE_FILE_POS, "Testing unaligned map merge");
      }
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(planner_interfaces_TESTSUITE, OccupancyGrid) {

  try {

    typedef OccupancyGrid::Index Index;
    OccupancyGrid::Matrix mat;
    OccupancyGrid grid(Pose2d(0.0, 0.0, 0.0), 0.1);

    {
      mat = OccupancyGrid::Matrix::Constant(100, 100, OccupancyValue::FREE);
      grid.matrix() = mat;
      EXPECT_EQ(OccupancyGrid::Size2d(100,100), grid.sizeInCells());

      grid.at(0,10) = OccupancyValue::UNKNOWN;
      EXPECT_EQ(grid.at(0,10), OccupancyValue::UNKNOWN);
      EXPECT_EQ(grid.at(Index(0,10)), OccupancyValue::UNKNOWN);
      EXPECT_EQ(grid.at(Position2d(0.0, 1.0)), OccupancyValue::UNKNOWN);
      EXPECT_NE(grid.at(Position2d(0.0, 1.2)), OccupancyValue::UNKNOWN);

      grid.set(0,10, OccupancyValue::OCCUPIED);
      EXPECT_EQ(grid.at(0,10), OccupancyValue::OCCUPIED);
      EXPECT_EQ(grid.at(Index(0,10)), OccupancyValue::OCCUPIED);
      EXPECT_EQ(grid.at(Position2d(0.0, 1.0)), OccupancyValue::OCCUPIED);
      EXPECT_NE(grid.at(Position2d(0.0, 1.2)), OccupancyValue::OCCUPIED);
    }

    {
      OccupancyGrid::Index oidx(99,0);
      mat = OccupancyGrid::Matrix::Constant(100, 100, OccupancyValue::FREE);
      mat(oidx.y(),oidx.x()) = OccupancyValue::OCCUPIED;
      grid.matrix() = mat;
      EXPECT_EQ(OccupancyGrid::Size2d(100,100), grid.sizeInCells());

      EXPECT_EQ(Position2d(0.1,0.1), grid.toPosition(OccupancyGrid::Index(1,1)));
      EXPECT_EQ(OccupancyGrid::Index(3,4), grid.toIndex(Position2d(0.31, 0.41)));

      EXPECT_EQ(OccupancyValue::OCCUPIED, grid.at(oidx)); // Note: Occupancy grid defines positions and indices wrt. to lower left corner, x-direction to the right (rows)
      EXPECT_EQ(OccupancyValue::OCCUPIED, grid.at(Position2d(9.91, 0.0)));

      EXPECT_TRUE(grid.isOccupied(Position2d(9.91, 0.0)));
      EXPECT_FALSE(grid.isOccupied(Position2d(8.91, 0.0)));
    }

    // Test metric check
    {
      Map<int> m1(Pose2d(0.0, 0.0, 0.0), 0.1, Map<int>::Size2d(2,2));
      {
        Map<float> m2(Pose2d(0.0, 0.0, 0.0), 0.1, Map<float>::Size2d(2,2));
        EXPECT_TRUE(m1.hasEqualMetric(m2));
      }
      {
        Map<double> m2(Pose2d(0.0, 0.0, 0.0), 0.2, Map<float>::Size2d(2,2));
        EXPECT_FALSE(m1.hasEqualMetric(m2));
      }
      {
        Map<double> m2(Pose2d(0.0, 0.0, 0.0), 0.1, Map<float>::Size2d(2,3));
        EXPECT_FALSE(m1.hasEqualMetric(m2));
      }
      {
        Map<double> m2(Pose2d(0.1, 0.0, 0.0), 0.1, Map<float>::Size2d(2,3));
        EXPECT_FALSE(m1.hasEqualMetric(m2));
      }
    }

    // Test is boundary methods
    {
      mat = OccupancyGrid::Matrix::Constant(5, 5, OccupancyValue::FREE);
      grid.matrix() = mat;
      grid.block<3,3>(OccupancyGrid::Index(0,0)) = OccupancyGrid::Matrix::Constant(3, 3, OccupancyValue::OCCUPIED);
      EXPECT_TRUE(grid.isMapBoundary(OccupancyGrid::Index(0, 0)));
      EXPECT_TRUE(grid.isMapBoundary(OccupancyGrid::Index(0, 1)));
      EXPECT_TRUE(grid.isMapBoundary(OccupancyGrid::Index(1, 0)));
      EXPECT_TRUE(grid.isMapBoundary(OccupancyGrid::Index(0, 1)));
      EXPECT_TRUE(grid.isMapBoundary(OccupancyGrid::Index(grid.sizeInCellsX()-1, 0)));
      EXPECT_TRUE(grid.isMapBoundary(OccupancyGrid::Index(0, grid.sizeInCellsY()-1)));
      EXPECT_TRUE(grid.isMapBoundary(OccupancyGrid::Index(grid.sizeInCellsX()-1, 1)));
      EXPECT_TRUE(grid.isMapBoundary(OccupancyGrid::Index(1, grid.sizeInCellsY()-1)));
      EXPECT_FALSE(grid.isMapBoundary(OccupancyGrid::Index(1,1)));

      EXPECT_TRUE(grid.isObstacleBoundary(OccupancyGrid::Index(0, 0)));
      EXPECT_TRUE(grid.isObstacleBoundary(OccupancyGrid::Index(1, 0)));
      EXPECT_FALSE(grid.isObstacleBoundary(OccupancyGrid::Index(1, 1)));
      EXPECT_TRUE(grid.isObstacleBoundary(OccupancyGrid::Index(1, 2)));
      EXPECT_TRUE(grid.isObstacleBoundary(OccupancyGrid::Index(2, 2)));
      EXPECT_FALSE(grid.isObstacleBoundary(OccupancyGrid::Index(3, 3)));
    }

    // test shift
    {
      mat = OccupancyGrid::Matrix::Constant(100, 100, OccupancyValue::FREE);
      grid.matrix() = mat;
      grid.at(Index(20,30)) = OccupancyValue::OCCUPIED;
      grid.shift(Position2d(1.0, 2.0), OccupancyValue::UNKNOWN);
      EXPECT_EQ(OccupancyValue::FREE, grid.at(Index(20,30)));
      EXPECT_EQ(OccupancyValue::OCCUPIED, grid.at(Index(10,10)));
      EXPECT_EQ(OccupancyValue::UNKNOWN, grid.at(Index(91,81)));
    } /* test shift */

    // test inversion
    {
      OccupancyGrid grid(Pose2d(0.0, 0.0, 0.0), 0.1, OccupancyGrid::Size2d(5,5), OccupancyValue::FREE);
      grid.at(OccupancyGrid::Index(2,2)) = OccupancyValue::OCCUPIED;
      grid.invert();
      EXPECT_EQ(OccupancyValue::FREE, grid.at(OccupancyGrid::Index(2,2)));
      EXPECT_EQ(OccupancyValue::OCCUPIED, grid.at(OccupancyGrid::Index(0,0)));
    } /* test inversion */

    // test OccupancyGridStamped
    {

      // Test constructors
      OccupancyGridStamped grid1(Time(1.0));
      EXPECT_EQ(Time(1.0), grid1.stamp());
      grid1 = OccupancyGridStamped(Pose2d(0.0, 0.0, 0.0), 0.1, Time(1.0));
      EXPECT_EQ(Time(1.0), grid1.stamp());
      grid1 = OccupancyGridStamped(Pose2d(0.0, 0.0, 0.0), 0.1, OccupancyGrid::Matrix::Constant(5,5,OccupancyValue::FREE), Time(1.0));
      EXPECT_EQ(Time(1.0), grid1.stamp());
      grid1 = OccupancyGridStamped(Pose2d(0.0, 0.0, 0.0), 0.1, OccupancyGrid::Size2d(5,5), Time(1.0));
      EXPECT_EQ(Time(1.0), grid1.stamp());
      grid1 = OccupancyGridStamped(Pose2d(0.0, 0.0, 0.0), 0.1, OccupancyGrid::Size2d(5,5), OccupancyValue::FREE, Time(1.0));
      EXPECT_EQ(Time(1.0), grid1.stamp());

      OccupancyGridStamped grid0 = grid1;
      ++grid0.stamp().nanosec;
      EXPECT_FALSE(grid0 == grid1);
      EXPECT_TRUE(grid0 != grid1);
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}


TEST(planner_interfaces_TESTSUITE, OccupancyGridInflation) {
#ifdef NDEBUG

  try {
    // test inflation
    OccupancyGrid::Matrix mat;
    OccupancyGrid grid(Pose2d(0.0, 0.0, 0.0), 0.1);
    mat = OccupancyGrid::Matrix::Constant(501, 501, OccupancyValue::FREE);
    mat(250,250) = OccupancyValue::OCCUPIED;
    mat(0,0) = OccupancyValue::UNKNOWN;
    grid.matrix() = mat;

    const double inflationRadius = 2.0;
    Time start = time::getCurrentTime();
    OccupancyGrid::MatrixXb kernel = grid.computeInflationKernel(inflationRadius);
    SM_INFO_STREAM("Inflation kernel computation ( grid size " << grid.sizeInCells() << " cells) took " << (time::getCurrentTime() - start).format(time::Formatter(time::MILLISEC)));
    EXPECT_EQ(kernel.rows(), kernel.cols());
    EXPECT_EQ(std::ceil(inflationRadius*grid.reciprocalResolution())*2 + 1, kernel.cols());
    start = time::getCurrentTime();
    grid.inflateWithKernel(kernel, false);
    SM_INFO_STREAM("Grid inflation " << grid.sizeInCells() << " cells) took " << (time::getCurrentTime() - start).format(time::Formatter(time::MILLISEC)));
    EXPECT_TRUE(grid.isOccupied(Position2d(25.0, 25.0)));
    EXPECT_TRUE(grid.isOccupied(Position2d(26.9, 25.0)));
    EXPECT_TRUE(grid.isOccupied(Position2d(25.0, 26.9)));
    EXPECT_FALSE(grid.isOccupied(Position2d(27.1, 25.0)));
    EXPECT_FALSE(grid.isOccupied(Position2d(25.0, 27.1)));
    EXPECT_FALSE(grid.isOccupied(Position2d(25.0, 27.1)));
    EXPECT_FALSE(grid.isOccupied(Position2d(0.0, 0.0))); // since treatUnknownOccupied is false

    // now with unknown = OCCUPIED
    grid.matrix() = mat;
    grid.inflateWithKernel(kernel, true);
    EXPECT_TRUE(grid.isOccupied(Position2d(25.0, 25.0)));
    EXPECT_TRUE(grid.isOccupied(Position2d(26.9, 25.0)));
    EXPECT_TRUE(grid.isOccupied(Position2d(25.0, 26.9)));
    EXPECT_FALSE(grid.isOccupied(Position2d(27.1, 25.0)));
    EXPECT_FALSE(grid.isOccupied(Position2d(25.0, 27.1)));
    EXPECT_FALSE(grid.isOccupied(Position2d(25.0, 27.1)));
    EXPECT_TRUE(grid.isOccupied(Position2d(0.0, 0.0))); // since treatUnknownOccupied is true
    EXPECT_TRUE(grid.isOccupied(Position2d(0.0 + inflationRadius/2., 0.0 + inflationRadius/2.))); // since treatUnknownOccupied is true
  } catch (const exception& e) {
    FAIL() << e.what();
  }

#else
  SM_WARN_STREAM("Grid inflation will only run when compiled in Release mode");
#endif
}

TEST(planner_interfaces_TESTSUITE, Trajectories) {
  PoseTrajectory trajectory;
  const Duration dt(0.1);
  for (Time t(0.0); t<Time(10.0); t+=Duration(0.1))
    trajectory.push_back( Pose2dStamped(Pose2dStamped::Vector::Random(), t) );

  PoseTrajectory extracted = extractRange(Time(1.0), Time(3.0), trajectory);
  size_t cnt = 0;
  EXPECT_EQ(Time(1.0), extracted.front().stamp());
  for (auto& item : extracted) {
    EXPECT_EQ(Time(1.0) + dt*static_cast<double>(cnt), item.stamp());
    cnt++;
  }
  EXPECT_EQ(Time(3.0), extracted.back().stamp());
}


TEST(planner_interfaces_TESTSUITE, PlannerInterface) {

  try {
    TestPlanner planner;

    ReturnCode retCode;

    TestAgent::StateStamped egoState;
    egoState.pose() = Pose2d(10.0, 10.0, 0.0);
    egoState.stamp() = (time::getCurrentTime());
    egoState.linearVelocity() = 0.5;

    std::vector<Agent::ConstPtr> dynamicObjects(1);
    dynamicObjects[0].reset(new TestAgent(1 /*id*/));

    Path referencePath(1);
    referencePath[0] = Pose2d(20., 20., 0.);

    // create occupancy grid
    OccupancyGridStamped grid(Pose2d(5.0, 5.0, 0.0), 0.1, Time(0.0));
    OccupancyGrid::Matrix mat = OccupancyGrid::Matrix::Constant(100,100, OccupancyValue::FREE);
    mat(50,50) = OccupancyValue::OCCUPIED; // the robot should stand exactly on that pixel
    grid.matrix() = mat;

    // call all the callbacks
    planner.callbackDynamicObjects(dynamicObjects);
    planner.callbackCurrentState(egoState);
    planner.callbackReferencePath(referencePath);
    planner.callbackOccupancyGrid(grid);

    // compute solution trajectory
    StateInputTrajectory solutionTrajectory;
    sm::logging::Level loglevel = sm::logging::getLevel();
    sm::logging::setLevel(sm::logging::Level::Fatal); // increase to suppress error
    retCode = planner.computePlan(time::getCurrentTime(), solutionTrajectory);
    EXPECT_EQ(RETCODE_ROBOT_IN_COLLISION_NOW, retCode);
    sm::logging::setLevel(loglevel);

    egoState.pose().position().x() = 11.; // move robot away from obstacle in gridmap
    planner.callbackCurrentState(egoState);
    retCode = planner.computePlan(time::getCurrentTime(), solutionTrajectory);
    EXPECT_EQ(RETCODE_OK, retCode);
  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(planner_interfaces_TESTSUITE, Serialization) {

  try {
    // Test the serialization methods of the classes

    Position2d position(1.0,2.0);
    EXPECT_EQ(position, deserialize<Position2d>(serialize<Position2d>(position)));

    Position2dStamped positionStamped(1.0,2.0,Time(1.1));
    EXPECT_EQ(positionStamped, deserialize<Position2dStamped>(serialize<Position2dStamped>(positionStamped)));

    Pose2d pose(1.0,2.0,3.0);
    EXPECT_EQ(pose, deserialize<Pose2d>(serialize<Pose2d>(pose)));

    Pose2dStamped poseStamped(1.0,2.0,3.0,Time(1.1));
    EXPECT_EQ(poseStamped, deserialize<Pose2dStamped>(serialize<Pose2dStamped>(poseStamped)));

    State state(State::T::Constant(3, 1, 1.0), pose);
    EXPECT_EQ(state, deserialize<State>(serialize<State>(state)));

    StateStamped stateStamped(State(State::T::Constant(3, 1, 1.0), pose), Time(100L));
    EXPECT_EQ(stateStamped, deserialize<StateStamped>(serialize<StateStamped>(stateStamped)));

    TestAgent::StateStamped testStateStamped(planning2d::StateStamped(State(State::T::Constant(3, 1, 1.0), pose), Time(100L)));
    EXPECT_EQ(testStateStamped, deserialize<TestAgent::StateStamped>(serialize<TestAgent::StateStamped>(testStateStamped)));

    SystemInput input(SystemInput(SystemInput::T::Constant(3, 1, 1.0)));
    EXPECT_EQ(input, deserialize<SystemInput>(serialize<SystemInput>(input)));

    SystemInputStamped inputStamped(SystemInput(SystemInput::T::Constant(3, 1, 1.0)), Time(100L));
    EXPECT_EQ(inputStamped, deserialize<SystemInputStamped>(serialize<SystemInputStamped>(inputStamped)));

    StateInputPairStamped stateInputStamped(State(State::T::Constant(3, 1, 1.0), pose),
                                            SystemInput(SystemInput::T::Constant(3, 1, 1.0)),
                                            Time(100L));
    EXPECT_EQ(stateInputStamped, deserialize<StateInputPairStamped>(serialize<StateInputPairStamped>(stateInputStamped)));

    OccupancyGrid grid(pose, 0.1, OccupancyGrid::Matrix::Constant(100,100, OccupancyValue::UNKNOWN));
    EXPECT_EQ(grid, deserialize<OccupancyGrid>(serialize<OccupancyGrid>(grid)));

    OccupancyGridStamped gridStamped(pose, 0.1, OccupancyGrid::Matrix::Constant(100,100, OccupancyValue::UNKNOWN), Time(100L));
    EXPECT_EQ(gridStamped, deserialize<OccupancyGridStamped>(serialize<OccupancyGridStamped>(gridStamped)));

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(planner_interfaces_TESTSUITE, DISABLED_Profiling) {
  sm::timing::Timing::print(cout, sm::timing::SORT_BY_MEAN);
}
