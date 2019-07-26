/*
 * ProfileEvaluateJacobiansExpressions.cpp.cpp
 *
 *  Created on: 11.03.2016
 *      Author: Ulrich Schwesinger
 */

// standard includes
#include <vector>
#include <string>

// boost includes
#include <boost/program_options.hpp>

// Schweizer Messer includes
#include <sm/logging.hpp>
#include <sm/timing/Timer.hpp>

// aslam backend includes
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/util/ProblemManager.hpp>

// self includes
#include "../include/probabilistic_planner/features/FeaturePairwiseIntegratedDistance.hpp"
#include "../include/probabilistic_planner/features/FeaturePairwiseIntegratedInverseDistance.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedVelocity.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedAcceleration.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedRotationRate.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonObservationPosition2d.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedStaticObstacleDistance.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedBarrierVelocity.hpp"
#include "Support.hpp"

using namespace std;
using namespace planning2d;
using namespace aslam::backend;
using namespace prob_planner;

int main(int argc, char** argv)
{
  try
  {
    string verbosity = "Info";
    vector<string> enableNamedStreams;
    bool disableDefaultStream = false;
    size_t nIterations = 100, nThreads = 1, nAgents = 2;
    double timeHorizonSec = 5.0;
    bool useSparseJacobianContainer = false;
    bool useGridMeasurement = false;

    namespace po = boost::program_options;
    po::options_description desc("local_planner options");
    desc.add_options()
      ("help", "Produce help message")
      ("verbosity,v", po::value(&verbosity)->default_value(verbosity), "Verbosity string")
      ("disable-default-stream", po::bool_switch(&disableDefaultStream), "Disable default logging stream")
      ("enable-named-streams", po::value< vector<string> >(&enableNamedStreams)->multitoken(), "Enable these named logging streams")
      ("num-iterations", po::value(&nIterations)->default_value(nIterations), "Number of iterations")
      ("num-threads", po::value(&nThreads)->default_value(nThreads), "Number of threads")
      ("num-agents", po::value(&nAgents)->default_value(nAgents), "Number of agents")
      ("time-horizon-sec", po::value(&timeHorizonSec)->default_value(timeHorizonSec), "Time horizon for optimization")
      ("use-sparse-jacobian-container", po::bool_switch(&useSparseJacobianContainer), "Use dense/sparse Jacobian container")
      ("use-grid-measurement", po::bool_switch(&useGridMeasurement), "Add a grid measurement")
    ;
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return EXIT_SUCCESS;
    }
    po::notify(vm);
    sm::logging::setLevel(sm::logging::levels::fromString(verbosity));
    for (auto& stream : enableNamedStreams)
      sm::logging::enableNamedStream(stream);
    if (disableDefaultStream)
      sm::logging::disableNamedStream("sm");


    ContinuousScene scene;
    const Time t0(0.0);
    const Duration timeHorizon(timeHorizonSec);
    const OptAgentType agentType = OptAgentType::PEDESTRIAN;
    populateScene(scene, false, nAgents, t0, timeHorizon, agentType);

    // add grid measurement
    if (useGridMeasurement) {
      auto snPtr = boost::make_shared<SceneSnapshot>(scene.getMinTime());
      OccupancyGrid grid(Pose2d(0., 0., 0.), 0.1, OccupancyGrid::Size2d(400,400), OccupancyValue::FREE);
      snPtr->setOccupancyGrid(grid);
      scene.addObservation(snPtr);
    }
    // Create features
    vector<RawFeature::Ptr> features;
    features.emplace_back( new FeatureSingletonIntegratedVelocity(agentType, 1.0) );
    features.emplace_back( new FeatureSingletonIntegratedAcceleration(agentType, 1.0) );
    features.emplace_back( new FeaturePairwiseIntegratedInverseDistance(agentType, 1.0) );
    features.emplace_back( new FeatureSingletonIntegratedStaticObstacleDistance(agentType, 1.0) );
    features.emplace_back( new FeatureSingletonIntegratedBarrierVelocity(agentType, 1.0) );
//    features.emplace_back( new FeatureSingletonIntegratedRotationRate(agentType, 1.0) );

    // add all error terms from the feature to the optimization problem
    boost::shared_ptr<OptimizationProblem> negLogDensity(new OptimizationProblem());
    scene.addDesignVariables(*negLogDensity, true);
    for (auto feature : features)
      feature->addErrorTerms(scene, *negLogDensity);

    ProblemManager pm;
    pm.setProblem(negLogDensity);
    pm.initialize();

    SM_INFO_STREAM("Timing Jacobians/error via expressions...");
    SM_INFO_STREAM("Optimization problem has " << negLogDensity->numTotalErrorTerms() << " error term(s) and " <<
                   negLogDensity->numDesignVariables() << " design variable(s)");

    {
      aslam::backend::RowVectorType gradient;
      sm::timing::Timer timer("FeatureEvaluation: Jacobians", false);
      for (size_t i=0; i<nIterations; ++i) {
        pm.computeGradient(gradient, nThreads, false, false, !useSparseJacobianContainer);
        pm.applyStateUpdate(aslam::backend::ColumnVectorType::Random(pm.numOptParameters(), 1));
      }
    }

    {
      sm::timing::Timer timer("FeatureEvaluation: Error", false);
      for (size_t i=0; i<nIterations; ++i) {
        pm.evaluateError(nThreads);
        pm.applyStateUpdate(aslam::backend::ColumnVectorType::Random(pm.numOptParameters(), 1));
      }
    }

    sm::timing::Timing::print(cout, sm::timing::SORT_BY_TOTAL);

    return EXIT_SUCCESS;

  }
  catch (exception& e)
  {
    SM_FATAL_STREAM(e.what());
    return EXIT_FAILURE;
  }

}
