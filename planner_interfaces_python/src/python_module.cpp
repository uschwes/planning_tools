#include <boost/python.hpp>
#include <sstream>

using namespace std;
using namespace boost::python;

void exportEigen21DontAlign();
void exportReturnCode();
void exportTime();
void exportStampedType();
void exportPosition2d();
void exportPose2d();
void exportState();
void exportSystemInput();
void exportAgent();
void exportShape();
void exportOccupancyGrid();
void exportPlannerInterface();
void exportTargetPlannerInterface();
void exportTrajectory();
void exportArchive();
void exportMathSupport();

BOOST_PYTHON_MODULE(libplanner_interfaces_python) {
  exportEigen21DontAlign();
  exportReturnCode();
  exportTime();
  exportStampedType();
  exportPosition2d();
  exportPose2d();
  exportState();
  exportSystemInput();
  exportAgent();
  exportShape();
  exportOccupancyGrid();
  exportPlannerInterface();
  exportTargetPlannerInterface();
  exportTrajectory();
  exportArchive();
  exportMathSupport();
}
