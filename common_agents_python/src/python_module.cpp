#include <boost/python.hpp>
#include <sstream>

using namespace std;
using namespace boost::python;

void exportHolonomicAgent();
void exportHolonomicSystemInput();
void exportHolonomicState();
void exportDifferentialDriveSystemInput();
void exportDifferentialDriveState();
void exportDifferentialDriveAgent();
void exportTypeRegistration();

BOOST_PYTHON_MODULE(libcommon_agents_python) {
  exportHolonomicState();
  exportHolonomicSystemInput();
  exportHolonomicAgent();
  exportDifferentialDriveSystemInput();
  exportDifferentialDriveState();
  exportDifferentialDriveAgent();
  exportTypeRegistration();
}
