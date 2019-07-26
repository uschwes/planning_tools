#include <boost/python.hpp>
#include <sstream>

using namespace std;
using namespace boost::python;

void exportDijkstra();
void exportMapChangeDetection();
void exportVoronoi();
void exportDistanceTransform();

BOOST_PYTHON_MODULE(libplanner_algorithms_python)
{
  exportDijkstra();
  exportMapChangeDetection();
  exportVoronoi();
  exportDistanceTransform();
}
