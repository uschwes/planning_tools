/*
 * StatePy.cpp
 *
 *  Created on: Feb 19, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <boost/python/args.hpp>

#include <numpy_eigen/boost_python_headers.hpp>
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include <planner_interfaces/OccupancyGrid.hpp>

#include <planner_interfaces_python/SupportPy.hpp>
#include <planner_interfaces_python/PythonPickleSupport.hpp>

#include <sm/eigen/NumericalDiff.hpp>

using namespace std;
using namespace boost::python;
using namespace planning2d;

template <typename T>
void setMatrix(Map<T>& map, const typename Map<T>::Matrix& m) {
  map.matrix() = m;
}

template <typename T>
Eigen::Vector2d gradientInterpolatedBilinearWrapper(const Map<T>& map,
                                                    const typename Map<T>::InterpolatedIndex& idxp,
                                                    const planning2d::details::MapExtrapolationMethod extrapolation = planning2d::details::MapExtrapolationMethod::NONE) {
  Eigen::Vector2d grad;
  map.gradientInterpolatedBilinear(idxp, grad.x(), grad.y(), extrapolation);
  return grad;
}

template <typename T>
Eigen::Vector2d gradientInterpolatedBicubicWrapper(const Map<T>& map,
                                                   const typename Map<T>::InterpolatedIndex& idxp,
                                                   const planning2d::details::MapExtrapolationMethod extrapolation = planning2d::details::MapExtrapolationMethod::NONE) {
  Eigen::Vector2d grad;
  map.gradientInterpolatedBicubic(idxp, grad.x(), grad.y(), extrapolation);
  return grad;
}

template <typename T>
const Eigen::Matrix3d& getTransformationWrapper(const Map<T>& map) {
  return map.getTransformation().matrix();
}

template <typename T>
typename Map<T>::Matrix getBlock(const Map<T>& map, const typename Map<T>::Index& index, const typename Map<T>::Size2d& size)
{
  return map.block(index, size);
}

template <typename T>
void setBlock(Map<T>& map, const typename Map<T>::Index& index, const typename Map<T>::Size2d& size, const T val)
{
  map.block(index, size) = Map<T>::Matrix::Constant(size.y(), size.x(), val);
}

template <typename T>
void addTo(const Map<T>& map, Map<T>& other) {
  map.mergeInto(other, [](const T& lhs, const T& rhs) { return lhs + rhs; } );
}

template <>
void addTo<OccupancyValue>(const Map<OccupancyValue>& map, Map<OccupancyValue>& other) {
  map.mergeInto(other, [](const OccupancyValue& lhs, const OccupancyValue& rhs) { return (lhs == OccupancyValue::OCCUPIED || rhs == OccupancyValue::OCCUPIED) ? OccupancyValue::OCCUPIED : lhs; } );
}

template <typename T> void exportMap(const std::string& name, const std::string& type, const bool registerNumpyEigenConverter = true) {
  typedef Map<T> MapT;
  typedef typename Map<T>::Index Index;
  typedef typename Index::Scalar IndexScalar;
  class_<MapT, typename MapT::Ptr>(name.c_str(), init<>("Default constructor"))
      .def(init<const Pose2d&, const double>((string() + name.c_str() + "(Pose2d origin, double resolution): Constructor").c_str()))
      .def(init<const Pose2d&, const double, const typename MapT::Matrix&>((string() + name.c_str() + "(Pose2d origin, double resolution, Array data): Constructor").c_str()))
      .def(init<const Pose2d&, const double, const typename MapT::Size2d&>((string() + name.c_str() + "(Pose2d origin, double resolution, Size2d size): Constructor").c_str()))
      .def(init<const Pose2d&, const double, const typename MapT::Size2d&, const T&>((string() + name.c_str() + "(Pose2d origin, double resolution, Size2d size, value): Constructor").c_str()))
      .def("initialize", &MapT::initialize, "Initializes the map with metric information. The actual map data will not be touched by this method.")
      .add_property("origin", make_function( (Pose2d& (MapT::*) (void)) &MapT::getOrigin, return_internal_reference<>() ), &MapT::setOrigin, "2d pose of the map's origin in the map's parent frame (lower left corner)")
      .add_property("transformation", make_function(&getTransformationWrapper<T>, return_value_policy<copy_const_reference>()), "Transformation from parent to grid frame")
      .add_property("resolution", &MapT::resolution, &MapT::setResolution, "Size of one cell (pixel) in the map [m]")
      .add_property("reciprocalResolution", &MapT::reciprocalResolution, "Reciprocal value of resolution")
      .add_property("sizeInCells", &MapT::sizeInCells, "The map's 2D size in cells")
      .add_property("sizeInCellsX", &MapT::sizeInCellsX, "The map's x size in cells")
      .add_property("sizeInCellsY", &MapT::sizeInCellsY, "The map's y size in cells")
      .add_property("sizeInMeters", &MapT::sizeInMeters, "The map's 2D size in meters")
      .add_property("sizeInMetersX", &MapT::sizeInMetersX, "The map's x size in meters")
      .add_property("sizeInMetersY", &MapT::sizeInMetersY, "The map's y size in meters")
      .add_property("matrix", make_function( (const typename MapT::Matrix& (MapT::*) (void) const) &MapT::matrix, return_value_policy<copy_const_reference>()), &setMatrix<T>, "The underlying matrix holding the data")
      .add_property("empty", &MapT::empty, "Whether the data in the matrix is empty")
      .def("getAtIxIy", make_function( (const T& (MapT::*) (IndexScalar, IndexScalar) const) &MapT::at, return_value_policy<copy_const_reference>()),
           (type + " value = getAtIxIy(MapT::Index::Scalar ix, MapT::Index::Scalar iy): Returns the data at index ix/iy").c_str())
      .def("setAtIxIy",  (void (MapT::*) (IndexScalar, IndexScalar, const T&)) &MapT::set,
           ("None = setAtIxIy(MapT::Index::Scalar ix, MapT::Index::Scalar iy, " + type + " value): Sets the data at index ix/iy").c_str())
      .def("getAtIndex", make_function( (const T& (MapT::*) (const Index&) const) &MapT::at, return_value_policy<copy_const_reference>()),
           (type + " value = getAtIndex(Index idx): Returns the data at index idx").c_str())
      .def("getAtInterpolatedBilinear", &MapT::atInterpolatedBilinear, (boost::python::arg("extrapolation") = planning2d::details::MapExtrapolationMethod::NONE),
           (type + " value = getAtInterpolatedBilinear(InterpolatedIndex idxp, [MapExtrapolationMethod extrapolation=NONE]): Returns the bilinear interpolation of the data at interpolated index idxp").c_str())
      .def("getAtInterpolatedBicubic", &MapT::atInterpolatedBicubic, (boost::python::arg("extrapolation") = planning2d::details::MapExtrapolationMethod::NONE),
           (type + " value = getAtInterpolatedBicubic(InterpolatedIndex idxp, [MapExtrapolationMethod extrapolation=NONE]): Returns the bicubic interpolation of the data at interpolated index idxp").c_str())
      .def("getAtInterpolatedBicubicPchip", &MapT::atInterpolatedBicubicPchip, (boost::python::arg("extrapolation") = planning2d::details::MapExtrapolationMethod::NONE),
           (type + " value = getAtInterpolatedBicubicPchip(InterpolatedIndex idxp, [MapExtrapolationMethod extrapolation=NONE]): Returns the bicubic PCHIP interpolation of the data at interpolated index idxp").c_str())
      .def("setAtIndex", (void (MapT::*) (const Index&, const T&)) &MapT::set,
           ("None = setAtIndex(Index idx, " + type + " value): Sets the data at index idx").c_str())
      .def("getAtPosition", make_function( (const T& (MapT::*) (const Position2d&) const) &MapT::at, return_value_policy<copy_const_reference>()),
           (type + " value = getAtPosition(Position2d pos): Returns the data at position pos").c_str())
      .def("setAtPosition", (void (MapT::*) (const Position2d&, const T&)) &MapT::set,
           ("None = setAtPosition(Position2d pos, " + type + " value): Sets the data at position pos").c_str())
      .def("isInsideMapIxIy", (bool (MapT::*)(IndexScalar, IndexScalar) const)&MapT::isInsideMap,
           "bool = isInsideMapIxIy(MapT::Index::Scalar ix, MapT::Index::Scalar ix): Returns whether an index points into the matrix or is out of bounds")
      .def("isInsideMapIndex", (bool (MapT::*)(const Index&) const)&MapT::isInsideMap,
           "bool = isInsideMapIndex(Index idx): Returns whether an index points into the matrix or is out of bounds")
      .def("isInsideMapPosition", (bool (MapT::*)(const Position2d&) const)&MapT::isInsideMap,
           "bool = isInsideMapPosition(Position2d pos): Returns whether a position points into the matrix or is out of bounds")
      .def("isInsideMapInterpolatedIndex", (bool (MapT::*)(const typename MapT::InterpolatedIndex&) const)&MapT::isInterpolatedIndexInsideMap,
           "bool = isInsideMapInterpolatedIndex(InterpolatedIndex idxp): Returns whether an interpolated index points into the matrix or is out of bounds")
      .def("isMapBoundary", &MapT::isMapBoundary,
           "bool = isMapBoundary(Index idx): Returns true if the index lies on the map's boundary.")
      .def("toIndex", (Index (MapT::*)(const Position2d&) const)&MapT::toIndex,
           "Index idx = toIndex(Position2d pos): Converts coordinates expressed in the parent frame of the map to an index")
      .def("toInterpolatedIndex", &MapT::toInterpolatedIndex,
           "InterpolatedIndex idxp = toInterpolatedIndex(Position2d pos): Converts coordinates expressed in the parent frame of the map to an interpolated index")
      .def("toPosition", (Position2d (MapT::*) (const typename MapT::InterpolatedIndex&) const)&MapT::toPosition,
           "Position2d pos = toPosition(InterpolatedIndex idx): Converts an interpolated index to a position expressed in the parent frame")
      .def("toPosition", (Position2d (MapT::*) (const typename MapT::Index&) const)&MapT::toPosition,
           "Position2d pos = toPosition(Index idx): Converts an index to a position expressed in the parent frame")
      .def("projectToNearestIndex", &MapT::projectToNearestIndex)
      .def("alignedBoxIndex", &MapT::alignedBoxIndex,
           "An aligned box for the occupancy grid extents in index coordinates")
      .def("alignedBoxPosition", &MapT::alignedBoxPosition,
           "Returns an aligned box for the occupancy grid extents in position coordinates")
      .def("block", &getBlock<T>,
           "Matrix mat = block(Index min, Size2d size): Returns a block of the underlying Eigen matrix")
      .def("setBlock", &setBlock<T>,
           ("None = setBlock(Index min, Size2d size, " + type + " val): Sets a block of the underlying Eigen matrix to val").c_str())
//      .def("getRegion", (typename MapT::Matrix (MapT::*) (const Eigen::AlignedBox2i&, const typename MapT::ExtrapolationMethod) const) &MapT::getRegion,
//           (boost::python::arg("extrapolation") = MapT::ExtrapolationMethod::NONE,
//           "Matrix mat = getRegion(AlignedBox2i region, [MapExtrapolationMethod extrapolation=NONE]): Extracts a region from the data"))
      .def("gradientInterpolatedBilinear", &gradientInterpolatedBilinearWrapper<T>,
           (boost::python::arg("extrapolation") = MapT::ExtrapolationMethod::NONE),
           "tuple(dx,dy) = gradientInterpolatedBilinear(InterpolatedIndex idxp, [MapExtrapolationMethod extrapolation=NONE]): "
           "Computes the gradient of the bilinear interpolation of the grid data at index idxp")
      .def("gradientInterpolatedBicubic", &gradientInterpolatedBicubicWrapper<T>,
           (boost::python::arg("extrapolation") = MapT::ExtrapolationMethod::NONE),
            "tuple(dx,dy) = gradientInterpolatedBicubic(InterpolatedIndex idxp, [MapExtrapolationMethod extrapolation=NONE]): "
            "Computes the gradient of the bicubic interpolation of the grid data at index idxp")
      .def("shift", &MapT::shift,
           "Shifts the map to a new location")
      .def("addTo", &addTo<T>,
           (string() + "None = addTo(" + name + " other): Adds the current map into the other map").c_str())
      .def("__eq__", &MapT::operator==)
      .def("__ne__", &MapT::operator!=)
      .def("__getitem__", boost::python::detail::make_function_aux([](MapT& map, const boost::python::tuple& index){
                                  return map(boost::python::extract<typename MapT::Index::Scalar>(index[0]), boost::python::extract<typename MapT::Index::Scalar>(index[1]));},
                          boost::python::default_call_policies(), boost::mpl::vector<T, MapT&, const boost::python::tuple&>()))
      .def("__getitem__", make_function((const T& (MapT::*) (const Position2d&) const)&MapT::operator(), return_value_policy<copy_const_reference>()))
      .def("__getitem__", make_function((const T& (MapT::*) (const typename MapT::Index&) const)&MapT::operator(), return_value_policy<copy_const_reference>()))
      .def("__setitem__", boost::python::detail::make_function_aux([](MapT& map, const boost::python::tuple& index, const T& val){
                                  map(boost::python::extract<typename MapT::Index::Scalar>(index[0]), boost::python::extract<typename MapT::Index::Scalar>(index[1])) = val;},
                          boost::python::default_call_policies(), boost::mpl::vector<void, MapT&, const boost::python::tuple&, const T&>()))
      .def("__setitem__", boost::python::detail::make_function_aux([](MapT& map, const Position2d& pos, const T& val){ map(pos) = val;},
                          boost::python::default_call_policies(), boost::mpl::vector<void, MapT&, const Position2d&, const T&>()))
      .def("__setitem__", boost::python::detail::make_function_aux([](MapT& map, const typename MapT::Index& index, const T& val){ map(index) = val;},
                          boost::python::default_call_policies(), boost::mpl::vector<void, MapT&, const typename MapT::Index&, const T&>()))
      .def_pickle(BoostSerializationBinary_pickle_suite<MapT>())
  ;
  register_ptr_to_python<typename MapT::ConstPtr>();
  implicitly_convertible<typename MapT::Ptr,typename MapT::ConstPtr >();

  if (registerNumpyEigenConverter)
    NumpyEigenConverter<typename MapT::Matrix>::register_converter();
}


template <typename T>
void exportAlignedBox2d(const std::string& name) {

  typedef Eigen::AlignedBox<T, 2> Box;
  typedef Eigen::Matrix<T, 2, 1> Vector;

  class_<Box>(name.c_str(), init<>())
    .def(init<Vector,Vector>())
    .add_property("min", make_function((const typename Box::VectorType& (Box::*)(void) const)&Box::min, return_value_policy<copy_const_reference>()),
                  boost::python::detail::make_function_aux([](Box& b, const Vector& min){ b.min() = min;}, boost::python::default_call_policies(), boost::mpl::vector<void,Box,Vector>()))
    .add_property("max", make_function((const typename Box::VectorType& (Box::*)(void) const)&Box::max, return_value_policy<copy_const_reference>()),
                  boost::python::detail::make_function_aux([](Box& b, const Vector& max){ b.max() = max;}, boost::python::default_call_policies(), boost::mpl::vector<void,Box,Vector>()))
    .add_property("sizes", boost::python::detail::make_function_aux([](const Box& b){return b.sizes().eval();}, boost::python::default_call_policies(), boost::mpl::vector<Vector,Box>()))
    .add_property("center", boost::python::detail::make_function_aux([](const Box& b){return b.center().eval();}, boost::python::default_call_policies(), boost::mpl::vector<Vector,Box>()))
    .def("intersection", &Box::intersection)
    .def("extend", make_function((Box& (Box::*)(const Box&))&Box::extend, return_internal_reference<>()))
    .def("extend", boost::python::detail::make_function_aux([](Box& b, const Vector& v){return b.extend(v);}, return_internal_reference<>(), boost::mpl::vector<Box&,Box,Vector>()))
    .def("contains", (bool (Box::*)(const Box&) const)&Box::contains)
    .def("contains", boost::python::detail::make_function_aux([](const Box& b, const Vector& v){return b.contains(v);}, boost::python::default_call_policies(), boost::mpl::vector<bool,Box,Vector>()))
    .def("merged", &Box::merged)
    .def("isEmpty", &Box::isEmpty)
    .def("volume", &Box::volume)
    .def("diagonal", &Box::diagonal)
    .def("__str__", &toString<Box>)
  ;
}


template<> struct TypeToNumPy<OccupancyValue> {
  enum { NpyType = NPY_UBYTE };
  static const char * npyString() { return "NPY_UBYTE"; }
  static const char * typeString() { return "occ"; }
  static bool canConvert(int type) { return type == NPY_UINT || type == NPY_UINT8 || type == NPY_USHORT ||
                                            type == NPY_UINT16 || type == NPY_UINT32 || type == NPY_UINT64 ||
                                            type == NPY_ULONG || type == NPY_ULONGLONG; }
};
template<> struct TypeToNumPy<bool> {
  enum { NpyType = NPY_BOOL };
  static const char * npyString() { return "NPY_BOOL"; }
  static const char * typeString() { return "Bool"; }
  static bool canConvert(int type) { return type == NPY_UINT || type == NPY_UINT8 || type == NPY_USHORT ||
                                            type == NPY_UINT16 || type == NPY_UINT32 || type == NPY_UINT64 ||
                                            type == NPY_ULONG || type == NPY_ULONGLONG ||
                                            type == NPY_INT || type == NPY_INT8 || type == NPY_SHORT ||
                                            type == NPY_INT16 || type == NPY_INT32 || type == NPY_INT64 ||
                                            type == NPY_LONG || type == NPY_LONGLONG ||
                                            type == NPY_BOOL; }
};

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(OccupancyGrid_inflateWithKernel_overloads, inflateWithKernel, 1, 2);

void exportOccupancyGrid() {

  import_array(); // important!

  exportAlignedBox2d<int>("AlignedBox2dInt");
  exportAlignedBox2d<int64_t>("AlignedBox2dInt64");
  exportAlignedBox2d<double>("AlignedBox2dDouble");

  enum_<planning2d::details::MapExtrapolationMethod>("MapExtrapolationMethod")
    .value("NONE", planning2d::details::MapExtrapolationMethod::NONE)
    .value("CONSTANT", planning2d::details::MapExtrapolationMethod::CONSTANT)
    .value("LINEAR", planning2d::details::MapExtrapolationMethod::LINEAR)
  ;

  exportMap<double>("MapDouble", "double");
  exportMap<float>("MapFloat", "float");
  exportMap<uint8_t>("MapUint8", "uint8");
  exportMap<uint64_t>("MapUint64", "uint64");
  exportMap<int32_t>("MapInt32", "int32");
  exportMap<OccupancyValue>("MapOccupancyValue", "OccupancyValue");
  exportMap<bool>("MapBool", "bool", false);

  enum_<OccupancyValue>("OccupancyValue")
    .value("FREE", OccupancyValue::FREE)
    .value("UNKNOWN", OccupancyValue::UNKNOWN)
    .value("OCCUPIED", OccupancyValue::OCCUPIED)
  ;

  // Note: use np.array(..., dtype=np.ubyte) or any other unsigned integer type to pass data to the grid
  typedef Map<OccupancyValue> OccupancyMap;
  class_<OccupancyGrid, OccupancyGrid::Ptr, bases<OccupancyMap> >("OccupancyGrid", init<>("Default constructor"))
      .def(init<const Pose2d&, const double>("OccupancyGrid(Pose2d origin, double resolution): Constructor"))
      .def(init<const Pose2d&, const double, const typename OccupancyMap::Matrix&>("OccupancyGrid(Pose2d origin, double resolution, Array data): Constructor"))
      .def(init<const Pose2d&, const double, const typename OccupancyMap::Size2d&>("OccupancyGrid(Pose2d origin, double resolution, Size2d size): Constructor"))
      .def(init<const Pose2d&, const double, const typename OccupancyMap::Size2d&, const OccupancyValue&>("OccupancyGrid(Pose2d origin, double resolution, Size2d size, OccupancyValue value): Constructor"))
      .def("isOccupied", (bool (OccupancyGrid::*)(const OccupancyMap::Index& idx) const)&OccupancyGrid::isOccupied,
           "bool = isOccupied(Index pos): Returns occupancy state of the grid at index idx. Does not check for valid bounds.")
      .def("isOccupied", (bool (OccupancyGrid::*)(const Position2d& idx) const)&OccupancyGrid::isOccupied,
           "bool = isOccupied(Position2d idx): Returns occupancy state of the grid at position pos. Does not check for valid bounds.")
      .def("isObstacleBoundary", &OccupancyGrid::isObstacleBoundary,
           "bool = isObstacleBoundary(Index idx): Returns true if the index is an obstacle boundary cell. "
           "A cell is an obstacle boundary cell if it is occupied and at least one of its 8 neighbors is not.")
      .def("invert", (void (OccupancyGrid::*)(void) const)&OccupancyGrid::invert)
      .def("computeInflationKernel", &OccupancyGrid::computeInflationKernel,
           "Array computeInflationKernel(double inflationRadiusMeters): Computes an inflation kernel that can be passed to inflateWithKernel(double inflationRadiusMeters) later.")
      .def("inflateWithKernel", &OccupancyGrid::inflateWithKernel, OccupancyGrid_inflateWithKernel_overloads(
           "None inflateWithKernel(Array inflationKernel): Dilates occupied cells of map by inflationRadiusMeters by moving a kernel over the matrix."))
      .def_pickle(BoostSerializationBinary_pickle_suite<OccupancyGrid>())
  ;
  register_ptr_to_python<OccupancyGrid::ConstPtr>();
  implicitly_convertible<OccupancyGrid::Ptr,OccupancyGrid::ConstPtr >();
  NumpyEigenConverter< Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> >::register_converter();
  NumpyEigenConverter< Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >::register_converter();

  class_<OccupancyGridStamped, OccupancyGridStamped::Ptr, bases<OccupancyGrid, StampedType> >("OccupancyGridStamped", init<>("Default constructor"))
    .def(init<const Pose2d&, const double>("OccupancyGrid(Pose2d origin, double resolution): Constructor"))
    .def(init<const Pose2d&, const double, const typename OccupancyMap::Matrix&>("OccupancyGridStamped(Pose2d origin, double resolution, Array data): Constructor"))
    .def(init<const Pose2d&, const double, const typename OccupancyMap::Size2d&>("OccupancyGridStamped(Pose2d origin, double resolution, Size2d size): Constructor"))
    .def(init<const Pose2d&, const double, const typename OccupancyMap::Size2d&, const OccupancyValue&>("OccupancyGridStamped(Pose2d origin, double resolution, Size2d size, OccupancyValue value): Constructor"))
    .def(init<const Time&>("OccupancyGridStamped(Time stamp): Constructor"))
    .def(init<const OccupancyGrid&, const Time&>("OccupancyGrid(OccupancyGrid grid, Time stamp): Constructor"))
    .def(init<const Pose2d&, const double, const Time&>("OccupancyGridStamped(Pose2d origin, double resolution): Constructor"))
    .def(init<const Pose2d&, const double, const typename OccupancyMap::Matrix&, const Time&>("OccupancyGridStamped(Pose2d origin, double resolution, Array data, Time stamp): Constructor"))
    .def(init<const Pose2d&, const double, const typename OccupancyMap::Size2d&, const Time&>("OccupancyGridStamped(Pose2d origin, double resolution, Size2d size, Time stamp): Constructor"))
    .def(init<const Pose2d&, const double, const typename OccupancyMap::Size2d&, const OccupancyValue&, const Time&>("OccupancyGridStamped(Pose2d origin, double resolution, Size2d size, OccupancyValue value, Time stamp): Constructor"))
    .def("__eq__", &OccupancyGridStamped::operator==)
    .def("__ne__", &OccupancyGridStamped::operator!=)
    .def_pickle(BoostSerializationBinary_pickle_suite<OccupancyGridStamped>())
  ;
  register_ptr_to_python<OccupancyGridStamped::ConstPtr>();
  implicitly_convertible<OccupancyGridStamped::Ptr,OccupancyGridStamped::ConstPtr >();

} /* void exportOccupancyGrid() */
