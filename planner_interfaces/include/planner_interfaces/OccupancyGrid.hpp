/*
 * OccupancyGrid.hpp
 *
 *  Created on: Oct 21, 2014
 *      Author: sculrich
 */

#ifndef PLANNING2D_OCCUPANCYGRID_HPP_
#define PLANNING2D_OCCUPANCYGRID_HPP_

// Standard includes
#include <iostream>
#include <stdint.h>

// Boost
#include <boost/optional.hpp>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// self includes
#include "Support.hpp"
#include "Position2d.hpp"
#include "Pose2d.hpp"
#include "StampedType.hpp"
#include "MathSupport.hpp"

namespace planning2d {

namespace details {
  enum MapExtrapolationMethod { NONE, CONSTANT, LINEAR };
}

namespace grid {

typedef Point2d<int64_t> Index; //! An index into a map
typedef Position2d InterpolatedIndex; //! An interpolated index into a map
typedef Point2d<std::size_t> Size2d; //! Size in x- and y-direction (x: right/columns, y: up/rows)
typedef Eigen::AlignedBox<Index::Scalar, 2> AlignedBoxIndex; //! An aligned box in index coordinates
typedef Eigen::AlignedBox<double, 2> AlignedBoxPosition; //! An aligned box in position coordinates
typedef ::planning2d::details::MapExtrapolationMethod ExtrapolationMethod;
typedef math::InterpolationMethod InterpolationMethod;

namespace details {

template <typename MAP, int EXTRAPOLATION_METHOD>
struct Extrapolator {
  static typename MAP::Scalar extrapolate(const MAP& map, const Index idx);
};

template <typename MAP>
struct Extrapolator<MAP, planning2d::details::MapExtrapolationMethod::CONSTANT> {
  static typename MAP::Scalar extrapolate(const MAP& map, const Index idx);
};

template <typename MAP>
struct Extrapolator<MAP, planning2d::details::MapExtrapolationMethod::LINEAR> {
  static typename MAP::Scalar extrapolate(const MAP& map, const Index idx);
};

template<int INTERPOLATION_METHOD> struct InterpolationShiftTraits { static constexpr int LeftShift = 1; };
template<> struct InterpolationShiftTraits<grid::InterpolationMethod::LINEAR> { static constexpr int LeftShift = 0; };

} /* namespace details */

} /* namespace map */

/**
 * @brief Map class
 * A map is matrix with metric information attached. The map follows the convention that
 * x-direction is in row direction and y-direction is in column direction. Keep that in mind
 * when accessing the map with a 2D \ref Index.
 * @tparam Data type of the cells
 * @tparam StorageOrder Use row-major storage order by default to facilitate interfacing with opencv.
 *                      Note that it is potentially more efficient to iterate over x-values in the inner loop
 *                      of two for loops first due to the memory layout.
 *
 */
template <class T, int StorageOrder = Eigen::RowMajor>
class Map {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(Map<T>);
  typedef T Scalar;
  typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, StorageOrder> Matrix;
  typedef T* iterator;
  typedef T const* const_iterator;

  // typedefs kept for downward compatibility reasons
  typedef grid::Index Index; //! An index into the matrix
  typedef grid::InterpolatedIndex InterpolatedIndex; //! An interpolated index into the matrix
  typedef grid::Size2d Size2d; //! Size in x- and y-direction (x: right/columns, y: up/rows)
  typedef grid::AlignedBoxIndex AlignedBoxIndex; //! An aligned box in index coordinates
  typedef grid::AlignedBoxPosition AlignedBoxPosition; //! An aligned box in position coordinates
  typedef grid::ExtrapolationMethod ExtrapolationMethod;
  typedef grid::InterpolationMethod InterpolationMethod;

 public:

  //! Default constructor
  inline Map();
  //! Constructs Map with metric information. The actual map data will still be uninitialized afterwards
  //! and has to be set with \ref matrix()
  inline Map(const Pose2d& origin, const double resolutionMeters);
  //! Full constructor
  inline Map(const Pose2d& origin, const double resolutionMeters, const Map::Matrix& data);
  //! Constructor that initializes matrix size. Cell values are left uninitialized.
  inline Map(const Pose2d& origin, const double resolutionMeters, const Size2d& size);
  //! Constructor with all cells set to \p val
  inline Map(const Pose2d& origin, const double resolutionMeters, const Size2d& size, const T& val);

  //! Initializes the map with metric information. The actual map data will not be touched by this method.
  inline void initialize(const Pose2d& origin, const double resolutionMeters);

  // setters/getters

  //! Returns const reference to origin of the map
  inline const Pose2d& getOrigin() const;
  //! Returns mutable reference to origin of the map
  inline Pose2d& getOrigin();
  //! Sets the origin of the map
  inline void setOrigin(const Pose2d& pose);
  //! Returns const reference to the transformation from parent to grid frame
  inline const Eigen::Isometry2d& getTransformation() const;
  //! Returns const reference to the map's resolution
  inline double resolution() const;
  //! Returns const reference to the map's reciprocal resolution
  inline double reciprocalResolution() const;
  //! Sets the resolution (size of one cell) of the map.
  inline void setResolution(double resolution);
  //! Returns the map's 2D size in cells
  inline Size2d sizeInCells() const;
  //! Returns the map's x size in cells
  inline std::size_t sizeInCellsX() const;
  //! Returns the map's y size in cells
  inline std::size_t sizeInCellsY() const;
  //! Returns the map's 2D size in meters
  inline Point2d<double> sizeInMeters() const;
  //! Returns the map's x size in meters
  inline double sizeInMetersX() const;
  //! Returns the map's y size in meters
  inline double sizeInMetersY() const;
  //! Returns const reference to the underlying Eigen matrix
  inline const Matrix& matrix() const;
  //! Returns mutable reference to the underlying Eigen matrix
  inline Matrix& matrix();
  //! Returns whether the data in the matrix is empty
  inline bool empty() const;
  //! Cast data to specific type
  template <typename TOut>
  inline Map<TOut,StorageOrder> cast() const;

  //! Const getter, does check for valid bounds (assertion)
  inline const T& at(const Index& idx) const;
  //! Mutable getter, does check for valid bounds (assertion)
  inline T& at(const Index& idx);
  //! Const getter, does check for valid bounds (assertion)
  inline const T& at(Index::Scalar ix, Index::Scalar iy) const;
  //! Mutable getter, does check for valid bounds (assertion)
  inline T& at(Index::Scalar ix, Index::Scalar iy);
  //! Const getter, does check for valid bounds (assertion)
  inline const T& at(const Position2d& idx) const;
  //! Mutable getter, does check for valid bounds (assertion)
  inline T& at(const Position2d& idx);

  //! Interpolated getter
  template <int INTERPOLATION_METHOD, int EXTRAPOLATION_METHOD>
  inline double atInterpolated(const InterpolatedIndex& idxp) const;
  //! Partially templated interpolated getter
  template <int EXTRAPOLATION_METHOD>
  inline double atInterpolated(const InterpolatedIndex& idxp, const InterpolationMethod interpolation) const;
  //! Partially templated interpolated getter
  template <int INTERPOLATION_METHOD>
  inline double atInterpolated(const InterpolatedIndex& idxp, const ExtrapolationMethod interpolation) const;
  //! Non-templated interpolated getter for e.g. python exports
  inline double atInterpolated(const InterpolatedIndex& idxp,
                               const InterpolationMethod interpolation,
                               const ExtrapolationMethod extrapolation) const;

  //! Getter with extrapolation
  template <int EXTRAPOLATION_METHOD>
  inline T atExtrapolated(const Index& idx) const;
  //! Non-templated extrapolated getter for e.g. python exports
  inline T atExtrapolated(const Index& idx,
                          const ExtrapolationMethod extrapolation) const;

  /**
   * Computes the gradient of the interpolation of the map at a certain location.
   * @param[in]  idxp  Interpolated index at which to compute the gradient
   * @param[out] gradX The gradient in x-direction
   * @param[out] gradY The gradient in y-direction
   * @tparam INTERPOLATION_METHOD Type of interpolation
   * @tparam EXTRAPOLATION_METHOD If NONE is specified, throws planning2d::OutOfBoundAccessException if no full overlap
   */
  template <int INTERPOLATION_METHOD, int EXTRAPOLATION_METHOD = ExtrapolationMethod::CONSTANT>
  inline Eigen::Vector2d gradientInterpolated(const InterpolatedIndex& idxp) const;

  // Kept for downwards compatibility reasons
  inline double atInterpolatedBilinear(
      const InterpolatedIndex& idxp,
      const ExtrapolationMethod extrapolation = ExtrapolationMethod::NONE) const;
  inline double atInterpolatedBicubic(
      const InterpolatedIndex& idxp,
      const ExtrapolationMethod extrapolation = ExtrapolationMethod::NONE) const;
  inline double atInterpolatedBicubicPchip(
      const InterpolatedIndex& idxp,
      const ExtrapolationMethod extrapolation = ExtrapolationMethod::NONE) const;
  inline void gradientInterpolatedBilinear(
      const InterpolatedIndex& idxp, double& gradX, double& gradY,
      const ExtrapolationMethod extrapolation = ExtrapolationMethod::NONE) const;
  inline void gradientInterpolatedBicubic(
      const InterpolatedIndex& idxp, double& gradX, double& gradY,
      const ExtrapolationMethod extrapolation = ExtrapolationMethod::NONE) const;


  /// Note: The following facilitates python exports
  //! Setter, does check for valid bounds (assertion)
  inline void set(const Index& idx, const T& val);
  //! Setter, does check for valid bounds (assertion)
  inline void set(Index::Scalar ix, Index::Scalar iy, const T& val);
  //! Setter, does check for valid bounds (assertion)
  inline void set(const Position2d& idx, const T& val);

  //! Const getter, does not check for valid bounds
  inline const T& operator()(const Index& idx) const;
  //! Mutable getter, does not check for valid bounds
  inline T& operator()(const Index& idx);
  //! Const getter, does not check for valid bounds
  inline const T& operator()(Index::Scalar ix, Index::Scalar iy) const;
  //! Mutable getter, does not check for valid bounds
  inline T& operator()(Index::Scalar ix, Index::Scalar iy);
  //! Const getter, does not check for valid bounds
  inline const T& operator()(const Position2d& idx) const;
  //! Mutable getter, does not check for valid bounds
  inline T& operator()(const Position2d& idx);

  //! Returns whether an index points into the matrix or is out of bounds
  inline bool isInsideMap(const Index& idx) const;
  //! Returns whether an index points into the matrix or is out of bounds
  inline bool isInsideMap(Index::Scalar ix, Index::Scalar iy) const;
  //! Returns whether the x-index points into the matrix or is out of bounds
  inline bool isInsideMapX(Index::Scalar ix) const;
  //! Returns whether the y-index points into the matrix or is out of bounds
  inline bool isInsideMapY(Index::Scalar iy) const;
  //! Returns whether a position points into the matrix or is out of bounds
  inline bool isInsideMap(const Position2d& idx) const;
  //! Returns whether an interpolated index points into the matrix or is out of bounds
  inline bool isInterpolatedIndexInsideMap(const InterpolatedIndex& idx) const;

  /**
   * Returns true if the index lies on the map's boundary.
   * @param index Index to query
   * @return True iff \p index is a map boundary cell
   */
  inline bool isMapBoundary(const Index& index) const;

  //! Converts coordinates expressed in the parent frame of the map to an index
  inline Index toIndex(const Position2d& position) const;
  //! Converts an interpolated index to an integer index
  inline Index interpolatedIndextoIndex(const InterpolatedIndex& idxp) const;
  //! Converts an interpolated index to a position expressed in the parent frame
  inline Position2d toPosition(const InterpolatedIndex& idxp) const;
  //! Converts coordinates expressed in the parent frame of the map to an interpolated index
  inline InterpolatedIndex toInterpolatedIndex(const Position2d& position) const;
  //! Converts an index to a position expressed in the parent frame
  inline Position2d toPosition(const Index& index) const;
  //! Projects an index (possibly outside the map) to its nearest index
  inline Index projectToNearestIndex(const Index& index) const;
  //! Converts a region given in position coordinates to an index region
  inline AlignedBoxIndex toIndex(const AlignedBoxPosition& region) const;

  //! equality operator
  inline bool operator==(const Map<T,StorageOrder>& map) const;
  //! inequality operator
  inline bool operator!=(const Map<T,StorageOrder>& map) const;
  //! test metric information for equality
  template <typename T2, int StorageOrder2>
  inline bool hasEqualMetric(const Map<T2,StorageOrder2>& map) const;

  //! get a mutable iterator to the beginning of the data. Be aware of the storage order of the data!
  inline iterator begin();
  //! get a const iterator to the beginning of the data. Be aware of the storage order of the data!
  inline const_iterator begin() const;
  //! get a mutable iterator to the end of the data. Be aware of the storage order of the data!
  inline iterator end();
  //! get a const iterator to the end of the data. Be aware of the storage order of the data!
  inline const_iterator end() const;

  //! Converts an iterator back to an index, taking care of the storage order of the data
  inline Index toIndex(const_iterator it) const;

  //! Returns an aligned box for the occupancy grid extents in index coordinates. Note that the maximum is excluded.
  inline AlignedBoxIndex alignedBoxIndex() const;
  //! Returns an aligned box for the occupancy grid extents in position coordinates. Note that the maximum is excluded.
  inline Eigen::AlignedBox<double, 2> alignedBoxPosition() const;

  /**
   * Returns a dynamic-size block of the underlying Eigen matrix
   * @param min Index of the lower left corner
   * @param sz Size of the block
   * @return Block
   */
  inline Eigen::Block<Matrix> block(const Index& min, const Size2d& sz);
  inline const Eigen::Block<const Matrix> block(const Index& min, const Size2d& sz) const;

  /**
   * Returns a fixed-size block of the underlying Eigen matrix
   * @param min Index of the lower left corner
   * @return Block
   */
  template<int SizeX, int SizeY>
  inline Eigen::Block<Matrix, SizeY, SizeX> block(const Index& min);
  template<int SizeX, int SizeY>
  inline const Eigen::Block<const Matrix, SizeY, SizeX> block(const Index& min) const;

  /**
   * Extracts a region from the data
   * @param[in] region Region to extract in indices. Note that the maximum is excluded.
   * @tparam EXTRAPOLATION_METHOD If NONE is specified, throws planning2d::OutOfBoundAccessException if no full overlap
   * @return Data of the region
   */
  template <int EXTRAPOLATION_METHOD = ExtrapolationMethod::NONE>
  inline Matrix getRegion(const AlignedBoxIndex& region) const;

  /**
   * Extracts a region from the data
   * @param[in] min Index of the lower left corner
   * @tparam EXTRAPOLATION_METHOD If NONE is specified, throws planning2d::OutOfBoundAccessException if no full overlap
   * @return Data of the region
   */
  template <int SizeX, int SizeY, int EXTRAPOLATION_METHOD = ExtrapolationMethod::NONE>
  inline Eigen::Matrix<T, SizeY, SizeX> getRegion(const Index& min) const;

  /**
   * Shifts the provided region so that the resulting region lies within the map
   * @param[in,out] region Region to shift. Note that the maximum is excluded.
   */
  inline void shiftRegionToMap(AlignedBoxIndex& region) const;

  /**
   * Shifts the map to a new location
   * @param shift         coordinates to shift
   * @param newCellValue  New cells get this value
   */
  inline void shift(const Position2d& shift, const T& newCellValue);

  /**
   * @brief Inverts the data.
   * All non-zero values will become 0 and all 0 values will become 255.
   */
  inline void invert();

  /**
   * Merge data from this map into \p other.
   * @param[out] other   Data from this map will be merged into other.
   * @param[in]  functor Binary functor for combining two cell values.
   * @param[in]  fillComplement If specified, fills the relative complement of \p this in \p other with \p value fillComplement
   * @tparam Functor Functor type, e.g. std::function<T(const T, constT)>
   */
  template <typename Functor>
  inline void mergeInto(Map<T>& other, Functor functor, boost::optional<T> fillComplement = boost::optional<T>()) const;

  //! Overloaded stream operator
  template <typename C>
  friend inline std::ostream& operator<<(std::ostream& os, const Map<C>& map);

  //! serialization method
  template <class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

 private:
  template <int INTERPOLATION_METHOD, int EXTRAPOLATION_METHOD>
  Eigen::Matrix<T,
  math::InterpolationKernelTraits<INTERPOLATION_METHOD>::RowsAndCols,
  math::InterpolationKernelTraits<INTERPOLATION_METHOD>::RowsAndCols>
  getInterpolationKernel(const InterpolatedIndex& , Point2d<double>& delta) const;

 private:
  Pose2d _origin; //! 2d pose of the map's origin in the map's parent frame (lower left corner)
  Eigen::Isometry2d _tf; //! transformation from parent to grid frame
  double _resolutionMeters; //! size of one cell (pixel) in the map [m]
  double _reciprocalResolutionMeters; //! for speedup

  Matrix _mat; //! the actual data

}; /* class Map <T> */

template <typename T>
std::ostream& operator<<(std::ostream& os, const typename Map<T>::ExtrapolationMethod method);

/// \brief Ternary occupancy values
enum class OccupancyValue : uint8_t {
  FREE = 255,
  UNKNOWN = 127,
  OCCUPIED = 0, // Note: opencv distance transform assumes occupied cells to have value zero
};

/**
 * @brief Class representing a ternary occupancy grid
 * An occupancy grid is a map encoding information about the meaning of certain cell values.
 * This occupancy grid class encodes values for free, occupied and unknown.
 */
class OccupancyGrid : public Map<OccupancyValue, Eigen::RowMajor> {
 public:
  PLANNING_2D_POINTER_TYPEDEFS(OccupancyGrid);
  typedef OccupancyValue T;
  typedef typename Map<T>::Matrix Matrix;
  typedef typename Map<T>::Index Index;
  typedef typename Map<T>::Size2d Size2d;
  typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;

 public:

  using Map<OccupancyValue, Eigen::RowMajor>::Map; // use constructors of parent class

  //! Default constructor
  OccupancyGrid() = default;

  /**
   * Returns occupancy state of the grid at index \var idx. Does not check for valid bounds.
   * @param [in] idx Index into the matrix
   * @return True iff the grid at index \var idx is occupied.
   */
  inline bool isOccupied(const Index& idx) const;

  /**
   * Returns occupancy state of the grid at position \var pos. Does not check for valid bounds.
   * @param[in] pos Position in map frame, i.e. with respect to origin of map.
   * @return True iff the grid at position \var pos is occupied.
   */
  inline bool isOccupied(const Position2d& pos) const;

  /**
   * Returns true if the index is an obstacle boundary cell. A cell is an obstacle
   * boundary cell if it is occupied and at least one of its 8 neighbors is not.
   * @param index Index to query
   * @return True iff \p index is a boundary cell
   */
  inline bool isObstacleBoundary(const Index& index) const;

  /**
   * Computes an inflation kernel that can be passed to \ref inflateWithKernel(double inflationRadiusMeters) later.
   * @param @param[in] inflationRadiusMeters Inflation distance [m]
   * @return Inflation kernel
   */
  inline MatrixXb computeInflationKernel(double inflationRadiusMeters);

  /**
   * Dilates occupied cells of map by \var inflationRadiusMeters by moving a kernel over the matrix.
   * This method is not very efficient.
   * @param[in] inflationKernel Inflation kernel computed with \ref computeInflationKernel(double inflationRadiusMeters)
   * @param[in] treatUnknownOccupied Whether or not to treat unknown cells as occupied
   */
  inline void inflateWithKernel(const MatrixXb& inflationKernel, bool treatUnknownOccupied = true);

  //! Overloaded stream operator
  friend inline std::ostream& operator<<(std::ostream& os, const OccupancyGrid& map);

  //! serialization method
  template <class Archive>
  void serialize(Archive & ar, const unsigned int version);

}; /* class OccupancyGrid */


// some specialization for OccupancyValue type, where some of the Map methods won't work

template <>
template <>
inline typename Map<OccupancyValue, Eigen::RowMajor>::Matrix Map<OccupancyValue, Eigen::RowMajor>::getRegion<grid::ExtrapolationMethod::LINEAR>(const AlignedBoxIndex& /*region*/) const {
  // Note: This doesn't work, since linear extrapolation for the enum type makes no sense
  // TODO: Would still be possible for no and constant extrapolation
  SM_THROW(planning2d::NoImplementationException, __PRETTY_FUNCTION__ << " not implemented for type " << typeid(*this).name());
}

template <>
template <int EXTRAPOLATION_METHOD>
inline double Map<OccupancyValue, Eigen::RowMajor>::atInterpolated(const InterpolatedIndex& /*idx*/, const InterpolationMethod) const {
  SM_THROW(planning2d::NoImplementationException, __PRETTY_FUNCTION__ << " not implemented for type " << typeid(*this).name());
}

template <>
template <int INTERPOLATION_METHOD>
inline double Map<OccupancyValue, Eigen::RowMajor>::atInterpolated(const InterpolatedIndex& /*idx*/, const ExtrapolationMethod) const {
  SM_THROW(planning2d::NoImplementationException, __PRETTY_FUNCTION__ << " not implemented for type " << typeid(*this).name());
}

template <>
inline OccupancyValue Map<OccupancyValue, Eigen::RowMajor>::atExtrapolated(const Index& /*idx*/, const ExtrapolationMethod) const {
  SM_THROW(planning2d::NoImplementationException, __PRETTY_FUNCTION__ << " not implemented for type " << typeid(*this).name());
}

template <>
inline double Map<OccupancyValue, Eigen::RowMajor>::atInterpolatedBilinear(const InterpolatedIndex& /*idx*/, const ExtrapolationMethod /*extrapolation*/ /*= ExtrapolationMethod::NONE*/) const {
  SM_THROW(planning2d::NoImplementationException, __PRETTY_FUNCTION__ << " not implemented for type " << typeid(*this).name());
}

template <>
inline double Map<OccupancyValue, Eigen::RowMajor>::atInterpolatedBicubic(const InterpolatedIndex& /*idx*/, const ExtrapolationMethod /*extrapolation*/ /*= ExtrapolationMethod::NONE*/) const {
  SM_THROW(planning2d::NoImplementationException, __PRETTY_FUNCTION__ << " not implemented for type " << typeid(*this).name());
}

template <>
template <int INTERPOLATION_METHOD, int EXTRAPOLATION_METHOD /*= ExtrapolationMethod::CONSTANT*/>
inline Eigen::Vector2d Map<OccupancyValue, Eigen::RowMajor>::gradientInterpolated(const InterpolatedIndex& /*idx*/) const {
  SM_THROW(planning2d::NoImplementationException, __PRETTY_FUNCTION__ << " not implemented for type " << typeid(*this).name());
}

template <>
inline void Map<OccupancyValue, Eigen::RowMajor>::gradientInterpolatedBilinear(const InterpolatedIndex& /*idx*/, double& /*gradX*/, double& /*gradY*/,
                                                                               const ExtrapolationMethod /*extrapolation*/ /*= ExtrapolationMethod::NONE*/) const {
  SM_THROW(planning2d::NoImplementationException, __PRETTY_FUNCTION__ << " not implemented for type " << typeid(*this).name());
}

template <>
inline void Map<OccupancyValue, Eigen::RowMajor>::gradientInterpolatedBicubic(const InterpolatedIndex& /*idx*/, double& /*gradX*/, double& /*gradY*/,
                                                                               const ExtrapolationMethod /*extrapolation*/ /*= ExtrapolationMethod::NONE*/) const {
  SM_THROW(planning2d::NoImplementationException, __PRETTY_FUNCTION__ << " not implemented for type " << typeid(*this).name());
}


class OccupancyGridStamped : public OccupancyGrid, public StampedType {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(OccupancyGridStamped);

 public:

  using OccupancyGrid::OccupancyGrid; // use constructors of parent class

  //! Default constructor
  inline OccupancyGridStamped() = default;
  //! Constructor with grid and timestamp
  inline OccupancyGridStamped(const OccupancyGrid& grid, const Time& stamp);
  //! Constructor setting only the timestamp
  inline OccupancyGridStamped(const Time& stamp);
  //! Constructs Map with metric information. The actual map data will still be uninitialized afterwards
  //! and has to be set with \ref matrix()
  inline OccupancyGridStamped(const Pose2d& origin, const double resolutionMeters, const Time& stamp);
  //! Full constructor
  inline OccupancyGridStamped(const Pose2d& origin, const double resolutionMeters, const Map::Matrix& data, const Time& stamp);
  //! Constructor that initializes matrix size. Cell values are left uninitialized.
  inline OccupancyGridStamped(const Pose2d& origin, const double resolutionMeters, const Size2d& size, const Time& stamp);
  //! Constructor with all cells set to \p val
  inline OccupancyGridStamped(const Pose2d& origin, const double resolutionMeters, const Size2d& size, const T& val, const Time& stamp);

  //! equality operator
  inline bool operator==(const OccupancyGridStamped& map) const;
  //! inequality operator
  inline bool operator!=(const OccupancyGridStamped& map) const;

  //! serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

};

} /* namespace planning2d */

#include "implementation/OccupancyGridImplementation.hpp"

#endif /* PLANNING2D_OCCUPANCYGRID_HPP_ */
