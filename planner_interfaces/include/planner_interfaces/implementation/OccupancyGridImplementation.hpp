/*
 * MapImplementation.hpp
 *
 *  Created on: Oct 21, 2014
 *      Author: sculrich
 */

#ifndef PLANNING2D_OCCUPANCYGRID_IMPLEMENTATION_HPP_
#define PLANNING2D_OCCUPANCYGRID_IMPLEMENTATION_HPP_

// standard includes
#include <cmath>

// Boost includes
#include <boost/serialization/base_object.hpp>

// Schweizer Messer includes
#include <sm/eigen/serialization.hpp>
#include <sm/logging.hpp>


// self includes
#include "../Exceptions.hpp"
#include "../Support.hpp"

namespace planning2d {

namespace grid {

namespace details {

template <typename MAP, int EXTRAPOLATION_METHOD>
typename MAP::Scalar Extrapolator<MAP, EXTRAPOLATION_METHOD>::extrapolate(const MAP& map, const Index idx) {
  static_assert(EXTRAPOLATION_METHOD == planning2d::details::MapExtrapolationMethod::NONE, "Unsupported extrapolation method");
  return map.at(idx);
}

template <typename MAP>
typename MAP::Scalar Extrapolator<MAP, planning2d::details::MapExtrapolationMethod::CONSTANT>::extrapolate(const MAP& map, const Index idx) {
  return map(map.projectToNearestIndex(idx));
}

template <typename MAP>
typename MAP::Scalar Extrapolator<MAP, planning2d::details::MapExtrapolationMethod::LINEAR>::extrapolate(const MAP& map, const Index idx) {

  if (map.isInsideMap(idx))
    return map(idx);

  AlignedBoxIndex region(idx.asVector(), (idx + 2).asVector()); // A 2x2 box
  map.shiftRegionToMap(region);
  const Index delta = idx - Index(region.min());

  const auto mat = map.template block<2,2>( Index(region.min()));

  Eigen::Vector2d vec;
  vec[0] = math::linearInterpolate(mat.col(0), delta.y());
  vec[1] = math::linearInterpolate(mat.col(1), delta.y());
  return math::linearInterpolate(vec, delta.x());
}

} /* namespace details */

} /* namespace grid */

#define _MAP_TEMPLATE template <typename T, int StorageOrder>
#define _MAP_CLASS Map<T,StorageOrder>


// ************************* //
// ****** Map methods ****** //
// ************************* //

template <typename T>
std::ostream& operator<<(std::ostream& os, const typename Map<T>::ExtrapolationMethod method) {
  switch (method) {
    case Map<T>::ExtrapolationMethod::NONE: os << "NONE"; break;
    case Map<T>::ExtrapolationMethod::CONSTANT: os << "CONSTANT"; break;
    case Map<T>::ExtrapolationMethod::LINEAR: os << "LINEAR"; break;
  }
  return os;
}

_MAP_TEMPLATE
inline _MAP_CLASS::Map() :
    Map::Map(Pose2d(), SIGNAN, Map::Matrix(0, 0))
{

}

_MAP_TEMPLATE
inline _MAP_CLASS::Map(const Pose2d& origin, const double resolutionMeters) :
    Map::Map(origin, resolutionMeters, Map::Matrix(0, 0))
{

}

_MAP_TEMPLATE
inline _MAP_CLASS::Map(const Pose2d& origin, const double resolutionMeters, const Map::Matrix& data) :
    _mat(data)
{
  this->initialize(origin, resolutionMeters);
}

_MAP_TEMPLATE
inline _MAP_CLASS::Map(const Pose2d& origin, const double resolutionMeters, const Size2d& size) :
    Map::Map(origin, resolutionMeters, Map::Matrix(size.y(), size.x())) {

}

_MAP_TEMPLATE
inline _MAP_CLASS::Map(const Pose2d& origin, const double resolutionMeters, const Size2d& size, const T& val) :
    Map::Map(origin, resolutionMeters, Map::Matrix::Constant(size.y(), size.x(), val)) {

}

_MAP_TEMPLATE
void _MAP_CLASS::initialize(const Pose2d& origin,
                            double resolutionMeters) {

  this->setOrigin(origin);
  this->setResolution(resolutionMeters);
}


_MAP_TEMPLATE
inline const Pose2d& _MAP_CLASS::getOrigin() const {
  return _origin;
}

_MAP_TEMPLATE
inline Pose2d& _MAP_CLASS::getOrigin() {
  return _origin;
}

_MAP_TEMPLATE
inline void _MAP_CLASS::setOrigin(const Pose2d& origin) {
  _origin = origin;
  _tf.translation() = _origin.position().asVector();
  _tf.linear() = Eigen::Rotation2D<double>(_origin.yaw()).toRotationMatrix();
  _tf = _tf.inverse();
}

_MAP_TEMPLATE
inline const Eigen::Isometry2d& _MAP_CLASS::getTransformation() const {
  return _tf;
}

_MAP_TEMPLATE
inline double _MAP_CLASS::resolution() const {
  return _resolutionMeters;
}

_MAP_TEMPLATE
inline double _MAP_CLASS::reciprocalResolution() const {
  return _reciprocalResolutionMeters;
}

_MAP_TEMPLATE
void _MAP_CLASS::setResolution(double resolutionMeters) {
  SM_ASSERT_GT(ParameterException, resolutionMeters, 0., "The size of a grid cell has to be larger than zero");
  _resolutionMeters = resolutionMeters;
  _reciprocalResolutionMeters = 1./resolutionMeters;
}

_MAP_TEMPLATE
inline typename _MAP_CLASS::Size2d _MAP_CLASS::sizeInCells() const {
  return Size2d(sizeInCellsX(), sizeInCellsY());
}

_MAP_TEMPLATE
inline std::size_t _MAP_CLASS::sizeInCellsX() const {
  return _mat.cols();
}

_MAP_TEMPLATE
inline std::size_t _MAP_CLASS::sizeInCellsY() const {
  return _mat.rows();
}

_MAP_TEMPLATE
inline Point2d<double> _MAP_CLASS::sizeInMeters() const {
  return sizeInCells().template cast<double>().cwiseProduct(resolution());
}

_MAP_TEMPLATE
inline double _MAP_CLASS::sizeInMetersX() const {
  return sizeInCellsX()*resolution();
}

_MAP_TEMPLATE
inline double _MAP_CLASS::sizeInMetersY() const {
  return sizeInCellsY()*resolution();
}


_MAP_TEMPLATE
inline const typename _MAP_CLASS::Matrix& _MAP_CLASS::matrix() const {
  return _mat;
}

_MAP_TEMPLATE
inline typename _MAP_CLASS::Matrix& _MAP_CLASS::matrix() {
  return _mat;
}

_MAP_TEMPLATE
inline bool _MAP_CLASS::empty() const {
  return matrix().size() == 0;
}

_MAP_TEMPLATE
template <typename TOut>
inline Map<TOut,StorageOrder> _MAP_CLASS::cast() const {
  return Map<TOut,StorageOrder>(this->getOrigin(), this->resolution(), this->matrix().cast<TOut>());
}

_MAP_TEMPLATE
inline const T& _MAP_CLASS::at(const Index& idx) const {
  return this->at(idx.x(), idx.y());
}

_MAP_TEMPLATE
inline T& _MAP_CLASS::at(const Index& idx) {
  return this->at(idx.x(), idx.y());
}

_MAP_TEMPLATE
inline const T& _MAP_CLASS::at(Index::Scalar ix, Index::Scalar iy) const {
  SM_ASSERT_TRUE(OutOfBoundAccessException, isInsideMap(ix, iy), "Index " << Index(ix, iy) << " is outside the map");
  return _mat(iy, ix); // NOTE: let's switch x/y axis, so x-axis points to the right (cols)
}

_MAP_TEMPLATE
inline T& _MAP_CLASS::at(Index::Scalar ix, Index::Scalar iy) {
  SM_ASSERT_TRUE(OutOfBoundAccessException, isInsideMap(ix, iy), "Index " << Index(ix, iy) << " is outside the map");
  return _mat(iy, ix); // NOTE: let's switch x/y axis, so x-axis points to the right (cols)
}

_MAP_TEMPLATE
inline const T& _MAP_CLASS::at(const Position2d& pos) const {
  try {
    return this->at(toIndex(pos));
  } catch (const OutOfBoundAccessException& e) {
    SM_THROW(OutOfBoundAccessException, "Position " << pos << " is outside the map. " << e.what());
  } catch (...) {
    throw;
  }
}

_MAP_TEMPLATE
inline T& _MAP_CLASS::at(const Position2d& pos) {
  try {
    return this->at(toIndex(pos));
  } catch (const OutOfBoundAccessException& e) {
    SM_THROW(OutOfBoundAccessException, "Position " << pos << " is outside the map. " << e.what());
  } catch (...) {
    throw;
  }
}

_MAP_TEMPLATE
template <int INTERPOLATION_METHOD, int EXTRAPOLATION_METHOD>
Eigen::Matrix<T, math::InterpolationKernelTraits<INTERPOLATION_METHOD>::RowsAndCols, math::InterpolationKernelTraits<INTERPOLATION_METHOD>::RowsAndCols>
_MAP_CLASS::getInterpolationKernel(const InterpolatedIndex& idxp, Point2d<double>& delta) const {

  const auto y11 = this->interpolatedIndextoIndex(idxp);
  delta = idxp - y11.template cast<double>();
  SM_ASSERT_GE_LE_DBG(planning2d::RuntimeException, delta.x(), 0.0, 1.0, "Integer overflow for interpolated index: " << idxp);
  SM_ASSERT_GE_LE_DBG(planning2d::RuntimeException, delta.y(), 0.0, 1.0, "Integer overflow for interpolated index: " << idxp);

  typedef math::InterpolationKernelTraits<INTERPOLATION_METHOD> IKT;
  return this->getRegion<IKT::RowsAndCols,IKT::RowsAndCols,EXTRAPOLATION_METHOD>
         (y11 - grid::details::InterpolationShiftTraits<INTERPOLATION_METHOD>::LeftShift);
}

_MAP_TEMPLATE
template <int INTERPOLATION_METHOD, int EXTRAPOLATION_METHOD>
inline double _MAP_CLASS::atInterpolated(const InterpolatedIndex& idxp) const {
  Timer timer("Map: atInterpolated", false);
  math::Interpolator2d<INTERPOLATION_METHOD> interp;
  Point2d<double> delta;
  const auto mat = this->getInterpolationKernel<INTERPOLATION_METHOD, EXTRAPOLATION_METHOD>(idxp, delta);
  return interp.interpolate(mat, delta.x(), delta.y());
}

_MAP_TEMPLATE
template <int EXTRAPOLATION_METHOD>
inline double _MAP_CLASS::atInterpolated(const InterpolatedIndex& idxp, const InterpolationMethod interpolation) const {
  switch (interpolation) {
    case InterpolationMethod::LINEAR:
      return this->atInterpolated<EXTRAPOLATION_METHOD, InterpolationMethod::LINEAR>(idxp);
      break;
    case InterpolationMethod::CUBIC_CATMULL_ROM:
      return this->atInterpolated<EXTRAPOLATION_METHOD, InterpolationMethod::CUBIC_CATMULL_ROM>(idxp);
      break;
    case InterpolationMethod::CUBIC_PCHIP:
      return this->atInterpolated<EXTRAPOLATION_METHOD, InterpolationMethod::CUBIC_PCHIP>(idxp);
      break;
    default:
      SM_THROW(NoImplementationException, "Interpolation method " << interpolation << " not supported");
      break;
  }
}

_MAP_TEMPLATE
template <int INTERPOLATION_METHOD>
inline double _MAP_CLASS::atInterpolated(const InterpolatedIndex& idxp, const ExtrapolationMethod extrapolation) const {
  switch (extrapolation) {
    case ExtrapolationMethod::NONE:
      return this->atInterpolated<INTERPOLATION_METHOD, ExtrapolationMethod::NONE>(idxp);
      break;
    case ExtrapolationMethod::CONSTANT:
      return this->atInterpolated<INTERPOLATION_METHOD, ExtrapolationMethod::CONSTANT>(idxp);
      break;
    case ExtrapolationMethod::LINEAR:
      return this->atInterpolated<INTERPOLATION_METHOD, ExtrapolationMethod::LINEAR>(idxp);
      break;
    default:
      SM_THROW(NoImplementationException, "Extrapolation method " << extrapolation << " not supported");
      break;
  }
}

_MAP_TEMPLATE
inline double _MAP_CLASS::atInterpolated(const InterpolatedIndex& idxp,
                                         const InterpolationMethod interpolation,
                                         const ExtrapolationMethod extrapolation) const {
  switch (extrapolation) {
    case ExtrapolationMethod::NONE:
      return this->atInterpolated<ExtrapolationMethod::NONE>(idxp, interpolation);
      break;
    case ExtrapolationMethod::CONSTANT:
      return this->atInterpolated<ExtrapolationMethod::CONSTANT>(idxp, interpolation);
      break;
    case ExtrapolationMethod::LINEAR:
      return this->atInterpolated<ExtrapolationMethod::LINEAR>(idxp, interpolation);
      break;
    default:
      SM_THROW(NoImplementationException, "Extrapolation method " << extrapolation << " not supported");
      break;
  }
}

_MAP_TEMPLATE
template <int EXTRAPOLATION_METHOD>
inline T _MAP_CLASS::atExtrapolated(const Index& idx) const {
  return grid::details::Extrapolator<_MAP_CLASS, EXTRAPOLATION_METHOD>::extrapolate(*this, idx);
}

_MAP_TEMPLATE
inline T _MAP_CLASS::atExtrapolated(const Index& idx,
                                    const ExtrapolationMethod extrapolation) const {
  switch (extrapolation) {
    case ExtrapolationMethod::NONE:
      return this->atExtrapolated<ExtrapolationMethod::NONE>(idx);
      break;
    case ExtrapolationMethod::CONSTANT:
      return this->atExtrapolated<ExtrapolationMethod::CONSTANT>(idx);
      break;
    case ExtrapolationMethod::LINEAR:
      return this->atExtrapolated<ExtrapolationMethod::LINEAR>(idx);
      break;
    default:
      SM_THROW(NoImplementationException, "Extrapolation method " << extrapolation << " not supported");
      break;
  }
}

_MAP_TEMPLATE
template <int INTERPOLATION_METHOD, int EXTRAPOLATION_METHOD /*= ExtrapolationMethod::CONSTANT*/>
inline Eigen::Vector2d _MAP_CLASS::gradientInterpolated(const InterpolatedIndex& idxp) const {
  Timer timer("Map: atInterpolated", false);
  math::Interpolator2d<INTERPOLATION_METHOD> interp;
  Point2d<double> delta;
  const auto mat = this->getInterpolationKernel<INTERPOLATION_METHOD,EXTRAPOLATION_METHOD>(idxp, delta);
  return interp.gradient(mat, delta.x(), delta.y()) * reciprocalResolution();
}

_MAP_TEMPLATE
inline double _MAP_CLASS::atInterpolatedBilinear(
    const InterpolatedIndex& idxp,
    const ExtrapolationMethod extrapolation /*= ExtrapolationMethod::NONE*/) const {
  return this->atInterpolated<InterpolationMethod::LINEAR>(idxp, extrapolation);
}

_MAP_TEMPLATE
inline double _MAP_CLASS::atInterpolatedBicubic(
    const InterpolatedIndex& idxp,
    const ExtrapolationMethod extrapolation /*= ExtrapolationMethod::NONE*/) const {
  return this->atInterpolated<InterpolationMethod::CUBIC_CATMULL_ROM>(idxp, extrapolation);
}

_MAP_TEMPLATE
inline double _MAP_CLASS::atInterpolatedBicubicPchip(
    const InterpolatedIndex& idxp,
    const ExtrapolationMethod extrapolation /*= ExtrapolationMethod::NONE*/) const {
  return this->atInterpolated<InterpolationMethod::CUBIC_PCHIP>(idxp, extrapolation);
}

_MAP_TEMPLATE
inline void _MAP_CLASS::gradientInterpolatedBilinear(
    const InterpolatedIndex& idxp, double& gradX, double& gradY,
    const ExtrapolationMethod extrapolation /*= ExtrapolationMethod::NONE*/) const {

  Eigen::Vector2d ret;
  switch (extrapolation) {
    case ExtrapolationMethod::NONE:
      ret = this->gradientInterpolated<InterpolationMethod::LINEAR, ExtrapolationMethod::NONE>(idxp);
      break;
    case ExtrapolationMethod::CONSTANT:
      ret = this->gradientInterpolated<InterpolationMethod::LINEAR, ExtrapolationMethod::CONSTANT>(idxp);
      break;
    case ExtrapolationMethod::LINEAR:
      ret = this->gradientInterpolated<InterpolationMethod::LINEAR, ExtrapolationMethod::LINEAR>(idxp);
      break;
    default:
      SM_THROW(NoImplementationException, "Extrapolation method " << extrapolation << " not supported");
      break;
  }
  gradX = ret.x();
  gradY = ret.y();
}

_MAP_TEMPLATE
inline void _MAP_CLASS::gradientInterpolatedBicubic(
    const InterpolatedIndex& idxp, double& gradX, double& gradY,
    const ExtrapolationMethod extrapolation /*= ExtrapolationMethod::NONE*/) const {

  Eigen::Vector2d ret;
  switch (extrapolation) {
    case ExtrapolationMethod::NONE:
      ret = this->gradientInterpolated<InterpolationMethod::CUBIC_CATMULL_ROM, ExtrapolationMethod::NONE>(idxp);
      break;
    case ExtrapolationMethod::CONSTANT:
      ret = this->gradientInterpolated<InterpolationMethod::CUBIC_CATMULL_ROM, ExtrapolationMethod::CONSTANT>(idxp);
      break;
    case ExtrapolationMethod::LINEAR:
      ret = this->gradientInterpolated<InterpolationMethod::CUBIC_CATMULL_ROM, ExtrapolationMethod::LINEAR>(idxp);
      break;
    default:
      SM_THROW(NoImplementationException, "Extrapolation method " << extrapolation << " not supported");
      break;
  }
  gradX = ret.x();
  gradY = ret.y();
}

_MAP_TEMPLATE
inline void _MAP_CLASS::set(const Index& idx, const T& val) {
  this->at(idx) = val;
}

_MAP_TEMPLATE
inline void _MAP_CLASS::set(Index::Scalar ix, Index::Scalar iy, const T& val) {
  this->at(ix, iy) = val;
}

_MAP_TEMPLATE
inline void _MAP_CLASS::set(const Position2d& pos, const T& val) {
  this->at(pos) = val;
}

_MAP_TEMPLATE
inline const T& _MAP_CLASS::operator()(const Index& idx) const {
  return _mat(idx.y(), idx.x()); // NOTE: let's switch x/y axis, so x-axis points to the right (cols)
}

_MAP_TEMPLATE
inline T& _MAP_CLASS::operator()(const Index& idx) {
  return _mat(idx.y(), idx.x()); // NOTE: let's switch x/y axis, so x-axis points to the right (cols)
}

_MAP_TEMPLATE
inline const T& _MAP_CLASS::operator()(Index::Scalar ix, Index::Scalar iy) const {
  return _mat(iy, ix); // NOTE: let's switch x/y axis, so x-axis points to the right (cols)
}

_MAP_TEMPLATE
inline T& _MAP_CLASS::operator()(Index::Scalar ix, Index::Scalar iy) {
  return _mat(iy, ix); // NOTE: let's switch x/y axis, so x-axis points to the right (cols)
}

_MAP_TEMPLATE
inline const T& _MAP_CLASS::operator()(const Position2d& pos) const {
  return this->at(toIndex(pos));
}

_MAP_TEMPLATE
inline T& _MAP_CLASS::operator()(const Position2d& pos) {
  return this->at(toIndex(pos));
}


_MAP_TEMPLATE
inline bool _MAP_CLASS::isInsideMap(const Index& idx) const {
  return this->isInsideMap(idx.x(), idx.y());
}

_MAP_TEMPLATE
inline bool _MAP_CLASS::isInsideMap(Index::Scalar ix, Index::Scalar iy) const {
  return this->isInsideMapX(ix) && this->isInsideMapY(iy);
}

_MAP_TEMPLATE
inline bool _MAP_CLASS::isInsideMapX(Index::Scalar ix) const {
  return (ix >= 0 && ix < _mat.cols());
}

_MAP_TEMPLATE
inline bool _MAP_CLASS::isInsideMapY(Index::Scalar iy) const {
  return (iy >= 0 && iy < _mat.rows());
}

_MAP_TEMPLATE
inline bool _MAP_CLASS::isInsideMap(const Position2d& pos) const {
  return isInsideMap(toIndex(pos));
}

_MAP_TEMPLATE
inline bool _MAP_CLASS::isInterpolatedIndexInsideMap(const InterpolatedIndex& idxp) const {
  return isInsideMap(interpolatedIndextoIndex(idxp));
}

_MAP_TEMPLATE
inline bool _MAP_CLASS::isMapBoundary(const Index& index) const {
  return UNLIKELY(index.x() == 0L || index.y() == 0L || index.x() == static_cast<Index::Scalar>(this->sizeInCellsX()) - 1
      || index.y() == static_cast<Index::Scalar>(this->sizeInCellsY()) - 1);
}

_MAP_TEMPLATE
inline typename _MAP_CLASS::Index _MAP_CLASS::toIndex(const Position2d& position) const {
  return this->toInterpolatedIndex(position).floor().template cast<Index::Scalar>();
}

_MAP_TEMPLATE
inline typename _MAP_CLASS::Index _MAP_CLASS::interpolatedIndextoIndex(const InterpolatedIndex& idxp) const {
  return idxp.floor().template cast<Index::Scalar>();
}

_MAP_TEMPLATE
inline Position2d _MAP_CLASS::toPosition(const InterpolatedIndex& idxp) const {
  if (fabs(getOrigin().yaw()) > 1e-12)
    return Position2d((_tf.inverse()*idxp.asVector())* this->resolution());
  return idxp.cwiseProduct(this->resolution()) + getOrigin().position(); // optimized version if yaw is zero
}

_MAP_TEMPLATE
inline typename _MAP_CLASS::InterpolatedIndex _MAP_CLASS::toInterpolatedIndex(const Position2d& position) const {
  if (fabs(getOrigin().yaw()) > 1e-12)
    return InterpolatedIndex((_tf*position.asVector())*_reciprocalResolutionMeters);
  return (position - getOrigin().position()).cwiseProduct(_reciprocalResolutionMeters); // optimized version if yaw is zero
}

_MAP_TEMPLATE
inline Position2d _MAP_CLASS::toPosition(const typename _MAP_CLASS::Index& index) const {
  if (fabs(getOrigin().yaw()) > 1e-12)
    return Position2d(_tf.inverse()*index.template cast<double>().cwiseProduct(_resolutionMeters).asVector());
  return index.template cast<double>().cwiseProduct(_resolutionMeters) + _origin.position(); // optimized version if yaw is zero
}

_MAP_TEMPLATE
inline typename _MAP_CLASS::Index _MAP_CLASS::projectToNearestIndex(const typename _MAP_CLASS::Index& index) const {
  return Index(std::max( std::min(index.x(), static_cast<Index::Scalar>(this->sizeInCellsX()) - (Index::Scalar)1), (Index::Scalar)0),
               std::max( std::min(index.y(), static_cast<Index::Scalar>(this->sizeInCellsY()) - (Index::Scalar)1), (Index::Scalar)0));
}

_MAP_TEMPLATE
inline typename _MAP_CLASS::AlignedBoxIndex _MAP_CLASS::toIndex(const AlignedBoxPosition& region) const {
  return AlignedBoxIndex(this->toIndex(Position2d(region.min())).asVector(), this->toIndex(Position2d(region.max())).asVector());
}

_MAP_TEMPLATE
inline bool _MAP_CLASS::operator==(const _MAP_CLASS& map) const {
  return  this->hasEqualMetric(map)
          && (_mat.array() == map.matrix().array()).all();
}

_MAP_TEMPLATE
inline bool _MAP_CLASS::operator!=(const  _MAP_CLASS& map) const {
  return !(*this == map);
}

_MAP_TEMPLATE
template <typename T2, int StorageOrder2>
inline bool _MAP_CLASS::hasEqualMetric(const Map<T2,StorageOrder2>& map) const
{
  return _origin == map.getOrigin()
      && _tf.isApprox(map.getTransformation())
      && _resolutionMeters == map.resolution()
      && _reciprocalResolutionMeters == map.reciprocalResolution()
      && this->sizeInCells() == map.sizeInCells();
}

_MAP_TEMPLATE
inline typename _MAP_CLASS::iterator _MAP_CLASS::begin() {
  return this->matrix().data();
}

_MAP_TEMPLATE
inline typename _MAP_CLASS::const_iterator _MAP_CLASS::begin() const {
  return this->matrix().data();
}

_MAP_TEMPLATE
inline typename _MAP_CLASS::iterator _MAP_CLASS::end() {
  return this->matrix().data() + this->matrix().size();
}

_MAP_TEMPLATE
inline typename _MAP_CLASS::const_iterator _MAP_CLASS::end() const {
  return this->matrix().data() + this->matrix().size();
}

_MAP_TEMPLATE
inline typename _MAP_CLASS::Index _MAP_CLASS::toIndex(const_iterator it) const {
  const Index::Scalar sub = it - this->begin();
  const Index::Scalar row=sub / this->sizeInCellsX();
  const Index::Scalar col=sub % this->sizeInCellsX();
  if (StorageOrder == Eigen::RowMajor)
    return Index(col, row);
  return Index(row, col);
}

_MAP_TEMPLATE
inline typename _MAP_CLASS::AlignedBoxIndex _MAP_CLASS::alignedBoxIndex() const {
  return AlignedBoxIndex(Eigen::Matrix<Index::Scalar, 2, 1>::Zero(), sizeInCells().template cast<Index::Scalar>().asVector());
}

_MAP_TEMPLATE
inline typename _MAP_CLASS::AlignedBoxPosition _MAP_CLASS::alignedBoxPosition() const {
  return AlignedBoxPosition(getOrigin().position().asVector(), (getOrigin().position() + sizeInCells().template cast<double>().cwiseProduct(resolution())).asVector());
}

_MAP_TEMPLATE
inline Eigen::Block<typename _MAP_CLASS::Matrix> _MAP_CLASS::block(const Index& min, const Size2d& sz) {
  return this->matrix().block(min.y(), min.x(), sz.y(), sz.x()); // TODO: Avoid code duplication with const version
}

_MAP_TEMPLATE
inline const Eigen::Block<const typename _MAP_CLASS::Matrix> _MAP_CLASS::block(const Index& min, const Size2d& sz) const {
  return this->matrix().block(min.y(), min.x(), sz.y(), sz.x());
}

_MAP_TEMPLATE
template<int SizeX, int SizeY>
inline Eigen::Block<typename _MAP_CLASS::Matrix, SizeY, SizeX> _MAP_CLASS::block(const Index& min) {
  return this->matrix().block<SizeY, SizeX>(min.y(), min.x()); // TODO: Avoid code duplication with const version
}

_MAP_TEMPLATE
template<int SizeX, int SizeY>
inline const Eigen::Block<const typename _MAP_CLASS::Matrix, SizeY, SizeX> _MAP_CLASS::block(const Index& min) const {
  return this->matrix().block<SizeY, SizeX>(min.y(), min.x());
}

_MAP_TEMPLATE
template <int EXTRAPOLATION_METHOD>
inline typename _MAP_CLASS::Matrix _MAP_CLASS::getRegion(const AlignedBoxIndex& region) const {

  Timer timer("Map: getRegion", false);

  const Eigen::Matrix<Index::Scalar, 2 ,1> regionSizes = region.sizes();
  Matrix mat(regionSizes.y(), regionSizes.x());

  Index idxSrc, idxDst;
  for (idxDst.y() = 0, idxSrc.y() = region.min().y(); idxDst.y() < regionSizes.y(); idxSrc.y()++, idxDst.y()++) {
    for (idxDst.x() = 0, idxSrc.x() = region.min().x(); idxDst.x() < regionSizes.x(); idxSrc.x()++, idxDst.x()++) {

      if (LIKELY(this->isInsideMap(idxSrc))) { // shortcut for efficiency
        mat(idxDst.y(), idxDst.x()) = this->operator()(idxSrc);
        continue;
      }

      mat(idxDst.y(), idxDst.x()) = this->atExtrapolated<EXTRAPOLATION_METHOD>(idxSrc);
    }
  }

  return mat;
}

_MAP_TEMPLATE
template <int SizeX, int SizeY, int EXTRAPOLATION_METHOD>
inline Eigen::Matrix<typename _MAP_CLASS::Scalar, SizeY, SizeX> _MAP_CLASS::getRegion(const Index& min) const {
  Eigen::Matrix<T, SizeY, SizeX> mat;
  Index idxSrc, idxDst;
  for (idxDst.y() = 0, idxSrc.y() = min.y(); idxDst.y() < SizeY; idxSrc.y()++, idxDst.y()++) {
    for (idxDst.x() = 0, idxSrc.x() = min.x(); idxDst.x() < SizeX; idxSrc.x()++, idxDst.x()++) {

      if (LIKELY(this->isInsideMap(idxSrc))) { // shortcut for efficiency
        mat(idxDst.y(), idxDst.x()) = this->operator()(idxSrc);
        continue;
      }

      mat(idxDst.y(), idxDst.x()) = this->atExtrapolated<EXTRAPOLATION_METHOD>(idxSrc);
    }
  }

  return mat;
}

_MAP_TEMPLATE
inline void _MAP_CLASS::shiftRegionToMap(AlignedBoxIndex& region) const {
  Timer timer("Map: shiftRegionToMap", false);
  const AlignedBoxIndex map = this->alignedBoxIndex();
  SM_ASSERT_TRUE(planning2d::FunctionInputException, (region.sizes().array() <= map.sizes().array()).all(),
                 "The region you want to shift inside the map is larger than the map. This is not possible");
  const AlignedBoxIndex intersection = map.intersection(region);
  Index::Scalar shiftX = -std::max(region.max().x() - intersection.max().x(), (Index::Scalar)0);
  if (shiftX == 0)
    shiftX = std::max(intersection.min().x() - region.min().x(), (Index::Scalar)0);
  Index::Scalar shiftY = -std::max(region.max().y() - intersection.max().y(), (Index::Scalar)0);
  if (shiftY == 0)
    shiftY = std::max(intersection.min().y() - region.min().y(), (Index::Scalar)0);
  region.translate( Eigen::Matrix<Index::Scalar, 2 ,1>(shiftX, shiftY) );
}

//_MAP_TEMPLATE
//inline void _MAP_CLASS::gradientInterpolatedBilinear(const InterpolatedIndex& idxp, double& gradX, double& gradY, const ExtrapolationMethod extrapolation /*= ExtrapolationMethod::NONE*/) const {
//
//  Timer timer("Map: gradientInterpolatedBilinear", false);
//
//  // we need to retrieve the 4 pixels around the lookup position
//  const typename _MAP_CLASS::Index y11 = this->interpolatedIndextoIndex(idxp);
//  const Eigen::Matrix<T,2,2> mat = this->getRegion(AlignedBoxIndex(y11.asVector(), (y11 + 2).asVector()), extrapolation);
//
//  const Point2d<double> delta = idxp - y11.template cast<double>();
//  SM_ASSERT_GE_LE_DBG(planning2d::RuntimeException, delta.x(), 0.0, 1.0, "Integer overflow for interpolated index: " << idxp);
//  SM_ASSERT_GE_LE_DBG(planning2d::RuntimeException, delta.y(), 0.0, 1.0, "Integer overflow for interpolated index: " << idxp);
//  const Eigen::Vector2d grad = math::gradientBilinearInterpolate(mat, delta.x(), delta.y()) * reciprocalResolution();
//  gradX = grad.x();
//  gradY = grad.y();
//}
//
//_MAP_TEMPLATE
//inline void _MAP_CLASS::gradientInterpolatedBicubic(const InterpolatedIndex& idxp, double& gradX, double& gradY, const ExtrapolationMethod extrapolation /*= ExtrapolationMethod::NONE*/) const {
//
//  Timer timer("Map: gradientInterpolatedBicubic", false);
//
//  // we need to retrieve the 16 pixels around the lookup position
//  const typename _MAP_CLASS::Index y11 = this->interpolatedIndextoIndex(idxp);
//  const Eigen::Matrix<T,4,4> mat = this->getRegion(AlignedBoxIndex( (y11 - 1).asVector(), (y11 + 3).asVector()), extrapolation);
//
//  const Point2d<double> delta = idxp - y11.template cast<double>();
//  SM_ASSERT_GE_LE_DBG(planning2d::RuntimeException, delta.x(), 0.0, 1.0, "Integer overflow for interpolated index: " << idxp);
//  SM_ASSERT_GE_LE_DBG(planning2d::RuntimeException, delta.y(), 0.0, 1.0, "Integer overflow for interpolated index: " << idxp);
//  const Eigen::Vector2d grad = math::gradientBicubicInterpolate(mat, delta.x(), delta.y()) * reciprocalResolution();
//  gradX = grad.x();
//  gradY = grad.y();
//}

_MAP_TEMPLATE
inline void _MAP_CLASS::shift(const Position2d& shift, const T& newCellValue) {
  // TODO: more efficient implementation possible?
  Matrix mat = Matrix::Constant(_mat.rows(), _mat.cols(), newCellValue);
  const Index::Vector shiftPixel = shift.cwiseProduct(_reciprocalResolutionMeters).template cast<Index::Scalar>().asVector(); // shift in pixels
  const AlignedBoxIndex source = this->alignedBoxIndex();
  const AlignedBoxIndex dest = AlignedBoxIndex(source).translate(shiftPixel);
  const AlignedBoxIndex intersection = source.intersection(dest);
  if (!intersection.isEmpty()) {
    const Eigen::Matrix<Index::Scalar, 2 ,1> blockSize = intersection.sizes();
    const Eigen::Matrix<Index::Scalar, 2 ,1> destMin = intersection.min() - shiftPixel;
    mat.block(destMin.y(), destMin.x(), blockSize.y(), blockSize.x()) = _mat.block(intersection.min().y(), intersection.min().x(), blockSize.y(), blockSize.x());
  }
  _mat = mat;
  this->getOrigin().position() += shift;
}

_MAP_TEMPLATE
inline void _MAP_CLASS::invert() {
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> zeros = this->matrix().array() == (T)0;
  this->matrix() = (this->matrix().array() != (T)0).select((T)0, this->matrix());
  this->matrix() = zeros.select((T)255, this->matrix());
}

_MAP_TEMPLATE
template <typename Functor>
inline void _MAP_CLASS::mergeInto(Map<T>& other, Functor functor, boost::optional<T> fillComplement /*= boost::optional<T>()*/) const {

  if (this->hasEqualMetric(other)) { // optimized version if metric information is the same

    other.matrix() = other.matrix().binaryExpr(this->matrix(), functor);

  } else if (this->resolution() == other.resolution() && this->getOrigin().yaw() == other.getOrigin().yaw()) { // optimized version if maps are aligned

    const auto isecPosition = other.alignedBoxPosition().intersection(this->alignedBoxPosition());
    if (!isecPosition.isEmpty()) {
      const auto isecIndexThis = this->toIndex(isecPosition);
      const auto isecIndexOther = other.toIndex(isecPosition);

      auto otherBlock = other.block(Index(isecIndexOther.min()), Size2d(isecIndexOther.sizes().cast<std::size_t>()));
      auto thisBlock = this->block(Index(isecIndexThis.min()), Size2d(isecIndexThis.sizes().cast<std::size_t>()));
      SM_ASSERT_EQ_DBG(planning2d::RuntimeException, otherBlock.rows(), thisBlock.rows(), "");
      SM_ASSERT_EQ_DBG(planning2d::RuntimeException, otherBlock.cols(), thisBlock.cols(), "");
      otherBlock = otherBlock.binaryExpr(thisBlock, functor);

      if (fillComplement) {
        int num;
        num = other.sizeInCellsY() - isecIndexOther.max().y();
        other.matrix().bottomRows(num > 0 ? num : 0).array().setConstant(fillComplement.get());
        other.matrix().topRows(isecIndexOther.min().y()).array().setConstant(fillComplement.get());
        num = other.sizeInCellsX() - isecIndexOther.max().x();
        other.matrix().rightCols(num > 0 ? num : 0).array().setConstant(fillComplement.get());
        other.matrix().leftCols(isecIndexOther.min().x()).array().setConstant(fillComplement.get());
      }

    } else if (fillComplement) { // no overlap
      other.matrix().array().setConstant(fillComplement.get());
    }

  } else { // not aligned or not same resolution

    for (auto it = other.begin(); it != other.end(); ++it) {
      const auto pos = other.toPosition(other.toIndex(it));
      const auto thisIndex = this->toIndex(pos);
      if (this->isInsideMap(thisIndex))
        *it = functor(*it, this->operator()(thisIndex));
      else if (fillComplement)
        *it = fillComplement.get();
    }

  }
}

_MAP_TEMPLATE
template<class Archive>
void _MAP_CLASS::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & _origin;
  ar & _tf;
  ar & _resolutionMeters;
  ar & _reciprocalResolutionMeters;
  ar & _mat;
}

template <typename C>
inline std::ostream& operator<<(std::ostream& os, const Map<C>& map) {
  static Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
  return os << map.matrix().format(fmt);
}

// Pre-compile some map instances
extern template class Map<uint8_t, Eigen::RowMajor>;
extern template class Map<uint16_t, Eigen::RowMajor>;
extern template class Map<uint32_t, Eigen::RowMajor>;
extern template class Map<uint64_t, Eigen::RowMajor>;
extern template class Map<int8_t, Eigen::RowMajor>;
extern template class Map<int16_t, Eigen::RowMajor>;
extern template class Map<int32_t, Eigen::RowMajor>;
extern template class Map<int64_t, Eigen::RowMajor>;
extern template class Map<OccupancyValue, Eigen::RowMajor>;
extern template class Map<float, Eigen::RowMajor>;
extern template class Map<double, Eigen::RowMajor>;

#undef _MAP_TEMPLATE
#undef _MAP_CLASS

// ************************** //
// * Occupancy grid methods * //
// ************************** //

inline bool OccupancyGrid::isOccupied(const Index& idx) const {
  return this->at(idx) == OccupancyValue::OCCUPIED;
}

inline bool OccupancyGrid::isOccupied(const Position2d& pos) const {
  return this->at(pos) == OccupancyValue::OCCUPIED;
}

inline bool OccupancyGrid::isObstacleBoundary(const Index& index) const {
  return this->isMapBoundary(index) || (this->isOccupied(index) && this->block<3,3>(index-1).maxCoeff() != OccupancyValue::OCCUPIED);
}


inline OccupancyGrid::MatrixXb OccupancyGrid::computeInflationKernel(double inflationRadiusMeters) {

  // build kernel: generate a circle of ones in the middle of the kernel
  const int inflationRadiusInCells = std::ceil(inflationRadiusMeters * reciprocalResolution());
  const int inflationRadiusInCellsSquared = math::square(inflationRadiusInCells);
  const int kernelSizeInCells = 2*inflationRadiusInCells+1;
  MatrixXb kernel = MatrixXb::Zero(kernelSizeInCells, kernelSizeInCells);
  for (int ix = 0; ix < kernelSizeInCells; ix++) {
    for (int iy = 0; iy < kernelSizeInCells; iy++) {
      if (Index((ix - inflationRadiusInCells), iy - inflationRadiusInCells).squaredNorm() <= inflationRadiusInCellsSquared) {
        kernel(ix, iy) = true;
      }
    }
  }
  return kernel;
}

inline void OccupancyGrid::inflateWithKernel(const OccupancyGrid::MatrixXb& inflationKernel, bool treatUnknownOccupied /*= true*/) {

  SM_ASSERT_EQ(planning2d::FunctionInputException, inflationKernel.rows(), inflationKernel.cols(),
               "Use computeInflationKernel to compute the kernel or pass a quadratic kernel");
  SM_ASSERT_EQ(planning2d::FunctionInputException, inflationKernel.rows()%2, 1,
               "Use computeInflationKernel to compute the kernel or pass a kernel with odd number of rows and columns");

  const int inflationRadiusInCells = (inflationKernel.rows()-1)/2;
  const int kernelSizeInCells = 2*inflationRadiusInCells+1;

  OccupancyGrid::Matrix& mat = matrix();

  // create temporary binary map
  // Note, we could avoid that if we did not have a ternary representation in the first place
  MatrixXb binaryMat;
  if (treatUnknownOccupied)
    binaryMat = (mat.array() != OccupancyValue::FREE);
  else
    binaryMat = mat.array() == OccupancyValue::OCCUPIED;

  // dilate obstacles by moving the kernel over the map
  for (int col = 0, szc = binaryMat.cols(); col < szc; col++) {
    for (int row = 0, szr = binaryMat.rows(); row < szr; row++) {

      // We have to take care not to move the kernel beyond the boundaries of the matrix
      const int cutRowStart = std::max(inflationRadiusInCells - row, 0);
      const int cutRowEnd = std::max(inflationRadiusInCells + row + 1 - (int)matrix().rows(), 0);
      const int cutColStart = std::max(inflationRadiusInCells - col, 0);
      const int cutColEnd = std::max(inflationRadiusInCells + col + 1 - (int)matrix().cols(), 0);

      const int szRow = kernelSizeInCells - (cutRowStart + cutRowEnd);
      const int szCol = kernelSizeInCells - (cutColStart + cutColEnd);

      if (binaryMat.block(row - inflationRadiusInCells + cutRowStart, col - inflationRadiusInCells + cutColStart, szRow, szCol).cwiseProduct(
          inflationKernel.block(cutRowStart, cutColStart, szRow, szCol)).any()) {
        mat(row, col) = OccupancyValue::OCCUPIED;
      }
    }
  }
}

template<class Archive>
void OccupancyGrid::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & boost::serialization::base_object< Map<OccupancyGrid::T> >(*this);
}

inline std::ostream& operator<<(std::ostream& os, const OccupancyGrid& grid) {
  return os << "origin: " << grid.getOrigin() << ", resolution: " << grid.resolution() << std::endl <<
      grid.matrix().cast<unsigned int>().format(Eigen::IOFormat(1, 0, ", ", "\n", "[", "]"));
}



inline OccupancyGridStamped::OccupancyGridStamped(const OccupancyGrid& grid, const Time& stamp)
    : OccupancyGrid::OccupancyGrid(grid), StampedType(stamp)
{

}

inline OccupancyGridStamped::OccupancyGridStamped(const Time& stamp)
    : OccupancyGrid::OccupancyGrid(), StampedType(stamp)
{

}

inline OccupancyGridStamped::OccupancyGridStamped(const Pose2d& origin, const double resolutionMeters, const Time& stamp)
    : OccupancyGrid::OccupancyGrid(origin, resolutionMeters, OccupancyGrid::Matrix(0, 0)), StampedType(stamp)
{

}

inline OccupancyGridStamped::OccupancyGridStamped(const Pose2d& origin, const double resolutionMeters, const OccupancyGridStamped::Matrix& data, const Time& stamp)
    : OccupancyGrid::OccupancyGrid(origin, resolutionMeters, data), StampedType(stamp)
{

}

inline OccupancyGridStamped::OccupancyGridStamped(const Pose2d& origin, const double resolutionMeters, const Size2d& size, const Time& stamp)
    : OccupancyGrid::OccupancyGrid(origin, resolutionMeters, size), StampedType(stamp)
{

}

inline OccupancyGridStamped::OccupancyGridStamped(const Pose2d& origin, const double resolutionMeters, const Size2d& size, const T& val, const Time& stamp)
    : OccupancyGrid::OccupancyGrid(origin, resolutionMeters, size, val), StampedType(stamp)
{

}

inline bool OccupancyGridStamped::operator==(const OccupancyGridStamped& map) const
{
  return OccupancyGrid::operator==(map) && StampedType::operator==(map);
}

inline bool OccupancyGridStamped::operator!=(const OccupancyGridStamped& map) const
{
  return !(*this == map);
}

template<class Archive>
inline void OccupancyGridStamped::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & boost::serialization::base_object<OccupancyGrid>(*this);
  ar & boost::serialization::base_object<StampedType>(*this);
}

} /* namespace planning2d */

#endif /* PLANNING2D_OCCUPANCYGRID_IMPLEMENTATION_HPP_ */
