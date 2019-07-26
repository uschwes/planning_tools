/*
 * ChangeDetectionImpl.hpp
 *
 *  Created on: 18.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#ifndef INCLUDE_PLANNER_ALGORITHMS_IMPLEMENTATION_MAPCHANGEDETECTIONIMPL_HPP_
#define INCLUDE_PLANNER_ALGORITHMS_IMPLEMENTATION_MAPCHANGEDETECTIONIMPL_HPP_


namespace planning2d
{
namespace algorithms
{

template <typename Scalar>
void changeDetection(const Map<Scalar>& map, Map<bool>& binary, bool tagBoundary = true)
{
  SM_ASSERT_EQ( FunctionInputException, map.sizeInCells(), binary.sizeInCells(), "");
  SM_ASSERT_EQ( FunctionInputException, map.resolution(), binary.resolution(), "");
  SM_ASSERT_EQ( FunctionInputException, map.getOrigin(), binary.getOrigin(), "");

  // tag boundary cells
  binary.matrix().bottomRows(1).array().setConstant(tagBoundary);
  binary.matrix().topRows(1).array().setConstant(tagBoundary);
  binary.matrix().leftCols(1).array().setConstant(tagBoundary);
  binary.matrix().rightCols(1).array().setConstant(tagBoundary);

  Map<int32_t>::Index index;
  for (index.x() = 1; index.x() < static_cast<Map<int32_t>::Index::Scalar>(map.sizeInCellsX())-1; ++index.x()) {
    for (index.y() = 1; index.y() < static_cast<Map<int32_t>::Index::Scalar>(map.sizeInCellsY())-1; ++index.y())
      binary(index) = map(index) != map(index.x(), index.y()+1) || map(index) != map(index.x()+1, index.y());
  }
}

// Explicit template instantiation
extern template void changeDetection<uint8_t>(const Map<uint8_t>& map, Map<bool>& binary, bool tagBoundary = true);
extern template void changeDetection<float>(const Map<float>& map, Map<bool>& binary, bool tagBoundary = true);
extern template void changeDetection<double>(const Map<double>& map, Map<bool>& binary, bool tagBoundary = true);

} /* namespace algorithms */
} /* namespace planning2d */


#endif /* INCLUDE_PLANNER_ALGORITHMS_IMPLEMENTATION_MAPCHANGEDETECTIONIMPL_HPP_ */
