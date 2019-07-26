/*
 * DijkstraImpl.hpp
 *
 *  Created on: 07.04.2016
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PLANNER_INTERFACES_ALGORITHMS_IMPLEMENTATION_DIJKSTRAIMPL_HPP_
#define INCLUDE_PLANNER_INTERFACES_ALGORITHMS_IMPLEMENTATION_DIJKSTRAIMPL_HPP_


namespace planning2d
{
namespace algorithms
{
namespace dijkstra
{

#define _TEMPLATE template <typename Scalar>
#define _CLASS OccupancyGridConnectivity4d<Scalar>

_TEMPLATE
_CLASS::OccupancyGridConnectivity4d(const OccupancyGrid& grid)
    : _grid(grid),
      _increments(
          {
            OccupancyGrid::Index(-1,+0),
            OccupancyGrid::Index(+0,-1),
            OccupancyGrid::Index(+1,+0),
            OccupancyGrid::Index(+0,+1)
          }
      )
{
  _neighbors.reserve(4);
}

_TEMPLATE
const std::vector<typename _CLASS::Item>& _CLASS::operator()(const OccupancyGrid::Index& index)
{
  _neighbors.clear();
  for (std::size_t i=0; i<4; ++i) {
    OccupancyGrid::Index newIdx = index + _increments[i];
    if (_grid.isInsideMap(newIdx) && !_grid.isOccupied(newIdx))
      _neighbors.push_back( Item(newIdx, (Scalar)_grid.resolution()) );
  }
  return _neighbors;
}

#undef _TEMPLATE
#undef _CLASS

#define _TEMPLATE template <typename Scalar>
#define _CLASS OccupancyGridConnectivity8d<Scalar>

_TEMPLATE
_CLASS::OccupancyGridConnectivity8d(const OccupancyGrid& grid)
    : _grid(grid),
      _increments(
          {
            Item(OccupancyGrid::Index(-1,+0), (Scalar)_grid.resolution()),
            Item(OccupancyGrid::Index(+0,-1), (Scalar)_grid.resolution()),
            Item(OccupancyGrid::Index(+1,+0), (Scalar)_grid.resolution()),
            Item(OccupancyGrid::Index(+0,+1), (Scalar)_grid.resolution()),
            Item(OccupancyGrid::Index(-1,-1), (Scalar)(sqrt((Scalar(2))))*_grid.resolution()),
            Item(OccupancyGrid::Index(+1,-1), (Scalar)(sqrt((Scalar(2))))*_grid.resolution()),
            Item(OccupancyGrid::Index(-1,+1), (Scalar)(sqrt((Scalar(2))))*_grid.resolution()),
            Item(OccupancyGrid::Index(+1,+1), (Scalar)(sqrt((Scalar(2))))*_grid.resolution())
          }
      )
{
  _neighbors.reserve(8);
}

_TEMPLATE
const std::vector<typename _CLASS::Item>& _CLASS::operator()(const OccupancyGrid::Index& index)
{
  _neighbors.clear();
  for (std::size_t i=0; i<8; ++i) {
    OccupancyGrid::Index newIdx = index + _increments[i].first;
    if (_grid.isInsideMap(newIdx) && !_grid.isOccupied(newIdx))
      _neighbors.push_back( Item(newIdx, _increments[i].second) );
  }
  return _neighbors;
}

#undef _TEMPLATE
#undef _CLASS

#define _TEMPLATE template <typename Index, typename Scalar>
#define _CLASS Vertex<Index, Scalar>

_TEMPLATE
_CLASS::Vertex(const Index& id_, const Scalar cost_)
    : id(id_), cost(cost_)
{
}

_TEMPLATE
bool _CLASS::operator<(const Vertex& v) const
{
  return cost != v.cost ? cost < v.cost : id != v.id;
}


#undef _TEMPLATE
#undef _CLASS

namespace internal
{

#define _TEMPLATE template<class T, size_t S>
#define _CLASS SetAllocFixed<T,S>

_TEMPLATE
_CLASS::SetAllocFixed()
{
  buf = (T*)malloc(sizeof(T) * S);
  ptr = 0;
}

_TEMPLATE
_CLASS::~SetAllocFixed()
{
  free(buf);
}

_TEMPLATE
T* _CLASS::allocate(size_t n, std::allocator<void>::const_pointer hint /*=0*/)
{
  ptr += n;
  return &buf[ptr - n];
}

_TEMPLATE
void _CLASS::deallocate(T* p, size_t n)
{
  /*Do nothing.*/
}

#undef _TEMPLATE
#undef _CLASS

} /* namespace internal */

} /* namespace dijkstra  */


#define _TEMPLATE template <typename Index, typename Costs, typename AdjacencyFunctor, typename Scalar, \
  template <typename, typename...> class PriorityQueue, typename Allocator>
#define _CLASS Dijkstra<Index, Costs, AdjacencyFunctor, Scalar, PriorityQueue, Allocator>

_TEMPLATE
_CLASS::Dijkstra(const Index& source, AdjacencyFunctor neighborsFcn, Costs& costs, Index const * const goal /*= nullptr*/)
    : _source(source), _neighborsFcn(neighborsFcn), _costs(costs), _goal(goal)
{
  _queue.insert(vertex_t(source, (Scalar)0)); // insert source with cost 0.0
  _costs(source) = (Scalar)0;
}

_TEMPLATE
bool _CLASS::step()
{
  bool done = false;

  if(_queue.empty())
    return done = true;

  const vertex_t u = *_queue.begin();

  if (_goal != nullptr && u.id == *_goal) {
    SM_DEBUG_STREAM_NAMED("dijkstra", "Goal" << *_goal << " reached, stopping.");
    return done = true;
  }

  // Visit each edge exiting u
  const auto& neighbors = _neighborsFcn(u.id);
  SM_VERBOSE_STREAM_NAMED("dijkstra", "Expanding node" << u.id << " with " << neighbors.size() <<
                          " neighbors (heap size: " << _queue.size() << ") ");

  _queue.erase(_queue.begin());

  for (const auto& neighbor : neighbors)
  {
    const auto idV = neighbor.first;
    const auto edgeWeight = neighbor.second;
    const auto costThroughU = u.cost + edgeWeight;
    const Scalar costV = _costs(idV);

    SM_VERBOSE_STREAM_NAMED("dijkstra", "cost" << idV << " = " << costV << " -> " <<
                            u.cost << " + " << edgeWeight << " = " << costThroughU);

    if (costThroughU < costV)
    {
      _costs(idV) = costThroughU;
      _queue.erase(vertex_t(idV, costV)); // container must accept non-present key
      _queue.insert(vertex_t(idV, costThroughU));
    }
  }
  return done;
}

_TEMPLATE
bool _CLASS::run()
{
  bool done = false;
  while (!done)
    done = this->step();
  return done;
}

#undef _TEMPLATE
#undef _CLASS

namespace dijkstra {

template <typename Index, typename Costs, typename AdjacencyFunctor, template <typename, typename...> class Path = std::vector>
void extractBestPath(AdjacencyFunctor neighborsFcn, const Costs& costs, const Index& source, Path<Index>& path) {
  path.clear();
  path.push_back(source);
  SM_VERBOSE_STREAM_NAMED("dijkstra", "Adding node " << path.back() << " with cost " << costs(path.back()) << " to path.");
  while (true) {
    const auto& neighbors = neighborsFcn(path.back());
    const Index* next = nullptr;
    auto minCost = std::numeric_limits<typename Costs::Scalar>::infinity();
    for (const auto& neighbor : neighbors) {
      if (costs(neighbor.first) < std::min(costs(path.back()), minCost)) {
        next = &neighbor.first;
        minCost = costs(neighbor.first);
      }
    }
    if (next != nullptr) {
      path.push_back(*next);
      SM_VERBOSE_STREAM_NAMED("dijkstra", "Adding node " << path.back() << " with cost " << costs(path.back()) << " to path.");
    } else {
      SM_VERBOSE_STREAM_NAMED("dijkstra", "Goal " << path.back() << " reached");
      break;
    }
  }
}

} /* namespace dijkstra  */

} /* namespace algorithms */
} /* namespace planning2d */

#endif /* INCLUDE_PLANNER_INTERFACES_ALGORITHMS_IMPLEMENTATION_DIJKSTRAIMPL_HPP_ */
