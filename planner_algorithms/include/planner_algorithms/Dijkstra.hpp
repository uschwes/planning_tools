/*
 * Dijkstra.hpp
 *
 *  Created on: 07.04.2016
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PLANNER_INTERFACES_ALGORITHMS_DIJKSTRA_HPP_
#define INCLUDE_PLANNER_INTERFACES_ALGORITHMS_DIJKSTRA_HPP_

// standard
#include <vector>
#include <string>
#include <list>

#include <limits> // for numeric_limits

#include <set>
#include <map>
#include <queue>
#include <utility> // for pair
#include <algorithm>
#include <iterator>
#include <type_traits>

// Schweizer Messer
#include <sm/logging.hpp>

// self
#include <planner_interfaces/Exceptions.hpp>
#include <planner_interfaces/Support.hpp>
#include <planner_interfaces/OccupancyGrid.hpp>

namespace planning2d
{
namespace algorithms
{
namespace dijkstra
{

/**
 * \class OccupancyGridConnectivity4d
 * \brief Functor for a dynamic 4-connectivity on a planning2d::OccupancyGrid
 * \tparam Scalar Scalar type of occupancy grid map
 */
template <typename Scalar = double>
struct OccupancyGridConnectivity4d
{
 protected:
  static_assert(std::is_integral<Scalar>::value || std::is_floating_point<Scalar>::value, "");
  typedef std::pair<OccupancyGrid::Index, Scalar> Item; // increment/cost pair

 public:
  inline OccupancyGridConnectivity4d(const OccupancyGrid& grid);
  virtual ~OccupancyGridConnectivity4d() { }
  virtual inline const std::vector<Item>& operator()(const OccupancyGrid::Index& index);
 protected:
  const OccupancyGrid& _grid;
  OccupancyGrid::Index _increments[4];
  std::vector<Item> _neighbors;
};

/**
 * \class OccupancyGridConnectivity8d
 * \brief Functor for a dynamic 8-connectivity on a planning2d::OccupancyGrid
 * \tparam Scalar Scalar type of occupancy grid map
 */
template <typename Scalar = double>
struct OccupancyGridConnectivity8d
{
 protected:
  static_assert(std::is_integral<Scalar>::value || std::is_floating_point<Scalar>::value, "");
  typedef std::pair<OccupancyGrid::Index, Scalar> Item; // increment/cost pair

 public:
  inline OccupancyGridConnectivity8d(const OccupancyGrid& grid);
  virtual ~OccupancyGridConnectivity8d() { }
  virtual inline const std::vector<Item>& operator()(const OccupancyGrid::Index& index);
 protected:
  const OccupancyGrid& _grid;
  Item _increments[8];
  std::vector<Item> _neighbors;
};

template <typename Index, typename Scalar = double>
struct Vertex
{
  static_assert(std::is_integral<Scalar>::value || std::is_floating_point<Scalar>::value, "");
  inline Vertex(const Index& id_, const Scalar cost_);
  inline bool operator<(const Vertex& v) const;
  Index id;
  Scalar cost;
};

namespace internal
{
  /**
   * \class SetAllocFixed
   * Custom fixed-size memory allocator for STL set in case the maximum heap size during Dijkstra
   * runtime is surely known before.
   */
  template<class T, size_t S>
  class SetAllocFixed: public std::allocator<T>
  {
    T *buf;
    size_t ptr;
   public:
    inline SetAllocFixed();
    inline ~SetAllocFixed();
    inline T* allocate(size_t n, std::allocator<void>::const_pointer hint=0);
    inline void deallocate(T* p, size_t n);

    template<class T1>
    struct rebind { typedef SetAllocFixed<T1, S> other; };
  };

} /* namespace internal */

} /* namespace dijkstra  */

/**
 * \class Dijkstra
 * \tparam Index type of the index
 * \tparam Scalar Scalar type of cost
 * \tparam PriorityQueue type of container used for the priority queue
 * \tparam AdjacencyFunctor Functor type to return the neighbors of an node
 *                          Must support the signature iterable = AdjacencyFunctor(Index)
 *                          iterable must return a pair<Index,weight>
 *
 * \brief Dijkstra's algorithm
 */
template <typename Index, typename Costs, typename AdjacencyFunctor, typename Scalar = double,
    template <typename, typename...> class PriorityQueue = std::set, typename Allocator = std::allocator<dijkstra::Vertex<Index, Scalar> > >
class Dijkstra
{
 private:
  typedef dijkstra::Vertex<Index, Scalar> vertex_t;
  typedef PriorityQueue<vertex_t, std::less<vertex_t>, Allocator> queue_t;

 public:
  inline Dijkstra(const Index& source, AdjacencyFunctor neighborsFcn, Costs& costs, Index const * const goal = nullptr);

  /// \brief Execute one step meaning expand the lowest cost node on the heap
  inline bool step();

  /// \brief Run until end
  inline bool run();

 private:
  Index _source;
  AdjacencyFunctor _neighborsFcn;
  Costs& _costs;
  Index const * _goal;
  queue_t _queue;
};

namespace dijkstra
{

/**
 * Follows the gradient of the cost map computed by Dijkstra's algorithm to extract the shortest path
 * @param[in] neighborsFcn Adjacency functor
 * @param[in] costs Cost/geodesic distance map
 * @param[in] source Source index to start the search from. Usually this is the start index if you did the expansion in Dijkstra's algorithm from the goal index.
 * @param[out] path Least cost path
 */
template <typename Index, typename Costs, typename AdjacencyFunctor, template <typename, typename...> class Path = std::vector>
void extractBestPath(AdjacencyFunctor neighborsFcn, const Costs& costs, const Index& source, Path<Index>& path);

} /* namespace dijkstra */

/// \brief Wrapper function for automatic template argument deduction
template <typename Scalar = double, template <typename, typename...> class PriorityQueue = std::set, typename Index, typename Costs, typename AdjacencyFunctor>
auto make_dijkstra(const Index& source, AdjacencyFunctor neighborsFcn, Costs& costs, Index const * const goal = nullptr) -> decltype(Dijkstra<Index, Costs, AdjacencyFunctor, Scalar, PriorityQueue>(source, neighborsFcn, costs, goal))
{
  return Dijkstra<Index, Costs, AdjacencyFunctor, Scalar, PriorityQueue>(source, neighborsFcn, costs, goal);
}


//std::list<vertex_t> DijkstraGetShortestPathTo(vertex_t vertex, const std::vector<vertex_t> &previous)
//{
//  std::list<vertex_t> path;
//  for ( ; vertex != -1; vertex = previous[vertex])
//    path.push_front(vertex);
//  return path;
//}

} /* namespace algorithms */
} /* namespace planning2d */

#include "implementation/DijkstraImpl.hpp"

#endif /* INCLUDE_PLANNER_INTERFACES_ALGORITHMS_DIJKSTRA_HPP_ */
