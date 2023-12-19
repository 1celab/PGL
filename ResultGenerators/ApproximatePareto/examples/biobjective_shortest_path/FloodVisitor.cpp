/*! \file examples/biobjective_shortest_path/FloodVisitor.cpp
 *  \brief The implementation of the FloodVisitor class.
 *  \author Christos Nitsas
 *  \date 2012
 *  
 *  Won't `include` FloodVisitor.h. In fact FloodVisitor.h will `include` 
 *  FloodVisitor.cpp because we want a header-only code base.
 */


#include <limits>

#include "biobjective_shortest_path_example_common.h"


using pareto_approximator::Point;
using pareto_approximator::NonDominatedSet;


/*!
 *  \addtogroup BiobjectiveShortestPathExample An example biobjective shortest path problem.
 *  
 *  @{
 */


//! Everything needed for the example biobjective shortest path problem.
namespace biobjective_shortest_path_example {


//! A simple constructor.
/*!
 *  \param source The source vertex of the graph.
 *  \param target The target vertex of the graph.
 *  \param numVertices The number of vertices in the graph.
 */
FloodVisitor::FloodVisitor(const Vertex & source, 
                           const Vertex & target, 
                           unsigned int numVertices) : source_(source), 
                                                       target_(target)
{
  vertexDistances_.assign(numVertices, NonDominatedSet<Point>());
}


//! Initialize a vertex's vertexDistances_.
/*!
 *  \param u A vertex of the graph.
 *  \param g The graph.
 *  
 *  \sa FloodVisitor and FloodVisitor::vertexDistances_.
 */
void 
FloodVisitor::initializeVertex(const Vertex & u, const Graph &)
{
  if (u == source_) {
    // initialize the source's distance to [1.0, 1.0] because (currently) 
    // we do not allow not strictly positive points like [0.0, 0.0]
    // inside Point::dominates() and Point::distance() (because of 
    // Point::ratioDistance())
    vertexDistances_[u].insert(Point(1.0, 1.0));
  }
  else
    vertexDistances_[u].insert(Point(std::numeric_limits<double>::max(), 
                                     std::numeric_limits<double>::max()));
}


//! Broadcast distances over edge.
/*!
 *  \param e An edge of the graph.
 *  \param g The graph.
 *  \return true if some of v's distances actually needed an update;
 *          false if nothing changed.
 *  
 *  Let's call e's source u and e's target v. Update v's distances 
 *  using u's distances plus e's weights.
 *  
 *  \sa FloodVisitor, FloodVisitor::vertexDistances_ and EdgeProperty.
 */
bool 
FloodVisitor::broadcastDistances(const Edge & e, const Graph & g)
{
  Vertex u = boost::source(e, g);
  Vertex v = boost::target(e, g);
  Point edgeWeight(g[e].black, g[e].red);

  bool insertedNewDistance = false;
  NonDominatedSet<Point>::iterator udi;
  for (udi = vertexDistances_[u].begin(); 
       udi != vertexDistances_[u].end(); ++udi) {
    insertedNewDistance |= vertexDistances_[v].insert(*udi + edgeWeight);
  }

  return insertedNewDistance;
}


//! Get the exact Pareto set as recorded in the visitor.
/*!
 *  \return The exact Pareto set (as recorded in the visitor).
 *
 *  The Pareto set will be vertexDistances_[target_].
 */
NonDominatedSet<Point> 
FloodVisitor::getParetoPoints()
{
  // Remember that we had initialized the source to [1.0, 1.0] instead 
  // of [0.0, 0.0]. We now have to subtract [1.0, 1.0] from each of the 
  // target vertex's distances to get the Pareto points.
  NonDominatedSet<Point>::iterator udi;
  NonDominatedSet<Point> paretoPoints;
  for (udi = vertexDistances_[target_].begin();
       udi != vertexDistances_[target_].end(); ++udi) {
    paretoPoints.insert(*udi - Point(1.0, 1.0));
  }
  return paretoPoints;
}


}  // namespace biobjective_shortest_path_example


/*!
 *  @}
 */
