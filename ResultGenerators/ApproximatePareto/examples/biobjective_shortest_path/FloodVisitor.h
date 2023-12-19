/*! \file examples/biobjective_shortest_path/FloodVisitor.h
 *  \brief The definition of the FloodVisitor class.
 *  \author Christos Nitsas
 *  \date 2012
 */


#ifndef EXAMPLE_CLASS_FLOOD_VISITOR_H
#define EXAMPLE_CLASS_FLOOD_VISITOR_H


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


//! A custom visitor for our flood algorithm.
/*!
 *  It floods the graph with 2-dimensional distances from the source s 
 *  and records all possible paths except paths that are definitely 
 *  not on the Pareto set (dominated paths).
 */
class FloodVisitor 
{
  public:
    //! A simple constructor.
    /*!
     *  \param source The source vertex of the graph.
     *  \param target The target vertex of the graph.
     *  \param numVertices The number of vertices in the graph.
     */
    FloodVisitor(const Vertex & source, const Vertex & target, 
                     unsigned int numVertices);

    //! Initialize a vertex's vertexDistances_.
    /*!
     *  \param u A vertex of the graph.
     *  \param g The graph.
     *  
     *  \sa FloodVisitor and FloodVisitor::vertexDistances_.
     */
    void initializeVertex(const Vertex & u, const Graph & g);

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
    bool broadcastDistances(const Edge & e, const Graph & g);

    //! Get the exact Pareto set as recorded in the visitor.
    /*!
     *  \return The exact Pareto set (as recorded in the visitor).
     *
     *  The Pareto set will be vertexDistances_[target_].
     */
    NonDominatedSet<Point> getParetoPoints();

  private:
    //! The source vertex. (for the shortest path problem)
    Vertex source_;

    //! The target vertex. (for the shortest path problem)
    Vertex target_;

    //! A vector of distances for each vertex. (distances from source_)
    /*!
     *  A vertex can have multiple distances, one for each path from 
     *  the source vertex. We will not be considering long paths 
     *  i.e. paths with cycles or paths that are definitely worse than 
     *  other paths we have already discovered.
     *
     *  We represent distances as Point instances.
     */
    std::vector< NonDominatedSet<Point> > vertexDistances_;
};


}  // namespace biobjective_shortest_path_example


/*! 
 *  @}
 */


// We will #include the implementation here because we want to make a 
// header-only code base.
#include "FloodVisitor.cpp"


#endif  // EXAMPLE_CLASS_FLOOD_VISITOR_H
