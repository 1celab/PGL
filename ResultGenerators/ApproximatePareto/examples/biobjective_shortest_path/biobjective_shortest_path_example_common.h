/*! \file examples/biobjective_shortest_path/biobjective_shortest_path_example_common.h
 *  \brief Common classes and typedefs used throughout the example.
 *  \author Christos Nitsas
 *  \date 2012
 */


#ifndef BIOBJECTIVE_SHORTEST_PATH_EXAMPLE_COMMON_H
#define BIOBJECTIVE_SHORTEST_PATH_EXAMPLE_COMMON_H


#include <boost/graph/adjacency_list.hpp>

// CLEANUP CHANGE
#include <Algorithms/Pareto/ApproximatePareto/Point.h>
#include <Algorithms/Pareto/ApproximatePareto/PointAndSolution.h>
#include <Algorithms/Pareto/ApproximatePareto/BaseProblem.h>
#include <Algorithms/Pareto/ApproximatePareto/NonDominatedSet.h>
//#include "../../Point.h"
//#include "../../PointAndSolution.h"
//#include "../../BaseProblem.h"
//#include "../../NonDominatedSet.h"


/*!
 *  \defgroup BiobjectiveShortestPathExample An example biobjective shortest path problem.
 *  
 *  @{
 */


//! Everything needed for the example biobjective shortest path problem.
namespace biobjective_shortest_path_example {


//! A boost graph bundled edge property.
class EdgeProperty 
{
  public:
    //! The edge's "black" weight.
    double black;
    //! The edge's "red" weight.
    double red;
    //! Edge label for printing the graph to a dot file.
    /*!
     *  The edge label is just the two weights inside parentheses. e.g.:
     *  - (10, 15)
     *  - (36, 17)
     */
    std::string label;
};


//! A boost adjacency_list representing an undirected graph.
/*!
 *  - vertices are stored in a std::vector
 *  - no parallel edges
 *  - vertices have a color property (for BFS search)
 *  - edges have a bundled property (class EdgeProperty)
 */
typedef boost::adjacency_list<boost::setS, 
                              boost::vecS, 
                              boost::undirectedS, 
                              boost::no_property,
                              EdgeProperty>           Graph;
//! A graph vertex. 
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
//! A graph vertex iterator.
typedef boost::graph_traits<Graph>::vertex_iterator   VertexIterator;
//! A graph edge.
typedef boost::graph_traits<Graph>::edge_descriptor   Edge;
//! A graph edge iterator.
typedef boost::graph_traits<Graph>::edge_iterator     EdgeIterator;
//! A simple predecessor map.
/*!
 *  Maps a vertex to its predecessor. (e.g. in a shortest path)
 *  - Implemented using a std::vector of vertices since vertex descriptors 
 *    are integers. If the vertex storage type changes (in the Graph typedef) 
 *    this must change too.
 */
typedef std::vector<Vertex>                    PredecessorMap;


}  // namespace biobjective_shortest_path_example


/*! 
 *  @}
 */


#endif  // BIOBJECTIVE_SHORTEST_PATH_EXAMPLE_COMMON_H
