/*! \file examples/tripleobjective_shortest_path/main.cpp
 *  \brief The main program using RandomGraphProblem's computeConvexParetoSet() 
 *         to solve a tripleobjective shortest path problem.
 *  \author Christos Nitsas
 *  \date 2012
 *  
 *  We use a tripleobjective_shortest_path::RandomGraphProblem object to 
 *  represent a tripleobjective shortest path problem on a random boost graph.
 *  
 *  All typedefs and class declarations are inside 
 *  examples/tripleobjective_shortest_path/tripleobjective_shortest_path_example_common.h.
 *  
 *  \sa tripleobjective_shortest_path_example::RandomGraphProblem and 
 *      pareto_approximator::BaseProblem
 */


#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <ctime>
#include <list>
#include <boost/graph/adjacency_list.hpp>

#include "tripleobjective_shortest_path_example_common.h"
#include "RandomGraphProblem.h"


using std::cout;
using std::endl;

using pareto_approximator::Point;
using pareto_approximator::PointAndSolution;
using pareto_approximator::NonDominatedSet;
using tripleobjective_shortest_path_example::RandomGraphProblem;
using tripleobjective_shortest_path_example::PredecessorMap;



/*!
 *  \addtogroup TripleobjectiveShortestPathExample An example tripleobjective shortest path problem.
 *  
 *  @{
 */


//! Check if a specific command line option exists.
bool 
commandLineOptionExists(char ** begin, char ** end, 
                        const std::string & option)
{
  return std::find(begin, end, option) != end;
}


//! Get a specific command line argument if it exists.
char * 
getCommandLineArgument(char ** begin, char ** end, 
                       const std::string & option)
{
  char ** it = std::find(begin, end, option);
  // if the option exists and is followed by an argument return the argument
  if (it != end and ++it != end)
    return *it;
  // else 
  return NULL;
}


//! The example's main function.
/*!
 *  Will make a RandomGraphProblem instance and, if t is reachable, will 
 *  find a convex Pareto set and print its size, its points and the 
 *  problem solutions corresponding to those points.
 */
int 
main(int argc, char * argv[])
{
  // Parse the command line arguments.
  int seed;
  bool withoutExactParetoSet = false;
  char * arg = NULL;
  if (commandLineOptionExists(argv, argv + argc, "-h") or
      commandLineOptionExists(argv, argv + argc, "--help")) {
    cout << "Usage: tosp_example [-s seed] [--without-exact-pareto-set]" 
         << endl;
    return 0;
  }
  // else 
  if (commandLineOptionExists(argv, argv + argc, "-W") or 
      commandLineOptionExists(argv, argv + argc, 
      "--without-exact-pareto-set")) {
    // compute the approximate Pareto set only
    withoutExactParetoSet = true;
  }
  arg = getCommandLineArgument(argv, argv + argc, "-s");
  if (arg != NULL)
    // Use the input argument as a seed. 
    // Use 0 if the input argument is not an integer.
    seed = atoi(arg);
  else 
    // Use the current time as a seed.
    seed = time(0);

  // Initializations
  // =========================================
  // Make a RandomGraphProblem instance with numVertices vertices and 
  // numEdges edges.
  // - "black" edge weights should be random integers in 
  //   [minBlackWeight, maxBlackWeight] (uniformly distributed)
  // - "red" edge weights should be random integers in 
  //   [minRedWeight, maxRedWeight] (uniformly distributed)
  // - "green" edge weights should be random integers in 
  //   [minGreenWeight, maxGreenWeight] (uniformly distributed)
  // Reminder: All instances are created randomly so even instances with 
  // the same number of vertices and edges will almost surely be different.
  int numVertices = 100;
  int numEdges = 800;
  int minBlackWeight = 1;
  int maxBlackWeight = 100;
  int minRedWeight = 1;
  int maxRedWeight = 100;
  int minGreenWeight = 1;
  int maxGreenWeight = 100;
  unsigned int numObjectives = 3;
  double approximationRatio = 1e-12;

  RandomGraphProblem rgp(numVertices,    numEdges, 
                         minBlackWeight, maxBlackWeight, 
                         minRedWeight,   maxRedWeight, 
                         minGreenWeight, maxGreenWeight, 
                         seed);

  // Print problem info.
  cout << "Triple-objective shortest path problem:" << endl
       << "- undirected random boost graph with no parallel edges" << endl
       << "- random number generator's seed: " << seed << endl
       << "- " << numVertices << " vertices and " << numEdges << " edges" 
       << endl
       << "- three weights (\"black\", \"red\" and \"green\") on each edge" 
       << endl
       << "- \"black\" edge weights: random integers drawn uniformly from" 
       << " [" << minBlackWeight << ", " << maxBlackWeight << "]" << endl
       << "- \"red\" edge weights: random integers drawn uniformly from"
       << " [" << minRedWeight << ", " << maxRedWeight << "]" << endl
       << "- \"green\" edge weights: random integers drawn uniformly from"
       << " [" << minGreenWeight << ", " << maxGreenWeight <<"]" << endl
       << "- three objective functions to minimize: " << endl
       << "  (let P be an s-t path)" << endl
       << "  + Black(P): sum of \"black\" weights of all edges in P" << endl
       << "  + Red(P): sum of \"red\" weights of all edges in P" << endl
       << "  + Green(P): sum of \"green\" weights of all edges in P" << endl
       << "- find an eps-approximate convex Pareto set with " 
       << "eps = " << approximationRatio << endl << endl;

  //cout << "Printing graph as a dot file (graph.dot)..." << endl;
  //rgp.printGraphToDotFile();
  //cout << "Done!" << endl << endl;

  // Check whether or not t is reachable (from s).
  if (rgp.isTargetReachable())
    cout << "Vertex t is reachable." << endl << endl;
  else {
    // If it's not, don't bother trying to find shortest paths.
    cout << "Vertex t is not reachable! ";
    cout << "No point in continuing..." << endl << endl;
    return 1;
  }

  cout << "(computing approximate convex Pareto set... "
       << "please wait a few seconds)" << endl << endl;
  // All the work (essentially 2 lines!)
  // =========================================
  // Use RandomGraphProblem::computeConvexParetoSet() (inherited from 
  // BaseProblem) to find the Pareto set.
  std::vector< PointAndSolution<PredecessorMap> > paretoSet;
  paretoSet = rgp.computeConvexParetoSet(numObjectives, approximationRatio);

  // Output (convex Pareto set)
  // =========================================
  cout << "A. approximate convex Pareto set size: " << paretoSet.size() << endl;
  cout << endl << "B. approximate convex Pareto set points: " << endl;
  cout << "   (they are all points of the exact Pareto set)" << endl;
  std::vector< PointAndSolution<PredecessorMap> >::iterator vi;
  // Print each Pareto optimal point and the corresponding solution (path).
  for (vi = paretoSet.begin(); vi != paretoSet.end(); ++vi) {
    cout << vi->point << endl;
    rgp.printPath(vi->solution);
  }

  // Should we also compute and print the exact Pareto set?
  if (not withoutExactParetoSet) {
    // Exact Pareto set
    // =========================================
    cout << "\n(computing exact Pareto set... please wait a few seconds)\n" << endl;
    NonDominatedSet<Point> exactParetoSet = rgp.computeExactParetoSet();
    cout << "C. exact Pareto set size: " << exactParetoSet.size() << endl;
    cout << "\nD. exact Pareto set points: " << endl;
    NonDominatedSet<Point>::iterator epsi;
    for (epsi = exactParetoSet.begin(); epsi != exactParetoSet.end(); ++epsi)
      cout << *epsi << endl;
  }

  return 0;
}


/*! 
 *  @}
 */


