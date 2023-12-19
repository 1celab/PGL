//! \file experiments/vs_namoa_star/MultiobjectiveSpOnPmgProblem.h
//! \brief The MultiobjectiveSpOnPmgProblem class. It represents a 
//!        multi-objective shortest path problem on a Packed-memory graph.
//! \author Christos Nitsas
//! \date 2013
//!


#ifndef EXPERIMENTS_VS_NAMOA_STAR_MULTIOBJECTIVE_SP_ON_PMG_PROBLEM_H
#define EXPERIMENTS_VS_NAMOA_STAR_MULTIOBJECTIVE_SP_ON_PMG_PROBLEM_H


#include <algorithm>
#include <iterator>
#include <limits>
#include <utility>
#include <string>
#include <sstream>
#include <cmath>

#include <Utilities/timer.h>
#include <Utilities/graphIO.h>
#include <Utilities/progressBar.h>
#include <Utilities/mersenneTwister.h>

// CLEANUP CHANGE
#include "pgl-modified/multicriteriaDijkstra.h"
#include "pgl-modified/namoaStar.h"
#include "pgl-modified/dijkstra.h"
#include "pgl-modified/aStarDijkstra.h"
//#include <Algorithms/ShortestPath/Multicriteria/multicriteriaDijkstra.h>
//#include <Algorithms/ShortestPath/Multicriteria/namoaStar.h>
//#include <Algorithms/ShortestPath/dijkstra.h>
//#include <Algorithms/ShortestPath/aStarDijkstra.h>

// CLEANUP CHANGE
#include <Algorithms/Pareto/ApproximatePareto/Point.h>
#include <Algorithms/Pareto/ApproximatePareto/PointAndSolution.h>
#include <Algorithms/Pareto/ApproximatePareto/BaseProblem.h>
//#include "../../Point.h"
//#include "../../PointAndSolution.h"
//#include "../../BaseProblem.h"

#include "experiments_vs_namoa_star_common.h"
#include "experiments_vs_namoa_star_utility.h"
#include "AStarDijkstra.h"


namespace pa = pareto_approximator;


//! 
//! \addtogroup ExperimentsVsNamoaStar Experiments vs the NAMOA Star algorithm.
//! 
//! @{
//!


//! Everything needed for the experiments vs the NAMOA\* algorithm.
namespace experiments_vs_namoa_star {


//! 
//! \brief A class representing a multi-objective shortest path (SP) problem 
//!        on a Packed-Memory Graph.
//! 
//! Problem info
//! --------------------
//! A class representing a multi-objective shortest path problem on a 
//! packed-memory graph. (we use PGL's packed-memory graph implementation)
//! 
//! The problem loads maps with three different edge weights, which are 
//! related to three problem objectives but the user can choose to use 
//! either all three objectives (triple-objective problem) or just the 
//! first two (biobjective problem).
//!
//! More info:
//! - Edges have three (different) weights, "distance", "travel time" and 
//!   "hop" (always equal to 1) which are related to three problem objectives
//!   (one each).
//! - Two vertices, the source (s) and target (t) are singled out each 
//!   time.
//! - A problem solution is a path from s to t. 
//! - The problem can use either only the first two or all three objectives.
//!   The users can choose if they want to solve the biobjective or the 
//!   triple-objective problem when they call runQuery().
//! 
//! The goal is to find a path that simultaneously minimizes the objective 
//! functions, which we will call Distance, TravelTime and NumberOfHops. 
//! They are:
//! - Distance: The sum of the "distance" weights for all the path's edges.
//! - TravelTime: The sum of the "travel time" weights for all the path's 
//!               edges.
//! - NumberOfHops: The sum of the "hop" weights for all the path's edges.
//!   Since "hop" will be equal to 1 for every edge, NumberOfHops counts 
//!   the number of edges in the path.
//! 
//! It is easy to see that every problem solution corresponds to a point 
//! in objective space. More often than not, there will be no single point 
//! (solution) that dominates every other one but a set of optimal points
//! (solutions) which is called a Pareto set. 
//! - MultiobjectiveSpOnPmgProblem::computeConvexParetoSet() will 
//!   try to find a set of points whose convex combinations approximately 
//!   dominate every point in the problem's Pareto set (the set of 
//!   non-dominated points).
//! 
//! Reminder: The user can set the degree of approximation when he calls 
//! MultiobjectiveSpOnPmgProblem::computeConvexParetoSet(). We are going 
//! to be using a default value in this class, though.
//! 
//! What we had to do for computeConvexParetoSet() to work
//! --------------------------------------------------------
//! MultiobjectiveSpOnPmgProblem instances inherit 
//! BaseProblem::computeConvexParetoSet() directly from BaseProblem so the 
//! only thing we had to implement was the 
//! MultiobjectiveSpOnPmgProblem::comb() method (!) which is 
//! declared virtual in BaseProblem. 
//! 
//! About PGL and the NAMOA\* algorithm.
//! -------------------------------------
//! PGL is a library of efficient graph structures and algorithms for large
//! scale networks. We use its Packed-Memory graph implementation and its 
//! implementation of the A* (inside comb()) and NAMOA\* algorithms in order 
//! to compare (our implementation of) the Chord algorithm for approximating 
//! the convex hull of the Pareto set with the NAMOA\* algorithm.
//! 
//! We also use various other helpful functions/classes from PGL, e.g. to 
//! read files representing a graph using the 9th DIMACS challenge format 
//! and set up the respective Packed-Memory graph.
//! 
//! For more info on the PGL library or to download PGL please visit the 
//! following web page: 
//! http://www.ceid.upatras.gr/faculty/zaro/software/pgl/index.html
//! 
//! \sa pareto_approximator::BaseProblem, 
//!     pareto_approximator::PointAndSolution and 
//!     pareto_approximator::Point
//!
class MultiobjectiveSpOnPmgProblem : private pa::BaseProblem<Path>
{
  public:
    //! \brief Constructor. Do not read any graph.
    //!
    //! The MultiobjectiveSpOnPgmProblem::readDimacs9Graph() or 
    //! MultiobjectiveSpOnPgmProblem::readDimacs10Graph() method must 
    //! be called at some point, to read a graph.
    //!
    MultiobjectiveSpOnPmgProblem() : timestamp_(0), noGraph_(true), 
                                     numCallsToComb_(0), 
                                     timeSpentInComb_(0.0), 
                                     usingDimacs9Graph_(true), 
                                     useAStar_(false)
    {
    }

    //! \brief Constructor. Reads the graph from the given DIMACS-9 graph 
    //!        files.
    //!
    //! \param distanceFilename The distance graph's filename. (Dimacs 9 
    //!                         file format)
    //! \param travelTimeFilename The travel time graph's filename. (Dimacs 
    //!                           9 file format).
    //! \param coordinatesFilename The node coordinates file's name. (Dimacs 
    //!                            9 file format).
    //! 
    //! Simply calls MultiobjectiveSpOnPmgProblem::readDimacs9Graph() to 
    //! read the graph (i.e. initialize the graph_ member variable) 
    //! associated with the problem from a Dimacs 9 graph file.
    //!
    //! We can later change the associated graph with a new call to the 
    //! MultiobjectiveSpOnPmgProblem::readDimacs9Graph() method.
    //! 
    //! Please see http://www.dis.uniroma1.it/challenge9/format.shtml 
    //! for more information on the DIMACS 9 graph files' format.
    //!
    MultiobjectiveSpOnPmgProblem(const std::string & distanceFilename, 
                                 const std::string & travelTimeFilename, 
                                 const std::string & coordinatesFilename) 
                              : timestamp_(0), numCallsToComb_(0), 
                                timeSpentInComb_(0.0), 
                                useAStar_(false)
    {
      readDimacs9Graph(distanceFilename, travelTimeFilename, 
                       coordinatesFilename);
    }

    //! \brief Constructor. Reads the graph from the given DIMACS-10 graph 
    //!        files.
    //!
    //! \param graphFilename The graph's filename. (Dimacs 10 file format)
    //! \param coordinatesFilename The node coordinates file's name. (Dimacs 
    //!                            10 file format).
    //! 
    //! Simply calls MultiobjectiveSpOnPmgProblem::readDimacs10Graph() to 
    //! read the graph (i.e. initialize the graph_ member variable) 
    //! associated with the problem from a Dimacs 10 graph file.
    //!
    //! We can later change the associated graph with a new call to the 
    //! MultiobjectiveSpOnPmgProblem::readDimacs9Graph(), or 
    //! MultiobjectiveSpOnPmgProblem::readDimacs10Graph() method.
    //! 
    //! Please see http://www.cc.gatech.edu/dimacs10/archive/streets.shtml 
    //! for more information on the DIMACS 10 graph files.
    //!
    MultiobjectiveSpOnPmgProblem(const std::string & graphFilename, 
                                 const std::string & coordinatesFilename) 
                              : timestamp_(0), numCallsToComb_(0), 
                                timeSpentInComb_(0.0), 
                                useAStar_(false)
    {
      readDimacs10Graph(graphFilename, coordinatesFilename);
    }

    //! \brief Reads a DIMACS-9 graph from the given graph files.
    //!
    //! \param distanceFilename The distance graph's filename. (Dimacs 9 
    //!                         file format)
    //! \param travelTimeFilename The travel time graph's filename. (Dimacs 
    //!                           9 file format).
    //! \param coordinatesFilename The node coordinates file's name. (Dimacs 
    //!                            9 file format).
    //! 
    //! First clears the current graph.
    //! 
    //! Then uses PGL's DIMACS9DoubleReader class to read the graph (i.e. 
    //! set the graph_ member variable).
    //!
    //! Finally, it adds a third weight "hop" on each edge, which will be 
    //! the same for every edge (equal to 1).
    //! 
    //! Note: DIMACS 9 edge weights are given (we do not calculate them 
    //!       ourselves) and are integers. The length of an edge (u, v) 
    //!       (i.e. its cost in the first criterion "distance") is the (Great 
    //!       circle) distance between nodes u and v (using their coordinates) 
    //!       rounded down to the closest integer. This is not ok! Our 
    //!       heuristics (e.g. great circle distance between nodes s and t, 
    //!       in an s-t query) might overestimate the distance because of 
    //!       this. The heuristics are theoretically correct but this rounding 
    //!       down of edge weights might throw them off. To mitigate this 
    //!       problem, we underestimated the (actual) Great circle distance 
    //!       between nodes quite a bit when calculating our heuristics, plus 
    //!       we added checks about whether or not the resulting heuristics 
    //!       are admissible/consistent (elsewere), plus we added code below 
    //!       (currently commented out) to point out cases where the Great 
    //!       circle distance underestimation we did is still over the 
    //!       rounded down edge distance.
    //!
    //! Please see http://www.dis.uniroma1.it/challenge9/format.shtml 
    //! for more information on the DIMACS 9 graph files' format.
    //!
    void readDimacs9Graph(const std::string & distanceFilename, 
                          const std::string & travelTimeFilename, 
                          const std::string & coordinatesFilename)
    {
      // clear the problem's attributes (in case they are not already empty)
      graph_.clear();
      nodeIdsToDescriptors_.clear();
      timestamp_ = 0;

      // make a reader object
      DIMACS9DoubleReader<PmaGraph> reader(distanceFilename, 
                                           travelTimeFilename, 
                                           coordinatesFilename);

      // read the graph
      graph_.read(&reader);
      // PGL examples do this, so we do too:
      graph_.compress();

      NodeIterator u, lastNode;
      EdgeIterator e, lastEdge;

      // CLEANUP CHANGE
      /*
      // Point out cases where our Great circle distance underestimation for 
      // a pair of adjacent nodes u, v is over the rounded down edge length.
      NodeIterator v;
      double greatCircleUnderest;
      for (u = graph_.beginNodes(), lastNode = graph_.endNodes(); 
           u != lastNode; ++u) 
      {
        for (e = graph_.beginEdges(u), lastEdge = graph_.endEdges(u);
             e != lastEdge; ++e) 
        {
          v = graph_.target(e);
          greatCircleUnderest = greatCircleUnderestimate(double(u->x)/100000, double(u->y)/100000, double(v->x)/100000, double(v->y)/100000);
          if ( not (greatCircleUnderest < e->criteriaList[0]) ) {
            std::cout << "greatCircleUnderestimate(u,v) >= e->criteriaList[0])!\n" 
                      << "greatCircleUnderestimate(u,v): " << greatCircleUnderest 
                      << "\ne->criteriaList[0]: " << e->criteriaList[0] 
                      << std::endl;
          }
        }
      }
      */

#if OPTION_NUM_OBJECTIVES == 3
      // CHANGE-HERE
      // add a third weight ("hop") on each graph edge:
      InEdgeIterator k;
      for (u = graph_.beginNodes(), lastNode = graph_.endNodes(); 
           u != lastNode; ++u) 
      {
        for (e = graph_.beginEdges(u), lastEdge = graph_.endEdges(u);
             e != lastEdge; ++e) 
        {
          // the "hop" cost will be equal to 1 for every edge; this way 
          // the "NumberOfHops(P)" objective, which will be the sum of "hop" 
          // costs for all edges in a path P, will be equal to the number of 
          // edges (or hops) in the path
          e->criteriaList[2] = 1;
          // set the incoming edge's cost as well
          k = graph_.getInEdgeIterator(e);
          k->criteriaList[2] = e->criteriaList[2];
        }
      }
#endif

      // read the node-ids-to-node-descriptors mapping vector
      nodeIdsToDescriptors_ = reader.getIds();
      // remember that a graph has been read
      noGraph_ = false;
      // remember what type of graph we are using
      usingDimacs9Graph_ = true;
    }

    //! \brief Reads a DIMACS-10 graph from the given graph files.
    //!
    //! \param graphFilename The graph's filename. (Dimacs 10 file format)
    //! \param coordinatesFilename The node coordinates file's name. (Dimacs 
    //!                            10 file format).
    //! 
    //! First clears the current graph.
    //! 
    //! Then uses PGL's DIMACS10Reader class to read the graph (i.e. set the 
    //! graph_ member variable).
    //!
    //! Finally, it calculates and sets the edge weights. (uses node 
    //! coordinates)
    //! 
    //! Please see http://www.cc.gatech.edu/dimacs10/archive/streets.shtml 
    //! for more information on the DIMACS 10 graph files.
    //!
    void readDimacs10Graph(const std::string & graphFilename, 
                           const std::string & coordinatesFilename)
    {
      // clear the problem's attributes (in case they are not already empty)
      graph_.clear();
      nodeIdsToDescriptors_.clear();
      timestamp_ = 0;

      // make a reader object
      DIMACS10Reader<PmaGraph> reader(graphFilename, 
                                      coordinatesFilename);

      // read the graph
      graph_.read(&reader);
      // PGL examples do this, so we do too:
      graph_.compress();

      // calculate edge weights (the DIMACS10Reader does not set edge weights)
      unsigned int minSpeed = 50;
      unsigned int maxSpeed = 90;
      NodeIterator ui, vi, lastNode;
      EdgeIterator ei, lastEdge;
      InEdgeIterator ki;
      std::stringstream sstr;
      sstr << "Calculating weights of " << graph_.getNumEdges() << " edges";
      ProgressBar edgeProgress(graph_.getNumEdges(), sstr.str());
      MersenneTwister gen;
      for (ui = graph_.beginNodes(), lastNode = graph_.endNodes(); 
           ui != lastNode; ++ui) 
      {
        for (ei = graph_.beginEdges(ui), lastEdge = graph_.endEdges(ui);
             ei != lastEdge; ++ei) 
        {
          vi = graph_.target(ei);
          ki = graph_.getInEdgeIterator(ei);

          // set the first weight, "distance", of each edge:
          ei->criteriaList[0] = ceil(euclideanDistance(ui->x, ui->y, vi->x, vi->y));
          ki->criteriaList[0] = ei->criteriaList[0];

          // set the second weight, "travel time", of each edge:
          unsigned int speed =
                       (unsigned int) (gen.getRandomNormalizedDouble() 
                                       * (maxSpeed - minSpeed) + minSpeed);
          ei->criteriaList[1] = 
                       (unsigned int) ceil( 
                             (double(3.6) * ei->criteriaList[0]) / speed );
          ki->criteriaList[1] = ei->criteriaList[1];

#if OPTION_NUM_OBJECTIVES == 3
          // CHANGE-HERE
          // set the third weight, "hop", of each edge:
          // - the "hop" cost will be equal to 1 for every edge; this way 
          //   the "NumberOfHops(P)" objective, which will be the sum of "hop" 
          //   costs for all edges in a path P, will be equal to the number 
          //   of edges (or hops) in the path
          ei->criteriaList[2] = 1;
          // set the incoming edge's cost as well
          ki->criteriaList[2] = ei->criteriaList[2];
#endif

          ++edgeProgress;
        }
      }

      // read the node-ids-to-node-descriptors mapping vector
      nodeIdsToDescriptors_ = reader.getIds();
      // remember that a graph has been read
      noGraph_ = false;
      // remember what type of graph we are using
      usingDimacs9Graph_ = false;
    }

    //! \brief Run a multiobjective shortest path query on the included graph.
    //!
    //! \param sourceId The source node's id.
    //! \param targetId The target node's id.
    //! \param numObjectives How many objectives should we use? (the first 
    //!        two or all three?)
    //! \param computeExactParetoSetUsingNamoaStar If true, compute the exact 
    //!        Pareto Set using PGL's NAMOA\* implementation; if false, 
    //!        compute a convex (approximate) Pareto Set using 
    //!        pareto_approximator::BaseProblem::computeConvexParetoSet().
    //! \param useAStar If true, use our simple A* implementation 
    //!        inside comb; if false, use PGL's single objective Dijkstra 
    //!        implementation.
    //! \return A std::vector of Pareto points (as 
    //!         pareto_approximator::PointAndSolution objects).
    //! 
    //! Each pareto_approximator::PointAndSolution object will include 
    //! the corresponding path if computeExactParetoSetUsingNamoaStar was 
    //! false; if it was true (i.e. NAMOA\* was used) the object will 
    //! contain an empty path.
    //! 
    //! The resulting vector of pareto_approximator::PointAndSolution objects 
    //! will be sorted according to the objects' included 
    //! pareto_approximator::Point objects. pareto_approximator::Point 
    //! objects are sorted lexicographically.
    //! 
    //! \sa pareto_approximator::BaseProblem::computeConvexParetoSet() and 
    //!     MultiobjectiveSpOnPmgProblem::comb()
    //!
    std::vector< pa::PointAndSolution<Path> > 
    runQuery(unsigned int sourceId, unsigned int targetId, 
             unsigned int numObjectives=2, 
             bool computeExactParetoSetUsingNamoaStar=false, 
             bool useAStar=false) 
    {
      // We only do experiments for the 2 and 3 objectives cases.
      assert( (numObjectives == 2) || (numObjectives == 3) );

      timeSpentInComb_ = 0.0;

      std::vector< pa::PointAndSolution<Path> > result;

      // initialize the counter to comb() calls
      numCallsToComb_ = 0;

      if (noGraph_) {
        std::cerr << "No graph has been read! Cannot execute query!" << std::endl;
        exit(1);
      }

      // set the source and target nodes
      source_ = graph_.getNodeIterator(nodeIdsToDescriptors_.at(sourceId));
      target_ = graph_.getNodeIterator(nodeIdsToDescriptors_.at(targetId));

      if (computeExactParetoSetUsingNamoaStar) {
        // Compute the exact Pareto set using PGL's NAMOA* implementation.
        NamoaStarDijkstra<PmaGraph, 
//                          BoundedTCHeuristic> namoaStar(graph_, 
                          TCHeuristic> namoaStar(graph_, 
//                          GreatCircleHeuristic> namoaStar(graph_, 
                                                 numObjectives, 
                                                 &timestamp_);

        namoaStar.init(source_, target_);
        namoaStar.runQuery(source_, target_);

        result = transformLabelsToParetoPoints(target_->labels, numObjectives);
      }
      else {
        useAStar_ = useAStar;
        if (useAStar) {
          // Initialize the heuristics we'll use with A*.
          if (usingDimacs9Graph_) {
            // DIMACS 9 graph
            GreatCircleDistanceHeuristic<PmaGraph> heuristicEngine(graph_);
            heuristicEngine.initHeuristicLists(target_);
          }
          else {
            // DIMACS 10 graph
            EuclideanHeuristic<PmaGraph> heuristicEngine(graph_);
            heuristicEngine.initHeuristicLists(target_);
          }
          // Check if the heuristics (the ones we'll use with A*, not 
          // NAMOA*'s heuristics) are admissible/consistent.
          if (not hasAdmissibleHeuristic(graph_, &timestamp_, target_, 0))
            std::cout << "Careful, the 1st objective's heuristic is non-admissible! (please check it)\n";
          if (not hasConsistentHeuristic(graph_, 0))
            std::cout << "Careful, the 1st objective's heuristic is non-consistent! (please check it)\n";
          if (not hasAdmissibleHeuristic(graph_, &timestamp_, target_, 1))
            std::cout << "Careful, the 2nd objective's heuristic is non-admissible! (please check it)\n";
          if (not hasConsistentHeuristic(graph_, 1))
            std::cout << "Careful, the 2nd objective's heuristic is non-consistent! (please check it)\n";
#if OPTION_NUM_OBJECTIVES == 3
          // CHANGE-HERE
          if (not hasAdmissibleHeuristic(graph_, &timestamp_, target_, 2))
            std::cout << "Careful, the 3rd objective's heuristic is non-admissible! (please check it)\n";
          if (not hasConsistentHeuristic(graph_, 2))
            std::cout << "Careful, the 3rd objective's heuristic is non-consistent! (please check it)\n";
#endif
        }

        // Compute a convex (approximate) Pareto set using 
        // pareto_approximator::BaseProblem::computeConvexParetoSet().
        double approximationRatio = 1e-12;

        result = computeConvexParetoSet(numObjectives, approximationRatio);
      }

      return result;
    }

    //! \brief Initialize (non-permanent) node attributes to default values.
    //! 
    //! So that successive queries do not interfere with each other.
    //! 
    //! Note: Permanent values like the node's coordinates (x, y) 
    //!       will not be affected. 
    //! 
    //! A list of Node attributes that will be affected:
    //! - dist
    //! - pred
    //! - succ
    //! - marked
    //! - heuristicList
    //! - secondary_pqitem
    //! - pqitem
    //! - timestamp
    //! 
    //! \sa experiments_vs_namoa_star::Node
    //! 
    void 
    cleanNodeAttributes()
    {
      PmaGraph::NodeIterator ni, lastNode;
      for (ni = graph_.beginNodes(), lastNode = graph_.endNodes(); 
           ni != lastNode; ++ni) 
      {
        ni->pred = graph_.nilNodeDescriptor(); // Dijkstra and A*
        ni->closed = false; // A*
        ni->heuristicValue = 0.0; // A*
        ni->fScore = std::numeric_limits<double>::infinity(); // A*
        ni->heuristicList.clear(); // NAMOA* and A*
        ni->succ = graph_.nilNodeDescriptor(); // NAMOA*
        ni->marked = false; // NAMOA*
        ni->secondary_pqitem = std::numeric_limits<unsigned int>::max(); // NAMOA*
        ni->timestamp = timestamp_; // all
        ni->dist = std::numeric_limits<double>::infinity(); // all
        ni->pqitem = std::numeric_limits<unsigned int>::max(); // all
      }
    }

    //! \brief Get the graph associated with the problem.
    //!
    const PmaGraph & 
    getGraph()
    {
      // we assume a graph has been read (or the result will be empty)
      return graph_;
    }

    //! \brief Get the vector that maps the graph's node ids to node 
    //!        descriptors.
    //!
    const std::vector<PmaGraph::NodeDescriptor> & 
    getIdsToDescriptorsMapping()
    {
      // we assume a graph has been read (or the result will be empty)
      return nodeIdsToDescriptors_;
    }

    //! \brief Get the number of calls to comb(), during the last query.
    //!
    unsigned int 
    getNumCallsToComb()
    {
      return numCallsToComb_;
    }

    //! \brief Get the time (sec) spent inside comb(), during the last query.
    //!
    double 
    getTimeSpentInComb()
    {
      return timeSpentInComb_;
    }

  private:
    //! \brief The comb method we had to implement. (for 
    //!        pareto_approximator::BaseProblem)
    //! 
    //! \param weight Iterator to the initial position in an 
    //!        std::vector<double> containing the weights w_{i} of the 
    //!        objectives (in the linear combination of objective functions).
    //! \param lastWeight Iterator to the past-the-end position in an 
    //!        std::vector<double> containing the weights w_{i} of the 
    //!        objectives (in the linear combination of objective functions).
    //! \return A pareto_approximator::PointAndSolution object containing 
    //!         an s-t path (P) that minimizes \$f w_{0} * Cd(P) + 
    //!         w_{1} * Ct(P) \$f (where Cd(P) and Ct(P) are the costs of 
    //!         the path P according to the distance objective function and 
    //!         the travel time objective function respectively) and the 
    //!         corresponding point in objective space.
    //! 
    //! Minimizes linear combinations of the objective functions of the 
    //! following form:
    //! \$f w_{0} * Cd(P) + w_{1} * Ct(P) \$f, 
    //! where P is an s-t (source_ to target_) path, Cd(P) is the cost of 
    //! the path P according to the distance objective function and Ct(P) 
    //! is the cost of the path P according to the travel time objective 
    //! function.
    //! 
    //! The vector of weights will only contain two weights for this 
    //! problem, w_{0} (the weight for the distance cost) and w_{1} 
    //! (the weight for the travel time cost).
    //! 
    //! \sa MultiobjectiveSpOnPmgProblem, pareto_approximator::BaseProblem 
    //!     and pareto_approximator::BaseProblem::computeConvexParetoSet()
    //!
    pa::PointAndSolution<Path> 
    comb(std::vector<double>::const_iterator weight, 
         std::vector<double>::const_iterator lastWeight) 
    {
      Timer timer;
      timer.start();

      pa::PointAndSolution<Path> result;

      unsigned int numObjectives = std::distance(weight, lastWeight);
      assert( (numObjectives == 2) || (numObjectives == 3) );

      std::vector<double>::const_iterator wi;
      for (wi = weight; wi != lastWeight; ++wi)
        assert(*wi >= 0.0);

      // set each edge's "weight" member variable to a weighted sum 
      // of its "criteriaList" costs (w0 and w1 will be the weights)
      PmaGraph::NodeIterator u, lastNode;
      PmaGraph::EdgeIterator e, lastEdge;
      PmaGraph::InEdgeIterator k;

      if (useAStar_) {
        // Using the A* algorithm to compute an optimal solution for 
        // the combined objective function.

        // Make a vector of reduced weights, to use when calculating the 
        // combined heuristic. We use reduced weights for the heuristics 
        // to mitigate problems caused by floating point arithmetic etc.
        std::vector<double> reducedWeight(3);
        reducedWeight[0] = std::max(weight[0] - 1e-12, 0.0);
        reducedWeight[1] = std::max(weight[1] - 1e-12, 0.0);
        reducedWeight[2] = std::max(weight[2] - 1e-15, 0.0);

        // first, set edge weights and node heuristics
        if (numObjectives == 2) {
          // 2 objectives
          for (u = graph_.beginNodes(), 
               lastNode = graph_.endNodes(); u != lastNode; ++u) 
          {
            // set the node's heuristic value
            u->heuristicValue = reducedWeight[0] * u->heuristicList[0] + 
                                reducedWeight[1] * u->heuristicList[1];
//            u->heuristicValue = weight[0] * u->heuristicList[0] + 
//                                weight[1] * u->heuristicList[1];

            // Set edge weights:
            // for all outgoing and incoming edges
            for (e = graph_.beginEdges(u), 
                 lastEdge = graph_.endEdges(u); e != lastEdge; ++e) 
            {
              // set outgoing edge's weight
              e->weight = weight[0] * e->criteriaList[0] + 
                          weight[1] * e->criteriaList[1];
            }
          }
        }
        else if (numObjectives == 3) {
          // 3 objectives
          for (u = graph_.beginNodes(), 
               lastNode = graph_.endNodes(); u != lastNode; ++u) 
          {
            // set the node's heuristic value
            u->heuristicValue = reducedWeight[0] * u->heuristicList[0] + 
                                reducedWeight[1] * u->heuristicList[1] + 
                                reducedWeight[2] * u->heuristicList[2];
//            u->heuristicValue = weight[0] * u->heuristicList[0] + 
//                                weight[1] * u->heuristicList[1] + 
//                                weight[2] * u->heuristicList[2];

            // Set edge weights:
            // for all outgoing and incoming edges
            for (e = graph_.beginEdges(u), 
                 lastEdge = graph_.endEdges(u); e != lastEdge; ++e) 
            {
              // set outgoing edge's weight
              e->weight = weight[0] * e->criteriaList[0] + 
                          weight[1] * e->criteriaList[1] + 
                          weight[2] * e->criteriaList[2];
            }
          }
        }

        // next, call PGL's A* implementation (we modified it so that it 
        // uses each node's "heuristicValue" attribute as a heuristic)
        AStarDijkstra<PmaGraph> aStarDijkstra(graph_, &timestamp_);
        aStarDijkstra.runQuery(source_, target_);
      }
      else {
        // Using Dijkstra's algorithm to compute an optimal solution for 
        // the combined objective function.

        // First, set edge weights:
        if (numObjectives == 2) {
          // 2 objectives
          for (u = graph_.beginNodes(), 
               lastNode = graph_.endNodes(); u != lastNode; ++u) 
          {
            // for each outgoing edge
            for (e = graph_.beginEdges(u), 
                 lastEdge = graph_.endEdges(u); e != lastEdge; ++e) 
            {
              // set the edge's weight
              e->weight = weight[0] * e->criteriaList[0] + 
                          weight[1] * e->criteriaList[1];
            }
          }
        }
        else if (numObjectives == 3) {
          // 3 objectives
          for (u = graph_.beginNodes(), 
               lastNode = graph_.endNodes(); u != lastNode; ++u) 
          {
            // for each outgoing edge
            for (e = graph_.beginEdges(u), 
                 lastEdge = graph_.endEdges(u); e != lastEdge; ++e) 
            {
              // set the edge's weight
              e->weight = weight[0] * e->criteriaList[0] + 
                          weight[1] * e->criteriaList[1] + 
                          weight[2] * e->criteriaList[2];
            }
          }
        }

        // next, call PGL's Dijkstra implementation (this is the default)
        Dijkstra<PmaGraph> dijkstra(graph_, &timestamp_);
        dijkstra.runQuery(source_, target_);
      }

      // increment the counter of comb() calls
      ++numCallsToComb_;

      // make the PointAndSolution object holding the result
      // - the result will contain a point in objective space (with 
      //   coordinates the values of the d objective functions) plus the 
      //   solution that corresponds to that point plus the weights that 
      //   produced that point
      result = computePathAndCriteriaValuesUpToThisNode(
                                         graph_.getNodeDescriptor(target_), 
                                         numObjectives);

      timer.stop();
      timeSpentInComb_ += timer.getElapsedTime();

      return result;
    }

    //! \brief Transform a vector of labels (which NAMOA\* has computed) to a 
    //!        std::vector< pareto_approximator::PointAndSolution<Path> >.
    //! 
    //! \param labels A std::vector of labels (which NAMOA\* has computed).
    //! \return A vector of Pareto points (PointAndSolution<Path> objects).
    //! 
    //! Why should we need to do this?
    //! 1. Because our 
    //!    pareto_approximator::BaseProblem::computeConvexParetoSet() also 
    //!    returns a std::vector of pareto_approximator::PointAndSolution 
    //!    instances. This way comparing our results with NAMOA\*'s 
    //!    results will be straightforward.
    //! 2. Because the resulting Pareto set will now be independent from the 
    //!    graph's nodes' attributes whereas NAMOA\*'s results are 
    //!    inextricably linked to the nodes' attributes (e.g. the Pareto  
    //!    points are stored in the target node's 
    //!    experiments_vs_namoa_star::Node::labels attribute). This way 
    //!    we can then call any other algorithm that operates on the graph's 
    //!    nodes (not deleting nodes) without losing the Pareto set which 
    //!    NAMOA\* computed.
    //! 
    //! This method does not attempt to compute each Pareto point's 
    //! corresponding path. It would have to recursively follow each Label's 
    //! (let's call it L) m_pred attribute to the node's predecessor and 
    //! then search the predecessor's labels for the one which created L 
    //! but since we don't need the paths for this experiment we will not 
    //! do that. Note: if it did compute each Pareto point's corresponding 
    //! path every node's labels attribute would have to be valid - now 
    //! we only need one list of labels (the target node's one).
    //! 
    //! We will only use this method after a call to PGL's implementation 
    //! of the NAMOA\* algorithm (for some s-t query) and only on the 
    //! query target's list of labels (t.labels).
    //! 
    //! \sa pareto_approximator::PointAndSolution and 
    //!     pareto_approximator::BaseProblem::computeConvexParetoSet()
    //!
    std::vector< pa::PointAndSolution<Path> > 
    transformLabelsToParetoPoints(const std::vector<Label> & labels, 
                                  unsigned int numObjectives) const
    {
      assert( (numObjectives == 2) || (numObjectives == 3) );

      // we assume that a NAMOA\* query with this node as the target node 
      // has already been run and this node's labels attribute has been set

      std::vector< pa::PointAndSolution<Path> > result(labels.size());

      std::vector< pa::PointAndSolution<Path> >::iterator rit;
      std::vector<Label>::const_iterator lit;
      if (numObjectives == 2) {
        for (lit = labels.begin(), rit = result.begin(); 
             lit != labels.end(); 
             ++lit, ++rit) {
          rit->setPoint(pa::Point(lit->getCriteriaList()[0], 
                                  lit->getCriteriaList()[1]));
        }
      }
      else if (numObjectives == 3) {
        for (lit = labels.begin(), rit = result.begin(); 
             lit != labels.end(); 
             ++lit, ++rit) {
          rit->setPoint(pa::Point(lit->getCriteriaList()[0], 
                                  lit->getCriteriaList()[1], 
                                  lit->getCriteriaList()[2]));
        }
      }

      std::sort(result.begin(), result.end());

      return result;
    }

    //! \brief Follows the given node's pred attribute to compute the path 
    //!        up to here. Also computes the path's cost for every criterion.
    //! 
    //! \param n A node descriptor.
    //! \return A pareto_approximator::PointAndSolution<Path> object 
    //!         containing the point that corresponds to the path's criteria 
    //!         costs and the path itself as an
    //!         experiments_vs_namoa_star::Path object.
    //! 
    //! This method makes the Path by recursively following the node's pred 
    //! attribute to the node's predecessor. The node's (and every other 
    //! node's in the graph) pred attribute must have been set by running 
    //! PGL's single objective Dijkstra on the graph (for some 
    //! s-t query).
    //! 
    //! We know we have reached the start of the path when a node's pred 
    //! attribute is NULL.
    //! 
    //! We will only use this method inside  
    //! experiments_vs_namoa_star::MultiobjectiveSpOnPmgProblem::comb() and 
    //! only on the target node (to make comb's return value).
    //! 
    //! \sa pareto_approximator::PointAndSolution and 
    //!     pareto_approximator::BaseProblem::comb()
    //!
    pa::PointAndSolution<Path> 
    computePathAndCriteriaValuesUpToThisNode(const NodeDescriptor & n, 
                                             unsigned int numObjectives) 
    {
      assert( (numObjectives == 2) || (numObjectives == 3) );

      pa::PointAndSolution<Path> result;

      // Reminder: result.point will hold a pareto_approximator::Point 
      //           instance containing the path's (multiple) criteria costs
      //           and result.solution will hold the actual path

      // about n.pred (and every Node's pred attribute in general)
      // - n.pred is a (void *) pointer but it can (and will) hold a 
      //   NodeDescriptor (instead of a pointer to a NodeDescriptor) 
      //   because the graph's NodeDescriptor type is actually a pointer
      NodeDescriptor currentNode = n;
      NodeDescriptor predecessorNode = 
                     (NodeDescriptor) graph_.getNodeIterator(n)->pred;
      EdgeIterator ei;

      // We make a new (temporary) CriteriaList named pathCriteriaCosts, 
      // the same size as edge criteriaList attributes so that we can easily 
      // sum the edges' criteria lists. 
      CriteriaList pathCriteriaCosts(numObjectives);

      // make the current vertex (v) the end of the path
      // - the path is currently empty
      result.solution.push_back(currentNode);

      while (predecessorNode != graph_.nilNodeDescriptor()) {
        // add the predecessor to the front of the path
        result.solution.push_front(predecessorNode);

        // update the path's (multiple criteria) costs
        ei = graph_.getEdgeIterator(predecessorNode, currentNode);
        pathCriteriaCosts = pathCriteriaCosts + ei->criteriaList;
        
        // move to the predecessor and reiterate
        currentNode = predecessorNode;
        predecessorNode = (NodeDescriptor) 
                          graph_.getNodeIterator(currentNode)->pred;
      }

      // store the path's (multiple) criteria costs inside "result"
      if (numObjectives == 2) 
        result.point = pa::Point(pathCriteriaCosts[0], pathCriteriaCosts[1]);
      else if (numObjectives == 3) 
        result.point = pa::Point(pathCriteriaCosts[0], 
                                 pathCriteriaCosts[1], 
                                 pathCriteriaCosts[2]);
      
      return result;
    }

    //! \brief A Packed Memory Array graph. (PGL's implementation)
    //!
    //! MultiobjectiveSpOnPmgProblem's constructor initially reads this from 
    //! a DIMACS 9 graph file. We can later change the graph by calling 
    //! MultiobjectiveSpOnPmgProblem::readDimacs9Graph() to read a new one.
    //! 
    //! See http://www.dis.uniroma1.it/challenge9/format.shtml for more 
    //! information on the DIMACS 9 graph files' format.
    //!
    PmaGraph graph_;

    //! \brief The source node for an s-t shortest path query on the graph.
    //!
    //! MultiobjectiveSpOnPmgProblem::comb() will need this (and we can't 
    //! change comb()'s generic signature to pass this in as a parameter).
    //! 
    //! MultiobjectiveSpOnPmgProblem::runQuery() sets this before calling 
    //! pareto_approximator::BaseProblem::computeConvexParetoSet() or 
    //! PGL's NAMOA\* implementation. 
    //! Note: NAMOA\* does not actually need/use this but 
    //! MultiobjectiveSpOnPmgProblem::runQuery() sets it anyway in order to 
    //! remain consistent.
    //! 
    //! \sa experiments_vs_namoa_star::MultiobjectiveSpOnPmgProblem::comb()
    //!
    PmaGraph::NodeIterator source_;

    //! \brief The source node for an s-t shortest path query on the graph.
    //!
    //! experiments_vs_namoa_star::MultiobjectiveSpOnPmgProblem::comb() will 
    //! need this (and we can't change comb()'s generic signature to pass 
    //! this in as a parameter).
    //! 
    //! \sa experiments_vs_namoa_star::MultiobjectiveSpOnPmgProblem::comb()
    //!
    PmaGraph::NodeIterator target_;

    //! \brief A std::vector that maps node ids to NodeDescriptors.
    //! 
    std::vector<PmaGraph::NodeDescriptor> nodeIdsToDescriptors_;

    //! \brief A timestamp. Will use it with PGL's algorithms.
    //! 
    unsigned int timestamp_;

    //! \brief Remembers if a graph has (not) been read. 
    //!
    //! true if no graph has been read; false otherwise
    //!
    bool noGraph_;

    //! \brief The number of times comb() was called, during a single query.
    //!
    //! Set to zero when MultiobjectiveSpOnPmgProblem::runQuery() is called 
    //! and incremented inside MultiobjectiveSpOnPmgProblem::comb().
    //!
    unsigned int numCallsToComb_;

    //! \brief Time (sec) spent inside MultiobjectiveSpOnPmgProblem::comb().
    //!
    //! Reset every time MultiobjectiveSpOnPmgProblem::runQuery() is 
    //! called.
    //!
    double timeSpentInComb_;

    //! \brief Remembers what type of graph we are using (DIMACS 9 or 10).
    //!
    //! true if we are using a DIMACS 9 type graph; false if we are using 
    //! a DIMACS 10 type graph; not valid if noGraph_ is true
    //!
    bool usingDimacs9Graph_;

    //! \brief Flag, for choosing between A* and Dijkstra inside Comb.
    //! 
    //! True if we want to use A* inside Comb; false if we want to use 
    //! Dijkstra.
    //!
    bool useAStar_;
};


}  // namespace experiments_vs_namoa_star


//!
//! @}
//!


#endif  // EXPERIMENTS_VS_NAMOA_STAR_MULTIOBJECTIVE_SP_ON_PMG_PROBLEM_H
