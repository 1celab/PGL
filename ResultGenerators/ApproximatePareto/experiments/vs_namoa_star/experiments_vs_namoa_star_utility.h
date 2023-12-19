
//! \file experiments/vs_namoa_star/experiments_vs_namoa_star_utility.h
//! \brief Helpful functions for the experiment vs the NAMOA\* algorithm.
//! \author Christos Nitsas
//! \date 2013
//!


#ifndef EXPERIMENTS_VS_NAMOA_STAR_UTILITY_H
#define EXPERIMENTS_VS_NAMOA_STAR_UTILITY_H


#include <assert.h>
#include <fstream>
#include <cmath>

#include "experiments_vs_namoa_star_common.h"
#include "Hyperplane.h"

// CLEANUP CHANGE
#include <Algorithms/Pareto/ApproximatePareto/Point.h>
#include <Algorithms/Pareto/ApproximatePareto/PointAndSolution.h>
#include <Algorithms/Pareto/ApproximatePareto/utility.h>
//#include "../../Point.h"
//#include "../../PointAndSolution.h"
//#include "../../utility.h"


namespace pa = pareto_approximator;


//!
//! \addtogroup ExperimentsVsNamoaStar Experiments vs the NAMOA Star algorithm.
//!
//! @{
//!


//! Everything needed for the experiments vs the NAMOA\* algorithm.
namespace experiments_vs_namoa_star {


//!
//! \brief Read a list of s-t queries from a file.
//! 
//! Query file's format
//! ----------------------
//! Each line is considered a separate s-t query. Each line should 
//! contain two unsigned integers separated by a space. The first 
//! number on each line will be the id of the query's source node and 
//! the second one the id of the target node. Lines that start with a 
//! # (number sign) are considered comment lines and are ignored.
//!
std::list< std::pair<unsigned int, unsigned int> > 
parseQueriesFile(std::string filename)
{
  std::list< std::pair<unsigned int, unsigned int> > result;

  std::ifstream input;
  std::string line;
  unsigned int sourceId, targetId;

  try {
    input.open(filename.c_str());
    assert(input.good());

    // read the file line by line
    while (std::getline(input, line)) {
      if (line.c_str()[0] == '#') {
        // ignore the line; it's a comment line
        continue;
      }
      // else 

      std::istringstream iss(line);
      // the line should contain two numbers, store them in
      // sourceId and targetId
      if ( !(iss >> sourceId >> targetId) ) {
        // error (e.g. wrong number or type of items on the line)
        break;
      }
      // else

      result.push_back(std::pair<unsigned int, unsigned int>(sourceId, targetId));
    }

    input.close();
  }
  catch (std::ifstream::failure e) {
    std::cerr << "Exception opening/reading file '" << filename << "'\n";
    throw e;
  }

  return result;
}


//! \brief Find the point (pa::PointAndSolution<Path> instance) that has 
//!        the smallest ith coordinate, from a sequence of points.
//!
//! We use this inside computeLowerConvexEnvelopeOfPoints().
//!
pa::PointAndSolution<Path> 
findBestPointOnIthObjective(
          std::vector< pa::PointAndSolution<Path> >::const_iterator first, 
          std::vector< pa::PointAndSolution<Path> >::const_iterator last, 
          unsigned int i)
{
  std::vector< pa::PointAndSolution<Path> >::const_iterator min = first;
  for ( first = first + 1; first != last; ++first)
    if (first->point[i] < min->point[i])
      min = first;

  return *min;
}


//! \brief Compute the lower part of the convex hull of the given set of 
//!        points.
//!
//! \param points The set of points (PointAndSolution<Path> instances) whose 
//!        lower convex envelope (i.e. lower part of the convex hull) we want.
//! \return A list of the extreme points of the lower convex envelope. They 
//!         will all be vertices of the convex hull (but not all vertices of 
//!         the convex hull will be here).
//!
//! First calls pareto_approximator::utility::computeConvexHull(), which in 
//! turn calls qhull's qconvex (see www.qhull.org), to compute the convex 
//! hull of the set of points. It will return a set of the extreme points of 
//! the convex hull, let's call them EH.
//!
//! Then discards all points of EH that are above the line connecting points 
//! A and B, where A is the leftmost (i.e. best in x axis's criterion) point 
//! of EH and B is the lowest (i.e. best in y axis's criterion) point of EH.
//! Why? Because A and B will definitely be on the lower convex envelope but 
//! points above the line AB will be on the upper half of the convex 
//! envelope.
//!
//! Note: We only use this function on sets of points NAMOA* computed, which 
//!       will all be points of the Pareto set. One might think that, because 
//!       of this, there wouldn't be any points above the line AB after 
//!       computing EH but actually, a Pareto point can be above the line AB 
//!       and still be Pareto optimal if the problem is not convex (i.e. 
//!       objective functions not convex, etc).
//!
//! \sa pareto_approximator::utility::computeConvexHull()
//!
std::list< pa::PointAndSolution<Path> > 
computeLowerConvexEnvelopeOfPoints(
                  const std::vector< pa::PointAndSolution<Path> > & points,
                  unsigned int spaceDimension)
{
  std::list< pa::PointAndSolution<Path> > results;

  // we need at least #(spaceDimension+1) points to compute a convex hull
  if (points.size() <= spaceDimension) {
    // no need to continue, all points in "points" are on the lower envelope
    results.assign(points.begin(), points.end());
    return results;
  }
  // else

  // first compute the convex hull of the set of points
  // (and keep the extreme points (i.e. the vertices) of the convex hull)
  std::list< pa::PointAndSolution<Path> > hull;
  hull = pa::utility::computeConvexHull(points, spaceDimension);

  if (hull.size() <= spaceDimension) 
    // no need to continue, all points in "results" 
    // are on the lower envelope
    return hull;
  // else

  // We only need the lower envelope (i.e. the lower half) of the convex hull.
  // Find the best point for each objective (we call them anchor points), 
  // compute the hyperplane that connects those points and discard any 
  // points above the hyperplane. e.g. for 2 objectives:
  // - find the leftmost (i.e. best in x axis criterion) extreme point, A
  // - find the lowest (i.e. best in the y axis criterion) extreme point, B 
  // - A and B (like all the points in "points") will both be on the 
  //   all-positive quadrant (quadrant I of the Cartesian plane)
  // - discard any point that is "above" (i.e. on the side further away from 
  //   the origin of the Cartesian plane) the line connecting A and B

  std::vector< pa::PointAndSolution<Path> > anchorPoints;
  anchorPoints.reserve(spaceDimension);
  for (unsigned int i = 0; i != spaceDimension; ++i) {
    anchorPoints.push_back(findBestPointOnIthObjective(points.begin(), 
                                                  points.end(), i));
  }
  Hyperplane hyperplane(anchorPoints.begin(), anchorPoints.end());
  results.assign(anchorPoints.begin(), anchorPoints.end());

  std::list< pa::PointAndSolution<Path> >::iterator it, last;
  for (it = hull.begin(), last = hull.end(); it != last; ++it) {
    if (not hyperplane.dominates(it->point))
      results.push_back(*it);
  }
  results.sort();

  /*
  // the same for 2 objectives, without using a Hyperplane
  pa::PointAndSolution<Path> & A = results.front();
  pa::PointAndSolution<Path> & B = results.back();

  double slope = (A.point[1] - B.point[1]) / (A.point[0] - B.point[0]);
  double intercept = A.point[1] - slope * A.point[0];
  //assert(B.point[1] - slope * B.point[0] == intercept);

  std::list< pa::PointAndSolution<Path> >::iterator it, itToB;
  it = results.begin();
  ++it;     // iterator now points to the first element after A
  itToB = results.end();
  --itToB;  // iterator now points to B
  // for all elements between A and B:
  while (it != itToB) {
    // if the element is on or above the line that connects A and B:
    if ( it->point[1] - slope * it->point[0] >= intercept ) {
      // remove the element (iterator moves to the next element)
      it = results.erase(it);
    }
    else {
      // next element
      ++it;
    }
  }
  */

  return results;
}


//! \brief Write a sequence of points to a file.
//!
//! \tparam InputIterator The type of an input iterator. (should point to 
//!         pareto_approximator::PointAndSolution<Path> instances)
//! \param first Iterator to the first point (PointAndSolution object) in 
//!        the sequence.
//! \param last Iterator to the past-the-end point (PointAndSolution object) 
//!        in the sequence.
//! \param filename The name of the file to write to. If the file does 
//!        not exist it will be created; if the file already exists it 
//!        will be overwritten.
//!
template <class InputIterator> 
void 
printPointsToFile(InputIterator first, InputIterator last, std::string filename)
{
  std::ofstream out(filename.c_str(), std::ios::out | std::ios::trunc);

  if (not out.is_open()) {
    std::cerr << "An error occured while opening file \"" << filename 
              << "\" for output... Exiting" << std::endl;
    exit(-1);
  }
  // else

  for ( ; first != last; ++first)
    out << first->point << "\n";

  out.close();
}


//! \brief Read a sequence of points from a file.
//!
//! \param filename The file's name.
//! \return A std::vector of PointAndSolution<Path> instances containing 
//!         the points read from the file.
//!
std::vector< pa::PointAndSolution<Path> > 
readPointsFromFile(std::string filename)
{
  std::vector< pa::PointAndSolution<Path> > results;

  std::ifstream input(filename.c_str());

  if (not input.is_open()) {
    std::cerr << "An error occured while opening file \"" << filename
              << "\" for input... Exiting\n";
    exit(-1);
  }

  pa::Point point;
  input >> point;
  while (not input.eof()) {
    results.push_back(pa::PointAndSolution<Path>(point, Path()));
    input >> point;
  }

  return results;
}


//! \brief The euclidean distance between two points.
//!
double euclideanDistance(long x1, long y1, long x2, long y2)
{
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}


//! \brief Our implementation of the Great-Circle distance between two 
//!        points, in meters.
//!
//! \param latitude1 The first point's latitude.
//! \param longitude1 The first point's longitude.
//! \param latitude2 The second point's latitude.
//! \param longitude2 The second point's longitude.
//! \param earthRadius The value we want to use as the Earth radius. 
//!
//! According to Wikipedia, the Earth's radius varies from 6,378.137 km at 
//! the equator, to 6,356.752 km at the poles. Also, as a special case, and 
//! because the Earth more closely resembles a flattened sphere (spheroid), 
//! when calculating the length of a short north-south line at the equator, 
//! the sphere that best approximates that part of the spheroid has a radius 
//! of 6,335.439 km.
//!
//! The fact is that we cannot use a single value for the Earth's radius 
//! and expect accurate results every time.
//!
//! This function can accept a user-given value for the Earth's radius;
//! we assume the user knows what is best for his specific use case. If 
//! the user has no preference we use 6,367.4445, i.e. the average between 
//! the radius at the equator and the radius at the poles.
//!
double greatCircleDistance(double latitude1, double longitude1, 
                           double latitude2, double longitude2, 
                           double earthRadius=6367444.5)
{
  double pi = 4.0 * atan(1.0);

  // convert coordinates from degrees to radians
  double f1 = latitude1 * (pi/180);
  double l1 = longitude1 * (pi/180);
  double f2 = latitude2 * (pi/180);
  double l2 = longitude2 * (pi/180);

  // coordinate absolute differences
  double df = fabs(f1 - f2);
  double dl = fabs(l1 - l2);

  // central angle between the two points
  double radicant = pow(sin(df/2.0), 2) + cos(f1) * cos(f2) * pow(sin(dl/2.0), 2);
  double ds = 2.0 * asin( sqrt(radicant) );

  // alternate equation for central angle between the two points
  // - a bit better (less rounding errors) in the special (and somewhat 
  //   unusual) case of antipodal points (i.e. points located on opposite 
  //   ends of the sphere)
  //double radicant = pow( cos(f2) * sin(dl), 2.0) + pow( cos(f1) * sin(f2) - sin(f1) * cos(f2) * cos(dl), 2.0);
  //double denominator = sin(f1) * sin(f2) + cos(f1) * cos(f2) * cos(dl);
  //double ds = atan2( sqrt(radicant), denominator );

  return ds * earthRadius;
}


//! \brief Compute and return an overestimate of the actual Great Circle 
//!        distance between two points.
//!
//! \param latitude1 The first point's latitude.
//! \param longitude1 The first point's longitude.
//! \param latitude2 The second point's latitude.
//! \param longitude2 The second point's longitude.
//! \return The Great Circle distance between the given points assuming 
//!         an Earth radius of 6400000 meters.
//!
//! According to Wikipedia, the Earth's radius varies from 6,378.137 km at 
//! the equator, to 6,356.752 km at the poles. Also, as a special case, and 
//! because the Earth more closely resembles a flattened sphere (spheroid), 
//! when calculating the length of a short north-south line at the equator, 
//! the sphere that best approximates that part of the spheroid has a radius 
//! of 6,335.439 km.
//!
//! The fact is that we cannot use a single value for the Earth's radius 
//! and expect accurate results every time.
//!
//! This function calls the greatCircleDistance() function with an Earth 
//! radius of 6,400.000 km to get an overestimate of the actual distance.
//! This is useful if, for example, we use the estimate to compute upper 
//! bounds to distances. 
//!
//! \sa greatCircleDistance()
//!
double 
greatCircleOverestimate(double latitude1, double longitude1, 
                        double latitude2, double longitude2)
{
  return greatCircleDistance(latitude1, longitude1, 
                             latitude2, longitude2, 6400000);
}


//! \brief Compute and return an underestimate of the actual Great Circle 
//!        distance between two points.
//!
//! \param latitude1 The first point's latitude.
//! \param longitude1 The first point's longitude.
//! \param latitude2 The second point's latitude.
//! \param longitude2 The second point's longitude.
//! \return The Great Circle distance between the given points using an  
//!         Earth radius of 6000000 meters.
//!
//! According to Wikipedia, the Earth's radius varies from 6,378.137 km at 
//! the equator, to 6,356.752 km at the poles. Also, as a special case, and 
//! because the Earth more closely resembles a flattened sphere (spheroid), 
//! when calculating the length of a short north-south line at the equator, 
//! the sphere that best approximates that part of the spheroid has a radius 
//! of 6,335.439 km.
//!
//! The fact is that we cannot use a single value for the Earth's radius 
//! and expect accurate results every time.
//!
//! This function calls the greatCircleDistance() function with an Earth 
//! radius of 6,100.000 km to get an underestimate of the actual distance.
//! This is useful if for example we use the estimate to compute heuristics 
//! and we want them to be admissible.
//!
//! \sa greatCircleDistance()
//!
double 
greatCircleUnderestimate(double latitude1, double longitude1, 
                         double latitude2, double longitude2)
{
  return greatCircleDistance(latitude1, longitude1, 
                             latitude2, longitude2, 4000000);
}


}  // namespace experiments_vs_namoa_star


//! 
//! @}
//!


#endif  // EXPERIMENTS_VS_NAMOA_STAR_UTILITY_H
