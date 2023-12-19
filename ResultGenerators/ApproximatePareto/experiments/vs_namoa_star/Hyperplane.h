//! \file experiments/vs_namoa_star/Hyperplane.h
//! \brief The implementation of the Hyperplane class.
//! \author Christos Nitsas
//! \date 2013
//!


#ifndef EXPERIMENTS_VS_NAMOA_STAR_HYPERPLANE_H
#define EXPERIMENTS_VS_NAMOA_STAR_HYPERPLANE_H


#include <assert.h>
#include <iterator>
#include <algorithm>
#include <armadillo>

// CLEANUP CHANGE
#include <Algorithms/Pareto/ApproximatePareto/Point.h>
#include <Algorithms/Pareto/ApproximatePareto/PointAndSolution.h>
//#include "../Point.h"
//#include "../PointAndSolution.h"


namespace pa = pareto_approximator;


//! 
//! \addtogroup ExperimentsVsNamoaStar Experiments vs the NAMOA Star algorithm.
//! 
//! @{
//!


//! Everything needed for the experiments vs the NAMOA\* algorithm.
namespace experiments_vs_namoa_star {


//! \brief A (very) simple hyperplane implementation.
//!
//! We represent a hyperplane using its normal vector and its offset. 
//!
class Hyperplane
{
  public:
    //! The type of an element of the hyperplane's normal vector.
    typedef double NormalVectorElement;

    //! The type of the hyperplane's normal vector.
    typedef std::vector<double> HyperplaneNormalVector;

    //! Random access iterator to the elements of the hyperplane's normal.
    typedef std::vector<double>::iterator HyperplaneNormalIterator;

    //! Constant random access iterator to the elements of the hyperplane's normal.
    typedef std::vector<double>::const_iterator ConstHyperplaneNormalIterator;

    //! Constructor
    /*!
     *  \param firstVertex An iterator to the first element in the container 
     *         of points the hyperplane must pass through.
     *  \param lastVertex An iterator to the past-the-end element in the 
     *         container of points the hyperplane must pass through.
     *  \param preferPositiveNormalVector While computing the hyperplane's normal 
     *                    vector prefer the all-positive one (if it exists).
     */
    Hyperplane(
      std::vector< pa::PointAndSolution<Path> >::const_iterator firstVertex, 
      std::vector< pa::PointAndSolution<Path> >::const_iterator lastVertex, 
      bool preferPositiveNormalVector=true)
    {
      spaceDimension_ = firstVertex->dimension();
      assert(std::distance(firstVertex, lastVertex) == spaceDimension_);

      // Make sure that all the given vertices are valid.
      // - They all have the correct dimension. (i.e. the dimension of the space 
      //   that the hyperplane lives in)
      // - They (and the points they contain) are not null instances.
      std::vector< pa::PointAndSolution<Path> >::const_iterator cpi;
      for (cpi = firstVertex; cpi != lastVertex; ++cpi) {
        if (cpi->isNull() or cpi->point.isNull())
          throw pa::exception_classes::NullObjectException();
        if (cpi->dimension() != spaceDimension_)
          throw pa::exception_classes::DifferentDimensionsException();
      }

      // Compute and set the hyperplane's normal vector (Hyperplane<S>::normal_).
      computeAndSetHyperplaneNormal(firstVertex, lastVertex, 
                                    preferPositiveNormalVector);

      b_ = arma::dot( arma::vec(normal_), firstVertex->point.toVec() );
    }

    //! Get the hyperplane offset.
    double b()
    {
      return b_;
    }

    //! Return the dimension of the space that the hyperplane lives in.
    /*!
     *  \sa Hyperplane
     */
    unsigned int 
    spaceDimension() const
    {
      return spaceDimension_;
    }

    //! \brief Compute (and set) the hyperplane's normal vector using the 
    //!        points that the hyperplane must pass through (we call them 
    //!        vertices).
    /*!
     *  \param firstVertex An iterator to the first element in the container 
     *         of points the hyperplane must pass through.
     *  \param lastVertex An iterator to the past-the-end element in the 
     *         container of points the hyperplane must pass through.
     *  \param preferPositiveNormalVector Should we prefer the all-positive 
     *                                    normal vector (if it exists)?
     *  
     *  For each set of n vertices there are two different n-hyperplanes passing 
     *  through them with opposite normal vectors. This method will prefer the 
     *  all-positive normal vector (if one exists) if preferPositiveNormalVector 
     *  is set to true; otherwise it will choose one depending on the order of 
     *  the hyperplane vertices.
     *  
     *  \sa Hyperplane
     */
    void 
    computeAndSetHyperplaneNormal(
      std::vector< pa::PointAndSolution<Path> >::const_iterator firstVertex, 
      std::vector< pa::PointAndSolution<Path> >::const_iterator lastVertex, 
      bool preferPositiveNormalVector) 
    {
      // fill a matrix will each point's coordinates
      arma::mat M;
      for ( ; firstVertex != lastVertex; ++firstVertex)
        M.insert_rows(M.n_rows, firstVertex->point.toRowVec());
      // add a column of ones at the end (will make the following easier)
      M.insert_cols(M.n_cols, arma::ones<arma::vec>(spaceDimension()));
      
      // fill in the normal vector's elements
      for (unsigned int i = 0; i != spaceDimension(); ++i) {
        M.swap_cols(i, M.n_cols - 1);
        normal_.push_back(arma::det(M.cols(0, M.n_cols - 2)));
        M.swap_cols(i, M.n_cols - 1);
      }

      if (preferPositiveNormalVector && hasAllNormalVectorElementsNonPositive())
        reverseNormalVectorSign();
    }
    
    //! Check if every element of the hyperplane's normal vector is non-positive.
    /*!
     *  \return true if every element of the hyperplane's normal vector 
     *          (Hyperplane::normal_) is non-positive.
     *  
     *  Each element must be non-positive.
     *  
     *  \sa Hyperplane
     */
    bool 
    hasAllNormalVectorElementsNonPositive() const
    {
      for (unsigned int i = 0; i != spaceDimension(); ++i)
        if (normal_[i] > 0.0)
          return false;

      return true;
    }

    //! Reverse the sign of all elements of the hyperplane's normal vector.
    /*!
     *  Reverse the sign of all the elements of the hyperplane's normal vector 
     *  (Hyperplane::normal_).
     *  
     *  \sa Hyperplane
     */
    void 
    reverseNormalVectorSign()
    {
      for (unsigned int i = 0; i != spaceDimension(); ++i)
        normal_[i] = -normal_[i];
    }

    /*!
     *  \brief Compute a point's additive distance from the hyperplane.
     *  
     *  \param p A Point instance.
     *  \return The point's additive distance from the hyperplane.
     *          (i.e. the minimum value of \f$\epsilon\f$ such that the 
     *          hyperplane dominates the point in the additive sense)
     *  
     *  The additive distance from a point p to a hyperplane H is defined as:
     *  \f$ AD(p, H) = \min_{q \in H} AD(p, q) \f$, where q is a point on H.
     *  The additive distance from a point p to a point q is defined as:
     *  \f$ AD(p, q) = \max\{ \max_{i}\{(q_{i} - p_{i})\}, 0.0 \} \f$.
     *  
     *  Intuitively it is the minimum value of \f$ \epsilon \ge 0 \f$ such 
     *  that some point on H \f$\epsilon\f$ -dominates (\f$\epsilon\f$ -covers) 
     *  p in the additive sense. To calculate it we take advantage of the fact 
     *  that the point (p + \f$\epsilon\f$) will be lying on H.
     *  
     *  Possible exceptions:
     *  - May throw a NullObjectException exception if the given Point 
     *    instance is a null Point instance.
     *  - May throw a DifferentDimensionsException exception if the given 
     *    point and the hyperplane belong in spaces of different dimensions.
     *  - May throw a NotPositivePointException exception if the given point 
     *    is not positive. (i.e. some coordinate is less than 0.0)
     *
     *  \sa Point and Hyperplane
     */
    double 
    additiveDistance(const pa::Point & p) const
    {
      if (p.isNull())
        throw pa::exception_classes::NullObjectException();
      if (spaceDimension() != p.dimension())
        throw pa::exception_classes::DifferentDimensionsException();
      if (not p.isPositive())
        throw pa::exception_classes::NotPositivePointException();
      // else

      assert(spaceDimension() > 0);

      double sumOfHyperplaneNormal = 0.0;
      double dotProduct       = 0.0;
      for (unsigned int i=0; i!=spaceDimension(); ++i) {
        sumOfHyperplaneNormal += normal_[i];
        dotProduct       += normal_[i] * p[i];
      }

      // - To calculate the result we take advantage of the fact that point 
      //   (p + \f$\epsilon\f$) will be lying on H (the hyperplane).
      // - sumOfHyperplaneNormal should not be 0.0 - it can only be 0.0 if 
      //   the hyperplane's normal vector is all zero (not a valid hyperplane)
      //   (we assume the hyperplane has an all-positive normal vector; 
      //   if not, there is no point in calling this method)
      assert(sumOfHyperplaneNormal != 0.0);
      return std::max( (b_ - dotProduct) / sumOfHyperplaneNormal, 0.0 );
    }

    /*! 
     *  \brief Check if the Hyperplane approximately dominates (in the additive 
     *         sense) the given point.
     *  
     *  \param p A Point instance. (must be positive - i.e. all coordinates 
     *           greater than or equal to 0.0)
     *  \param eps The approximation factor.
     *  \return true if some point on the hyperplane's supporting hyperplane 
     *          approximately dominates the given point in the additive 
     *          sense; false otherwise
     *  
     *  Possible exceptions:
     *  - May throw a NullObjectException exception if the given Point 
     *    instance is a null Point instance.
     *  - May throw a DifferentDimensionsException exception if the given 
     *    point and the hyperplane belong in spaces of different dimensions.
     *  - May throw a NegativeApproximationRatioException exception if the 
     *    given approximation ratio/factor/threshold is less than 0.0.
     *  - May throw a NotPositivePointException exception if the given point 
     *    is not positive. (i.e. some coordinate is less than 0.0)
     *  
     *  \sa Hyperplane, Point, Point::dominatesAdditive()
     */
    bool 
    dominates(const pa::Point & p) const
    {
      if (p.isNull())
        throw pa::exception_classes::NullObjectException();
      if (spaceDimension() != p.dimension())
        throw pa::exception_classes::DifferentDimensionsException();
      if (not p.isPositive())
        throw pa::exception_classes::NotPositivePointException();
      // else

      if (additiveDistance(p) == 0.0)
        return true;
      else
        return false;
    }

  private:
    //! The dimension of the space that the hyperplane lives in.
    /*!
     *  \sa Hyperplane
     */
    unsigned int spaceDimension_;

    //! The hyperplane normal.
    /*! 
     *  The hyperplane normal is a vector perpendicular to the hyperplane's surface. 
     *  The normal is simply the direction that the hyperplane is facing.
     *  
     *  \sa Hyperplane
     */
    std::vector<double> normal_;

    //! The hyperplane offset.
    double b_;
};


}  // namespace experiments_vs_namoa_star


//!
//! @}
//!


#endif  // EXPERIMENTS_VS_NAMOA_STAR_HYPERPLANE_H
