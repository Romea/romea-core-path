#ifndef romea_PathSection2D_hpp
#define romea_PathSection2D_hpp

//romea
#include <romea_common/math/Interval.hpp>
#include <romea_common/containers/Eigen/VectorOfEigenVector.hpp>
#include <romea_common/containers/Eigen/DequeOfEigenVector.hpp>
#include "CumulativeSum.hpp"
#include "PathCurve2D.hpp"
#include "PathWayPoint2D.hpp"

//boost
#include <boost/optional.hpp>
#include <atomic>

namespace romea {


//TODO séparer la version static et online
class PathSection2D
{
public:

  using Vector = std::vector<double,Eigen::aligned_allocator<double> > ;
  using CurvilinearAbscissa = CumulativeSum<double,Eigen::aligned_allocator<double>>;

public:

  PathSection2D(const double & interpolationWindowLength,
                const double & initialCurvilinearAbcissa = 0);

  void addWayPoint(const PathWayPoint2D & wayPoint);

  void addWayPoints(const std::vector<PathWayPoint2D> & wayPoints);

  const PathCurve2D & getCurve(const size_t & pointIndex) const;

  const Vector & getX()const;

  const Vector & getY()const;

  const CurvilinearAbscissa &getCurvilinearAbscissa()const;

  const Vector & getSpeeds() const;

  const double & getLength()const;

  void reserve(size_t n);

  size_t size()const;

  void clear();


  size_t findIndex(const double & value) const;

  size_t findIndex(const double & value,
                   const size_t & startSearchIndex) const;

  Interval<size_t> findIntervalBoundIndexes(const size_t & intervalCenterIndex,
                                            const double & intervalWidth)const;

  Interval<size_t> findIntervalBoundIndexes(const size_t & intervalCenterIndex,
                                            const Interval<double> & interval)const;


private :

  void incrementCurvilinearAbscissa_();

  void computePathCurve_(const size_t & pointIndex)const;

  Interval<double> computeCurvilinearAbscissaInterval_(const size_t & intervalCenterIndex,
                                                       const double & intervalWidth) const;

private :

  Vector X_;
  Vector Y_;
  CurvilinearAbscissa curvilinearAbscissa_;

  mutable std::vector<boost::optional<PathCurve2D>> curves_;
  Vector speeds_;

  double interpolationWindowLength_;
  double length_;

};

}

#endif
