#ifndef romea_ENUPath2D_hpp
#define romea_ENUPath2D_hpp

//romea
#include <romea_common/math/Interval.hpp>
#include <romea_common/containers/Eigen/VectorOfEigenVector.hpp>
#include "PathCurve2D.hpp"

//boost
#include <boost/optional.hpp>
#include <atomic>

namespace romea {

class Path2D
{
public:

  using Vector = std::vector<double,Eigen::aligned_allocator<double> > ;

public:

  Path2D();

  Path2D(const double & interpolationWindowLength);

public :

  void setInterpolationWindowLength(const double & interpolationWindowLength);

  const double & getInterpolationWindowLength()const;

  void load(const VectorOfEigenVector2d & points);

  bool isLoaded()const;

  const Vector & getX()const;

  const Vector & getY()const;

  const Vector & getCurvilinearAbscissa()const;

  const std::vector<PathCurve2D> & getCurves() const;

  double getLength()const;



  size_t findNearestIndex(const double & curvilinearAbscissa) const;

  Interval<size_t> findMinMaxIndexes(const double & curvilinearAbscissa,
                                     const double & researchIntervalLength) const;

  Interval<size_t> findMinMaxIndexes(const size_t & pointIndex,
                                     const double & researchIntervalLength) const;

private :

  Vector X_;
  Vector Y_;
  Vector curvilinearAbscissa_;
  std::vector<PathCurve2D> curves_;
  double interpolationWindowLength_;

  std::atomic<bool> isLoaded_;
};

}

#endif
