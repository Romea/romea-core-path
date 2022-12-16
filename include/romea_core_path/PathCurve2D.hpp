#ifndef ROMEA_CORE_PATH_PATHCURVE2D_HPP_
#define ROMEA_CORE_PATH_PATHCURVE2D_HPP_

// std
#include <vector>

// romea
#include "PathPosture2D.hpp"
#include "PathFrenetPose2D.hpp"
#include <romea_core_common/math/Interval.hpp>

namespace romea {


class PathCurve2D
{
public :

    using Vector = std::vector<double, Eigen::aligned_allocator<double> > ;

public :

  PathCurve2D();

  bool estimate(const Vector & X,
                const Vector & Y,
                const Vector & S ,
                const Interval<size_t> & indexInterval,
                const Interval<double> & curvilinearAbscissaInterval);

  std::optional<double> findNearestCurvilinearAbscissa(
    const Eigen::Vector2d & vehiclePosition) const;

  double computeX(const double & curvilinearAbscissa)const;

  double computeY(const double & curvilinearAbscissa)const;

  double computeTangent(const double & curvilinearAbscissa)const;

  double computeCurvature(const double & curvilinearAbscissa)const;

  const Interval<double> & getCurvilinearAbscissaInterval()const;

  const Interval<size_t> & getIndexInterval()const;

private :

  Eigen::Array3d fxPolynomCoefficient_;
  Eigen::Array3d fyPolynomCoefficient_;

  Interval<size_t> indexInterval_;
  Interval<double> curvilinearAbscissaInterval_;
};

}  // namespace romea

#endif  // ROMEA_CORE_PATH_PATHCURVE2D_HPP_
