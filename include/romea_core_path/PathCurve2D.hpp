#ifndef romea_PathCurve2D_hpp
#define romea_PathCurve2D_hpp

//romea
#include "PathPosture2D.hpp"
#include "PathFrenetPose2D.hpp"
#include <romea_core_common/math/Interval.hpp>

//std
#include <vector>

namespace romea {


class PathCurve2D
{

public :

    using Vector = std::vector<double,Eigen::aligned_allocator<double> > ;

public :

  PathCurve2D();

  bool estimate(const Vector & X,
                const Vector & Y,
                const Vector & S ,
                const Interval<size_t> & indexInterval,
                const Interval<double> & curvilinearAbscissaInterval);

  bool findNearestCurvilinearAbscissa(const Eigen::Vector2d & vehiclePosition,
                                      double & curvilinearAbscissa) const;

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

}


#endif
