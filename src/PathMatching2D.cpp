//romea
#include "romea_path/PathMatching2D.hpp"
#include <romea_common/math/EulerAngles.hpp>

//std
#include <iterator>
#include <iostream>

namespace romea {

//-----------------------------------------------------------------------------
PathMatching2D::PathMatching2D():
  PathMatching2D(0)
{

}


//-----------------------------------------------------------------------------
PathMatching2D::PathMatching2D(const double & maximalResearchRadius):
  maximalResearchRadius_(maximalResearchRadius)
{

}

//-----------------------------------------------------------------------------
void PathMatching2D::setMaximalResearchRadius(const double & maximalResearchRadius)
{
  maximalResearchRadius_=maximalResearchRadius;
}

//-----------------------------------------------------------------------------
boost::optional<PathMatchedPoint2D> PathMatching2D::findMatchedPoint_(const Path2D & path,
                                                                      const Pose2D & vehiclePose,
                                                                      const size_t & nearestPointIndex)
{
  boost::optional<PathMatchedPoint2D> matchedPoint;
  double nearestCurvilinearAbscissa = path.getCurvilinearAbscissa()[nearestPointIndex];

//  std::cout <<"nearestPointIndex "<<nearestPointIndex<<std::endl;
//  std::cout <<"nearestCurvilinearAbscissa "<<nearestCurvilinearAbscissa<<std::endl;
//  std::cout <<" path length "<< path.getCurvilinearAbscissa().back()<<std::endl;

  const PathCurve2D & pathCurve = path.getCurves()[nearestPointIndex];
  if(pathCurve.findNearestCurvilinearAbscissa(vehiclePose.position,nearestCurvilinearAbscissa))
  {
    double xp = pathCurve.computeX(nearestCurvilinearAbscissa);
    double yp = pathCurve.computeY(nearestCurvilinearAbscissa);
    double tangent = pathCurve.computeTangent(nearestCurvilinearAbscissa);
    double curvature = pathCurve.computeCurvature(nearestCurvilinearAbscissa);

    const double & xv = vehiclePose.position.x();
    const double & yv = vehiclePose.position.y();
    const double & o = vehiclePose.yaw;
    const double & cost= std::cos(tangent);
    const double & sint = std::sin(tangent);

    double courseDeviation = betweenMinusPiAndPi(o-tangent);
    double lateralDeviation = (yv-yp)*cost-(xv-xp)*sint;

    Eigen::Matrix3d J=Eigen::Matrix3d::Identity();
    J.block<2,2>(0,0) = eulerAngleToRotation2D(courseDeviation);
    Eigen::Matrix3d frenetPoseCovariance = J*vehiclePose.covariance*J.transpose();

    size_t nearestPointIndex = findNearestPointIndex_(path,Eigen::Vector2d(xp,yp),pathCurve.getIndexRange());

    //Singularity
    if((std::abs(curvature) > 10e-6)&&(std::abs(lateralDeviation-(1/curvature))<=10e-6))
    {
      curvature = 0;
    }

    PathMatchedPoint2D matchedPointb;
    matchedPointb.pathPosture.position.x()=xp;
    matchedPointb.pathPosture.position.y()=yp;
    matchedPointb.pathPosture.course=tangent;
    matchedPointb.pathPosture.curvature=curvature;
    matchedPointb.frenetPose.curvilinearAbscissa=nearestCurvilinearAbscissa;
    matchedPointb.frenetPose.lateralDeviation=lateralDeviation;
    matchedPointb.frenetPose.courseDeviation=courseDeviation;
    matchedPointb.frenetPose.covariance=frenetPoseCovariance;
    matchedPointb.nearestPointIndex=nearestPointIndex;
    matchedPoint =matchedPointb;
  }

  return matchedPoint;
}

//-----------------------------------------------------------------------------
size_t PathMatching2D::findNearestPointIndex_(const Path2D & path,
                                              const Eigen::Vector2d & vehiclePosition,
                                              const Interval<size_t> indexRange)const
{

  //find neareast point on path
  const auto X = path.getX();
  const auto Y = path.getY();

  size_t nearestPointIndex= indexRange.upper();
  double minimalDistance = maximalResearchRadius_;

  for(size_t n=indexRange.lower();n<indexRange.upper();n++)
  {
    double distance = (vehiclePosition-Eigen::Vector2d(X[n],Y[n])).norm();
    if(distance < minimalDistance){
      minimalDistance=distance;
      nearestPointIndex=n;
    }
  }

  return nearestPointIndex;
}

//-----------------------------------------------------------------------------
double PathMatching2D::computeFutureCurvature(const Path2D & path,
                                              const PathMatchedPoint2D & matchedPoint,
                                              const double & linear_speed,
                                              const double &time_horizon)
{
  double futureCurvilinearAbscissa = linear_speed* time_horizon +
      matchedPoint.frenetPose.curvilinearAbscissa;

  size_t futurePointIndex = path.findNearestIndex(futureCurvilinearAbscissa);
  return path.getCurves()[futurePointIndex].computeCurvature(futureCurvilinearAbscissa);
}

//-----------------------------------------------------------------------------
boost::optional<PathMatchedPoint2D> PathMatching2D::match(const Path2D & path,
                                                          const Pose2D & vehiclePose)
{
  boost::optional<PathMatchedPoint2D> matchedPoint;

  //find neareast point on path
  const auto numberOfPoints = path.getX().size();
  const size_t nearestPointIndex = findNearestPointIndex_(path,
                                                          vehiclePose.position,
                                                          Interval<size_t>(0,numberOfPoints));

  //compute matched point
  if(nearestPointIndex!=numberOfPoints)
  {
    matchedPoint = findMatchedPoint_(path,
                                     vehiclePose,
                                     nearestPointIndex);

  }

  return matchedPoint;
}

//-----------------------------------------------------------------------------
boost::optional<PathMatchedPoint2D> PathMatching2D::match(const Path2D & path,
                                                          const Pose2D & vehiclePose,
                                                          const PathMatchedPoint2D & previousMatchedPoint,
                                                          const double & expectedTravelledDistance)
{
  Interval<size_t> rangeIndex =path.findMinMaxIndexes(previousMatchedPoint.nearestPointIndex,
                                                      expectedTravelledDistance);

  size_t nearestPointIndex = findNearestPointIndex_(path,
                                                    vehiclePose.position,
                                                    rangeIndex);

  return findMatchedPoint_(path,
                           vehiclePose,
                           nearestPointIndex);

}

}
